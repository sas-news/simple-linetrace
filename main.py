# ============================================================================
# SPIKE Prime 最強ライントレーサー  "LINE RACER"  (Pybricks 用)
# ----------------------------------------------------------------------------
# 概要:
#   1周目 = エクスプローララン(中速で完走し、弧長s基準で曲率マップ/誤差プロファイルを記録)
#   2周目以降 = 学習済み速度プロファイル + 曲率FF + ILC(反復学習)で毎周回タイム更新
#   スタートの「垂直な2本線」をゲートとして検出し、ラップタイムを計測・ベスト記録
#   無限周回し、ベストタイムを 5x5 ライトマトリクスと USB コンソールに出力
#
# ハードウェア (デフォルト配線):
#   Port A: 左モーター      Port B: 右モーター
#   Port C: 左カラーセンサー Port D: 右カラーセンサー (ラインを挟む差動配置)
#
# 使い方:
#   Pybricks Code でこのファイルを開き、ハブに書き込んで実行。
#   中央ボタンでスタート(10秒待つと自動スタート)。
#   停止: 中央+左+右 同時押し。
#   左ボタン: 周回終了時に直前ラップのテレメトリをUSBコンソールへダンプ。
#   右ボタン: 現在設定/統計を表示。
#
# 注意:
#   - 学習データはRAM上でラップ間を引き継ぎます(無限周回中は保持)。
#   - 符号関係(ステアリング/ジャイロ)が逆だと暴走します。READMEの
#     「初回セットアップ」を参照して SIGN_* を調整してください。
# ============================================================================

# --- 互換性: PC上での構文チェック用フォールバック ---------------------------
try:
    from micropython import native, mem_info
except ImportError:
    def native(f):
        return f

    def mem_info(*a, **k):
        pass

from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor
from pybricks.parameters import Port, Direction, Button, Color, Axis
from pybricks.tools import StopWatch, wait
from math import sqrt as _fsqrt

# ============================================================================
# 設定 (実機に合わせて調整)
# ============================================================================

# ---- 機体 ----------------------------------------------------------------
WHEEL_D_CM = 27.9       # ホイール直径 cm (転がり試験で実測推奨: 10回転の移動距離/10/pi)
TRACK_B_CM = 14.0       # 左右駆動輪のトレッド幅 cm (曲率フィードフォワード用)
BOOM_K = 200            # センサーブーム輸送遅延補償 (×1000, 0.2 = 200)。0で無効
PORT_LM = Port.A        # 左モーター
PORT_RM = Port.B        # 右モーター
PORT_LS = Port.C        # 左カラーセンサー (ラインの左)
PORT_RS = Port.D        # 右カラーセンサー (ラインの右)

# ---- 符号 (逆だと暴走。初回セットアップで要確認) ---------------------------
SIGN_ST = 1             # ステアリング: e<0(ラインが右)で右に曲がる = +1
SIGN_GYRO = 1           # ジャイロダンピング符号
SIGN_BOOM = 1           # ブーム補償符号

# ---- 制御ゲイン (e は ±1000 正規化誤差) -----------------------------------
KP = 90                 # 比例: cm/s per (1000 e)  ... st_P = KP*e/1000
KD = 30                 # 微分: cm/s per (1000 e/s) ... st_D = KD*de/1000
KI = 40                 # 積分: cm/s per (1000 e*s) ... st_I = KI*intg/1000000
GYRO = 300              # ジャイロ: cm/s per 1000 mdeg/s 差分 ... st_G = GYRO*dw/1000000
INTG_LIMIT = 250000     # 積分アンチワインドアップ (e*ms)
ST_MAX = 150            # ステアリング出力クランプ cm/s
FF_MAX = 150            # ILC フィードフォワード クランプ cm/s

# ---- 速度 ----------------------------------------------------------------
V_FIND = 30             # ゲート探索時速度 cm/s
V_LAP1 = 40             # 1周目(エクスプローラ)速度 cm/s
V_MIN = 12              # 最低速度 cm/s
V_MAX_INIT = 65         # 速度上限の初期値 cm/s (ラップ2の安全な出発点、良好ラップで+4成長)
MOTOR_MAX = 140         # 実測最高速の目安 cm/s (ホイールを浮かせて実測し設定)
HARD_MAX = 250          # 安全リミット cm/s (モーター指令クランプ)
A_ACC = 220             # 加速リミット cm/s^2
A_BRK = 350             # 減速リミット cm/s^2 (曲率先読みブレーキ)
A_LAT = 220             # 横加速度リミット cm/s^2 (グリップ限界、学習で適応)
A_LAT_MIN = 120
A_LAT_MAX = 450

# ---- 学習 ----------------------------------------------------------------
ILC_ALPHA_HI = 60        # ILC 学習率 ×1000 (誤差大のとき。0.06)
ILC_ALPHA_MID = 40       # ILC 学習率 ×1000 (通常。0.04。収束を速める)
ILC_ALPHA_LO = 25        # ILC 学習率 ×1000 (収束時。0.025)
KP_MIN, KP_MAX = 30, 160
KD_MIN, KD_MAX = 5, 80
GATE_DIP = 45            # ゲート手前の減速速度 cm/s
GATE_DIP_CM = 30         # ゲート手前で減速する距離 cm (読み飛ばし防止)
ROLLBACK_MARGIN = 103    # ベスト比でこの%を超えると「悪化」 (×100)
BAD_STREAK = 2           # 連続悪化この回数でベスト状態にロールバック
NEAR_BEST_MARGIN = 102   # ベスト比でこの%以内を「ベスト相当」とみなす (×100)
RECENT_N = 5             # 移動中央値に使う直近ラップ数

# ---- 局所ゲイン補正 (セクション単位) --------------------------------------
SEC_CM = 100             # セクション長 cm
N_SEC = 60               # セクション数 (60m 分)
GAIN_MIN, GAIN_MAX = 70, 130   # セクションゲイン倍率の範囲 (×100)

# ---- 局所探索 (E-greedy: ベスト基準で直線区間の速度を試す) ------------------
EXPLORE_START = 4        # このラップ以降に探索開始
EXPLORE_EVERY = 5        # このラップ間隔で探索
EXPLORE_LIFT = 10        # vmax を +10% 試す (×100)
EXPLORE_MIN_BINS = 100   # 探索対象の直線の最小長 (ビン=2cm → 200cm)
EXPLORE_MAX_BINS = 200   # 探索で保存する最大ビン数 (400cm)

# ---- スリップ検出 ----------------------------------------------------------
SLIP_THRESH = 25         # 指令ωと実測ωの乖離がこの%を超えるとスリップ
SLIP_CNT_LIMIT = 50      # 1ラップでこの回数以上で A_LAT 減

# ---- マップ/ログ メモリ ---------------------------------------------------
BIN_CM = 2              # 弧長ビン幅 cm
N_BINS = 3000           # ビン数 (60m 分)
LOG_N = 8192            # ラップログ サンプル数 (40Hzで204秒分)
LOG_MS = 25             # ログ間隔 ms (40Hz)
LA_BINS = 15            # 曲率ルックアヘッド ビン数 (30cm)

# ---- ゲート検出 (2本の垂直ライン: 白-黒-白-黒-白 パターン) ----------------
G_LINE_MIN = 1          # 1本のライン幅の最小 cm
G_LINE_MAX = 8          # 1本のライン幅の最大 cm
G_GAP_MIN = 3           # 2本のライン間隔の最小 cm (学習前のデフォルト)
G_GAP_MAX = 100         # 2本のライン間隔の最大 cm (学習前のデフォルト)
G_GAP_TOL = 50          # 学習した間隔の検証許容 ±% (×10 → 5割)
G_REARM = 30            # ゲート検出後、再アームまでの距離 cm
GATE_WIN = 100          # 2周目以降、ゲート検出を有効にする位置窓 ±cm
GATE_MISS_CM = 300      # この距離過ぎても未検出なら「読み飛ばし」と判定

# ---- バッテリー補償 -------------------------------------------------------
BATT_NOM = 7600         # 基準電圧 mV (2S Li-ion 公称)
COMP_MAX = 1300         # 補償率上限 ×1000 (1.3)

# ---- 起動時自動テスト (符号・モーター方向) --------------------------------
TEST_SPEED = 120        # テスト時のモーター速度 deg/s (~30cm/s)
TEST_MS = 300           # テスト1回の時間 ms
TEST_ACC_TH = 200       # 前進テストの判定閾値 mm/s (加速度積分)

# ---- ディスプレイ/ブザー --------------------------------------------------
DISPLAY_MS = 500        # ライトマトリクス更新間隔 ms
NEW_BEST_HOLD_MS = 3000 # ベストタイム表示ホールド ms
AUTO_DUMP_BEST = True   # ベスト更新ラップを自動ダンプ

# ============================================================================
# 固定テーブル (bytearray。ラップ間で引き継がれる学習データ)
# ============================================================================
# kappa   : i16 曲率 ×1000 [1/m]           (符号付き、FFと速度プロファイル用)
# ff      : i16 ILC フィードフォワード [cm/s @ v=100cm/s 基準] (速度正規化)
# best_ff : i16 ベストラップ時の ff スナップショット
# vmax    : u8  速度プロファイル [cm/s]
# best_vmax: u8 ベストラップ時の vmax スナップショット
# err_filt: i8  ラップ平均誤差 e×127/1000 (統計表示用)
# err_cnt : u8  誤差サンプル数
# err_sum : i16 誤差累積 (e±1000)
# vsum_bin: u16 ビンごとの速度累積 (cm/s、ILC速度正規化用)
# kappa_acc: i32 曲率累積 (κ×1000)
# kappa_cnt: u8  曲率サンプル数
# seg     : u8  セグメント種別 (0=直線 1=カーブ 2=ヘアピン)
# kp_mult / kd_mult: u8 セクションごとのゲイン倍率 (×100, 70..130)
# sec_zc / sec_esum / sec_cnt: セクションごとの統計 (発振/誤差)
kappa = bytearray(N_BINS * 2)
ff = bytearray(N_BINS * 2)
best_ff = bytearray(N_BINS * 2)
vmax = bytearray(N_BINS)
best_vmax = bytearray(N_BINS)
err_filt = bytearray(N_BINS)
err_cnt = bytearray(N_BINS)
err_sum = bytearray(N_BINS * 2)
vsum_bin = bytearray(N_BINS * 2)
kappa_acc = bytearray(N_BINS * 4)
kappa_cnt = bytearray(N_BINS)
seg = bytearray(N_BINS)

kp_mult = bytearray(N_SEC)
kd_mult = bytearray(N_SEC)
best_kp_mult = bytearray(N_SEC)   # ベストラップ時のセクションゲイン (ロールバック用)
best_kd_mult = bytearray(N_SEC)
sec_zc = bytearray(N_SEC * 2)     # u16 ゼロクロス数
sec_esum = bytearray(N_SEC * 4)   # i32 |e| 累積
sec_cnt = bytearray(N_SEC * 2)    # u16 サンプル数
for _i in range(N_SEC):
    kp_mult[_i] = 100
    kd_mult[_i] = 100
    best_kp_mult[_i] = 100
    best_kd_mult[_i] = 100

explore_saved = bytearray(EXPLORE_MAX_BINS)

# ラップログ (リングバッファ: s,e,v,w)
log_s = bytearray(LOG_N * 2)
log_e = bytearray(LOG_N)
log_v = bytearray(LOG_N)
log_w = bytearray(LOG_N * 2)

# ラップ開始時のテーブルリセット用ゼロバッファ (毎ラップ再利用、GC負荷回避)
ZERO1 = bytearray(N_BINS)
ZERO2 = bytearray(N_BINS * 2)
ZERO4 = bytearray(N_BINS * 4)


# ============================================================================
# ヘルパー
# ============================================================================

def clamp(v, lo, hi):
    if v < lo:
        return lo
    if v > hi:
        return hi
    return v


def isqrt(x):
    return int(_fsqrt(x))


def i16at(ba, i):
    j = i * 2
    v = ba[j] | (ba[j + 1] << 8)
    if v >= 32768:
        v -= 65536
    return v


def i16set(ba, i, v):
    j = i * 2
    ba[j] = v & 255
    ba[j + 1] = (v >> 8) & 255


def i32at(ba, i):
    j = i * 4
    v = ba[j] | (ba[j + 1] << 8) | (ba[j + 2] << 16) | (ba[j + 3] << 24)
    if v >= 0x80000000:
        v -= 0x100000000
    return v


def i32add(ba, i, v):
    j = i * 4
    x = ba[j] | (ba[j + 1] << 8) | (ba[j + 2] << 16) | (ba[j + 3] << 24)
    if x >= 0x80000000:
        x -= 0x100000000
    x += v
    ba[j] = x & 255
    ba[j + 1] = (x >> 8) & 255
    ba[j + 2] = (x >> 16) & 255
    ba[j + 3] = (x >> 24) & 255


def u16at(ba, i):
    j = i * 2
    return ba[j] | (ba[j + 1] << 8)


def u16add(ba, i, v):
    j = i * 2
    x = ba[j] | (ba[j + 1] << 8)
    x += v
    if x > 65535:
        x = 65535
    ba[j] = x & 255
    ba[j + 1] = (x >> 8) & 255


def percentile(hist, p):
    total = 0
    for h in hist:
        total += h
    target = total * p // 100
    acc = 0
    for i in range(101):
        acc += hist[i]
        if acc >= target:
            return i
    return 100


# ============================================================================
# 状態グローバル
# ============================================================================
hub = PrimeHub()
mL = Motor(PORT_LM, positive_direction=Direction.COUNTERCLOCKWISE)
mR = Motor(PORT_RM, positive_direction=Direction.CLOCKWISE)
csL = ColorSensor(PORT_LS)
csR = ColorSensor(PORT_RS)
sw = StopWatch()

# キャリブレーション (反射率 %)。line色が norm<500 に写るよう正規化する
calL_w, calL_b = 95, 10
calR_w, calR_b = 95, 10
wz_bias = 0
histL = [0] * 101
histR = [0] * 101

# 符号・方向 (起動時自動テストで設定。手動オーバーライドも可)
CMD_FLIP = 1             # モーター指令・エンコーダの反転 (両モーター逆向き時 -1)
SIGN_GYRO_CUR = SIGN_GYRO

# 制御状態
e_prev = 0
de_f = 0             # 微分の1次ローパス値 (ノイズ増幅対策)
intg = 0
v_cmd = 0            # 現在の速度指令 cm/s
v_glob_max = V_MAX_INIT
st_prev = 0
cmdL = 0
cmdR = 0
s_cm = 0             # 弧長 cm (ゲートで0リセット)
s_base_deg = 0       # 弧長原点 (モーター角度 deg)
vbat = BATT_NOM
comp = 1000          # バッテリー補償率 ×1000
scan_ph = 0
lost_ms = 0
e_hold = 0
zc_sign = 0          # 直前の誤差符号 (ゼロクロス計測用)
slip_cnt = 0         # スリップ検出回数 (ラップ内)
n_min_prev = 1000    # 直前の min(nL,nR) (ゲート通過時刻補間用)
gate_f = 0           # ゲート通過時刻の補間係数 ×100
gate_dt = 0          # ゲート通過時の tick 間隔 ms

# ゲート間隔学習
gate_gap_cm = 0      # 2本線の間隔 (学習値、0=未学習)
gate_gap_last = 0    # 直近の観測間隔

# ラップ状態
in_lap = False
lap_no = 0
lap_done = False
force_lap_end = False
slow_mode = False    # ゲート読み飛ばし後の低速探索モード
lap_valid = True     # このラップのデータが有効か (読み飛ばし時 False)
gate_s = 0
track_len = 0        # 1周の距離 cm (1周目終了後に確定)
track_drift = 0      # ゲート位置のドリフト方向カウンタ (EMA重み適応)
lap_start_t = 0
best_time = 0
best_lap = 0
best_kp = KP
best_kd = KD
best_a_lat = A_LAT
best_vglob = V_MAX_INIT
bad_streak = 0       # 連続悪化ラップ数 (ロールバック用)
recent_times = []    # 直近ラップタイム (移動中央値用)
disp_hold = 0
new_best_flag = False

# 局所探索 (E-greedy)
explore_active = False
explore_s0 = 0
explore_s1 = 0

# ループ/ハウスキーピング
t_prev = 0
v_req = 999
last_hk = 0
last_log = -1000
last_look = -1000
last_disp = -1000
last_batt = -1000
last_rate = -1000
tick_cnt = 0
rate_hz = 0
log_idx = 0
log_full = False
dump_enable = False
dump_idx = 0
dump_end = 0
dump_skip = 2
prev_left = False
prev_right = False

# ゲートマッチャー状態
gm_state = 0
gm_s0 = 0
gm_s1 = 0
gm_s2 = 0
gm_armed = True
gm_last_s = -1000
gm_gap = 0

# 学習パラメータ (適応あり)
A_LAT = A_LAT_INIT = 220
KP_cur = KP
KD_cur = KD

# ============================================================================
# キャリブレーション
# ============================================================================

def calib_stationary():
    """起動時: 1秒静止してジャイロバイアスと初期ヒストグラムを取得"""
    global wz_bias
    n = 0
    wzb = 0
    t0 = sw.time()
    while sw.time() - t0 < 1000:
        rl = csL.reflection()
        rr = csR.reflection()
        if histL[rl] < 60000:
            histL[rl] += 1
        if histR[rr] < 60000:
            histR[rr] += 1
        wzb += hub.imu.angular_velocity(Axis.Z)
        n += 1
    wz_bias = wzb // max(n, 1)
    print("calib: bias=%d n=%d" % (wz_bias, n))


# ============================================================================
# ゲートマッチャー (2本の垂直ライン: 白-黒-白-黒-白)
#   両センサーのOR信号でパターンを追跡し、距離検証で誤検出を排除する
# ============================================================================

def matcher_tick(s, line):
    """s: 現在の弧長 cm, line: 両センサ同時黒 (ゲートの垂直線)。通過位置(cm)を返す"""
    global gm_state, gm_s0, gm_s1, gm_s2, gm_armed, gm_last_s, gm_gap
    if not gm_armed:
        if s - gm_last_s > G_REARM:
            gm_armed = True
        else:
            return -1
    st = gm_state
    gate = -1
    if st == 0:
        if line:
            gm_s0 = s
            gm_state = 1
    elif st == 1:
        if not line:
            d = s - gm_s0
            if G_LINE_MIN <= d <= G_LINE_MAX:
                gm_s1 = s
                gm_state = 2
            else:
                gm_state = 0
    elif st == 2:
        if line:
            d = s - gm_s1
            if gate_gap_cm > 0:
                # 学習済み間隔で検証 (±50%)
                ok = (d >= gate_gap_cm * (100 - G_GAP_TOL) // 100) and \
                     (d <= gate_gap_cm * (100 + G_GAP_TOL) // 100)
            else:
                ok = G_GAP_MIN <= d <= G_GAP_MAX
            if ok:
                gm_s2 = s
                gm_state = 3
            else:
                gm_state = 0
    elif st == 3:
        if not line:
            d = s - gm_s2
            if G_LINE_MIN <= d <= G_LINE_MAX:
                gm_state = 0
                gm_armed = False
                gm_last_s = s
                gm_gap = gm_s2 - gm_s1   # 白の間隔 (学習用)
                gate = s
    return gate

# ============================================================================
# 制御ティック (ホットパス: ループごとに実行)
# ============================================================================

def _tick_impl(dt_ms, t_ms):
    """1回の制御周期。整数演算中心で GC 負荷を最小化する"""
    global e_prev, intg, v_cmd
    global s_cm, scan_ph, lost_ms, e_hold, de_f
    global last_look, v_req, last_log, log_idx, log_full
    global tick_cnt, gate_s, lap_done, zc_sign, slip_cnt, n_min_prev
    global gate_f, gate_dt, gate_gap_last

    # ---- センサー読み取り (Pybricks が 1kHz でキャッシュ更新) ----
    rl = csL.reflection()
    rr = csR.reflection()

    # ヒストグラム (キャリブレーション用、2ティックに1回)
    if not (t_ms & 1):
        hl = histL[rl]
        if hl < 60000:
            histL[rl] = hl + 1
        hr = histR[rr]
        if hr < 60000:
            histR[rr] = hr + 1

    # 正規化 (0..1000。ライン色が低値に写る)
    wl = calL_w - calL_b
    if wl < 1:
        wl = 1
    wr = calR_w - calR_b
    if wr < 1:
        wr = 1
    nL = (rl - calL_b) * 1000 // wl
    nR = (rr - calR_b) * 1000 // wr
    if nL < 0:
        nL = 0
    elif nL > 1000:
        nL = 1000
    if nR < 0:
        nR = 0
    elif nR > 1000:
        nR = 1000

    # 誤差: e<0 = ラインが右寄り → 右に曲がる (SIGN_ST で反転可)
    e = (nR - nL) * 1000 // (nL + nR + 1)
    if SIGN_ST < 0:
        e = -e

    # ジャイロ (mdeg/s、符号は起動時自動テストで確定)
    wz = hub.imu.angular_velocity(Axis.Z) - wz_bias
    if SIGN_GYRO_CUR < 0:
        wz = -wz

    # オドメトリ (CMD_FLIP でモーター方向を補正)
    aL = mL.angle() * CMD_FLIP
    aR = mR.angle() * CMD_FLIP
    # s_cm = 平均角 * pi*D/360 [cm]。pi*27.9/360 = 0.2435 → ×1000 = 244
    # (毎ティックの角度差は高ループレートで分解能未満のため速度推定には使わない)
    s_cm = ((aL + aR) // 2 - s_base_deg) * 244 // 1000
    # 注意: 毎ティックの ds は高ループレート(>40Hz)では分解能未満で常に0になり、
    #   エンコーダ実測速度(EMA)はほぼ0になる。κ/ブーム補償は指令速度 v_cmd を使う
    #   (FF/wref も v_cmd 基準なので自己整合する)

    # ライン有無 / ロスト判定
    # 両センサがラインを挟む配置では「両方白」= センター追従中の正常状態。
    # 「両方黒」のみをロスト扱い (ライン幅>センサ間隔 or 完全に線上の曖昧状態)
    line = (nL < 500) or (nR < 500)
    # ゲート検出用: 両センサ同時黒。垂直2本線(ゲート)は両方を暗くするが、
    # 主ライン(センサ間隔より狭い)は片側しか暗くしない → 蛇行による誤検出を排除
    line_both = (nL < 500) and (nR < 500)
    if nL < 200 and nR < 200:
        lost_ms += dt_ms
    else:
        lost_ms = 0
    lost = lost_ms > 300

    # 有効誤差 (輸送遅延補償: ブーム先端センサー→ホイール軸の横誤差)
    # 左旋回(wz>0)でブームが左へ振れる → センサーにはラインが右にずれて見える
    # → e を補正してホイール軸位置の真の誤差に戻す
    if not lost:
        comp_term = (BOOM_K * (wz // max(v_cmd, 20))) // 1000
        if SIGN_BOOM < 0:
            comp_term = -comp_term
        e_eff = e + comp_term
    else:
        e_eff = e_hold  # ロスト中は誤差フリーズ
    e_hold = e_eff

    # ---- 制御 ----
    b = s_cm // BIN_CM
    if b >= N_BINS:
        b = N_BINS - 1
    if b < 0:
        b = 0
    sec = b // (SEC_CM // BIN_CM)     # セクション番号 (局所ゲイン用)
    if sec >= N_SEC:
        sec = N_SEC - 1

    # 曲率・ILC 読み出し (in_lap かつマップあり、読み飛ばし探索中は不使用)
    j = b * 2
    k1000 = kappa[j] | (kappa[j + 1] << 8)
    if k1000 >= 32768:
        k1000 -= 65536
    ffb = ff[j] | (ff[j + 1] << 8)
    if ffb >= 32768:
        ffb -= 65536

    st = 0
    if in_lap and track_len > 0 and not slow_mode:
        # 幾何学的曲率FF: st = (vR-vL)/2 = κ*v*B/2 [cm/s]
        # B=14cm: st = (k1000/1000)*v*14/2 = k1000*v*7/100000
        st += ((k1000 * 7000) // 100000) * v_cmd // 1000
        # ILC 学習FF (v=100cm/s 基準で保存 → 現在速度でスケーリング)
        st += ffb * v_cmd // 100
    else:
        k1000 = 0
        ffb = 0

    # PID (セクションごとの局所ゲインを適用)
    # 微分: 1次ローパス (de_f) でノイズ増幅を抑制。de は高ループレートで
    # ノイズが st_D を飽和させ振動源になる (500Hzで ±25単位ノイズ → de≈12500)
    de = (e_eff - e_prev) * 1000 // max(dt_ms, 1)
    de_f = (de_f * 5 + de) // 6
    intg += e_eff * dt_ms
    if intg > INTG_LIMIT:
        intg = INTG_LIMIT
    elif intg < -INTG_LIMIT:
        intg = -INTG_LIMIT
    st += (KP_cur * kp_mult[sec] // 100 * e_eff) // 1000
    st += (KD_cur * kd_mult[sec] // 100 * de_f) // 1000
    st += (KI * intg) // 1000000

    # ジャイロダンピング (目標ヨーレート wref = v*κ [mdeg/s])
    # 注意: 単位は mdeg/s。κ[1/m]=k1000/1000, v[cm/s]=v_cmd/100 [m/s]
    #   wref = κ*v*57296 = k1000*v_cmd*57296/100000
    #   → (k1000*57296//1000)*v_cmd//100 で32bit範囲内・精度保持
    if in_lap and track_len > 0 and not slow_mode:
        wref = (k1000 * 57296 // 1000) * v_cmd // 100
    else:
        wref = 0
    dw = wref - wz
    # ラップ1/ゲート探索はマップ未学習のため wref=0 → 純減衰がカーブと戦う。
    # GYRO を半減し、定常誤差(→κマップ過小評価)とカーブ出口オーバーシュートを抑える
    gyr = GYRO // 2 if lap_no <= 1 else GYRO
    st += (dw * gyr) // 1000000

    if st > ST_MAX:
        st = ST_MAX
    elif st < -ST_MAX:
        st = -ST_MAX



    # ---- 速度指令 ----
    # ルックアヘッド: 前方 LA_BINS の最大曲率から許容速度を計算 (10msキャッシュ)
    if t_ms - last_look >= 10:
        last_look = t_ms
        ka = 0
        for jj in range(1, LA_BINS + 1):
            bb = b + jj
            if bb >= N_BINS:
                break
            j2 = bb * 2
            kk = kappa[j2] | (kappa[j2 + 1] << 8)
            if kk >= 32768:
                kk -= 65536
            if kk < 0:
                kk = -kk
            if kk > ka:
                ka = kk
        if ka > 0:
            v_req = isqrt(100 * A_LAT * 1000 // ka)
        else:
            v_req = 999

    if not in_lap:
        vt = V_FIND
    else:
        vt = v_req
        if lap_no == 1:
            if vt > V_LAP1:
                vt = V_LAP1
        elif not slow_mode:
            # 速度プロファイル (ゲート前減速もここに含まれる)
            lim = vmax[b]
            if vt > lim:
                vt = lim
        if vt > v_glob_max:
            vt = v_glob_max
    if vt < V_MIN:
        vt = V_MIN
    if slow_mode and vt > V_FIND:
        vt = V_FIND   # 読み飛ばし探索中は低速
    if lost and vt > V_FIND:
        vt = V_FIND   # ロスト中は低速でエッジ探索

    # 加減速ランプ
    acc = A_ACC if vt >= v_cmd else A_BRK
    dv = acc * dt_ms // 1000
    if vt > v_cmd + dv:
        v_cmd += dv
    elif vt < v_cmd - dv:
        v_cmd -= dv

    # ロスト時: 左右スキャン (速度は上で V_FIND に制限済み)
    if lost:
        scan_ph += dt_ms
        if (scan_ph // 500) & 1:
            st += 8
        else:
            st -= 8

    # バッテリー補償 (共通成分のみ)
    ve = v_cmd * comp // 1000

    # 左右指令 + モーター速度(deg/s)変換 (CMD_FLIP で方向補正)
    vL = ve - st
    vR = ve + st
    if vL > HARD_MAX:
        vL = HARD_MAX
    elif vL < -HARD_MAX:
        vL = -HARD_MAX
    if vR > HARD_MAX:
        vR = HARD_MAX
    elif vR < -HARD_MAX:
        vR = -HARD_MAX
    mL.run(vL * 411 // 100 * CMD_FLIP)
    mR.run(vR * 411 // 100 * CMD_FLIP)

    # スリップ検出: 指令ωと実測ωの乖離 (コーナリング時のみ)
    if in_lap and lap_valid and not slow_mode:
        wcmd = (vR - vL) * 57296 // 14   # 指令ヨーレート mdeg/s (B=14cm)
        if wcmd > 20000 or wcmd < -20000:
            if abs(wz - wcmd) > abs(wcmd) * SLIP_THRESH // 100:
                slip_cnt += 1

    # ---- 学習データ蓄積 (in_lap かつ有効ラップ、ロスト中は除外) ----
    # ILC/セクション学習は PID と同じ e_eff (ブーム補償済み誤差) で行う。
    # 生の e だとブーム輸送遅延分が系統誤差として残り、FF が過補償になる。
    if in_lap and lap_valid and not lost:
        if kappa_cnt[b] < 200:
            # κ×1000 = (wz//v) * 17453 // 10000  (mdeg/s → 1/m)
            i32add(kappa_acc, b, ((wz // max(v_cmd, 5)) * 17453) // 10000)
            kappa_cnt[b] += 1
        if err_cnt[b] < 20:
            x = err_sum[b * 2] | (err_sum[b * 2 + 1] << 8)
            if x >= 32768:
                x -= 65536
            x += e_eff
            if x > 32767:
                x = 32767
            elif x < -32768:
                x = -32768
            err_sum[b * 2] = x & 255
            err_sum[b * 2 + 1] = (x >> 8) & 255
            # ビンごとの速度累積 (ILC速度正規化用)
            vx = vsum_bin[b * 2] | (vsum_bin[b * 2 + 1] << 8)
            vx += v_cmd
            if vx > 65535:
                vx = 65535
            vsum_bin[b * 2] = vx & 255
            vsum_bin[b * 2 + 1] = (vx >> 8) & 255
            err_cnt[b] += 1
        # セクション統計 (局所ゲイン学習用)
        if e_eff > 40 or e_eff < -40:
            sgn = 1 if e_eff > 0 else -1
            if zc_sign != 0 and sgn != zc_sign:
                u16add(sec_zc, sec, 1)
            zc_sign = sgn
        u16add(sec_cnt, sec, 1)
        ae = e_eff
        if ae < 0:
            ae = -ae
        i32add(sec_esum, sec, ae)

    # ---- ゲート検出 (位置窓: 2周目以降は track_len±GATE_WIN のみ) ----
    if not lap_done:
        run_matcher = True
        if in_lap and lap_no > 1 and track_len > 0 and not slow_mode:
            if abs(s_cm - track_len) > GATE_WIN:
                run_matcher = False
        if run_matcher:
            # ラップ1/探索中は AND(両センサ同時黒) で誤検出を排除。
            # ラップ2以降は位置窓(|s-track_len|<=GATE_WIN)が誤検出を拒否するため、
            # OR(片側黒) で細いゲートラインも見逃さない
            mat_sig = line if (in_lap and lap_no > 1 and not slow_mode) else line_both
            g = matcher_tick(s_cm, mat_sig)
            if g > 0:
                gate_gap_last = gm_gap
                if in_lap and lap_no > 1 and not slow_mode:
                    tol = max(50, track_len // 30)
                    if abs(g - track_len) <= tol:
                        gate_s = g
                        lap_done = True
                    # 不一致なら誤検出として無視 (マッチャーは再アーム済み)
                else:
                    gate_s = g
                    lap_done = True
                if lap_done:
                    # ゲート通過時刻の補間 (G5)
                    n_min = nL if nL < nR else nR
                    den = n_min - n_min_prev
                    if den > 0:
                        f = (500 - n_min_prev) * 100 // den
                    elif den < 0:
                        f = 100
                    else:
                        f = 100
                    if f < 0:
                        f = 0
                    elif f > 100:
                        f = 100
                    gate_f = f
                    gate_dt = dt_ms

    n_min = nL if nL < nR else nR
    n_min_prev = n_min

    # ---- ラップログ (40Hz) ----
    if t_ms - last_log >= LOG_MS and not log_full:
        last_log = t_ms
        if log_idx < LOG_N:
            si = log_idx * 2
            s16 = s_cm & 0xFFFF
            log_s[si] = s16 & 255
            log_s[si + 1] = (s16 >> 8) & 255
            le = e * 127 // 1000
            if le > 127:
                le = 127
            elif le < -127:
                le = -127
            log_e[log_idx] = le & 255
            lv = v_cmd
            if lv > 255:
                lv = 255
            elif lv < 0:
                lv = 0
            log_v[log_idx] = lv
            lw = wz
            if lw > 32000:
                lw = 32000
            elif lw < -32000:
                lw = -32000
            log_w[si] = lw & 255
            log_w[si + 1] = (lw >> 8) & 255
            log_idx += 1
        else:
            log_full = True

    tick_cnt += 1
    e_prev = e_eff


# ============================================================================
# ハウスキーピング (10ms 周期: バッテリー/ボタン/表示/ダンプ/レート)
# ============================================================================

def hk(t_ms):
    global last_batt, comp, vbat, last_disp, last_rate, rate_hz
    global dump_enable, dump_idx, dump_end, prev_left, prev_right
    global tick_cnt

    # バッテリー補償 (100ms)
    if t_ms - last_batt >= 100:
        last_batt = t_ms
        vbat = hub.battery.voltage()
        c = BATT_NOM * 1000 // max(vbat, 6000)
        if c > COMP_MAX:
            c = COMP_MAX
        elif c < 1000:
            c = 1000
        comp = c

    # ループレート (1s)
    if t_ms - last_rate >= 1000:
        last_rate = t_ms
        rate_hz = tick_cnt
        tick_cnt = 0

    # ボタン (エッジ検出)
    pressed = hub.buttons.pressed()
    left = Button.LEFT in pressed
    right = Button.RIGHT in pressed
    if left and not prev_left:
        dump_enable = not dump_enable
        print("dump_enable=%d" % (1 if dump_enable else 0))
    if right and not prev_right:
        print_stats()
    prev_left = left
    prev_right = right

    # 表示 (500ms)
    if t_ms - last_disp >= DISPLAY_MS:
        last_disp = t_ms
        if t_ms < disp_hold:
            hub.display.number(best_time // 100)  # ベストタイム[s×10]
        elif in_lap:
            n = (t_ms - lap_start_t) // 100
            if n > 9999:
                n = 9999
            hub.display.number(n)  # 経過秒×10
        else:
            hub.display.off()

    # ダンプ (ラップ終了後、最大10行/10ms)
    if dump_enable and dump_idx < dump_end:
        for _ in range(10):
            if dump_idx >= dump_end:
                break
            si = dump_idx * 2
            s16 = log_s[si] | (log_s[si + 1] << 8)
            e8 = log_e[dump_idx]
            if e8 > 127:
                e8 -= 256
            w16 = log_w[si] | (log_w[si + 1] << 8)
            if w16 >= 32768:
                w16 -= 65536
            print("%d,%d,%d,%d" % (s16, e8, log_v[dump_idx], w16))
            dump_idx += dump_skip
    if dump_end > 0 and dump_idx >= dump_end:
        dump_enable = False
        dump_end = 0
        print("#END")


def print_stats():
    print("stats: kp=%d kd=%d a_lat=%d vmax_glob=%d track=%d best=%d lap=%d rate=%dHz comp=%d gap=%d slip=%d" %
          (KP_cur, KD_cur, A_LAT, v_glob_max, track_len, best_time, lap_no, rate_hz, comp,
           gate_gap_cm, slip_cnt))


def bench():
    """起動時ベンチマーク: ループレートとカラーセンサーの実効更新レート測定"""
    t0 = sw.time()
    n = 0
    chg = 0
    rl0 = -1
    rr0 = -1
    while sw.time() - t0 < 1000:
        rl = csL.reflection()
        rr = csR.reflection()
        if rl != rl0 or rr != rr0:
            chg += 1
            rl0 = rl
            rr0 = rr
        hub.imu.angular_velocity(Axis.Z)
        mL.angle()
        mR.angle()
        n += 1
    print("bench: %d Hz loop, %d sensor updates/s" % (n, chg))


# ============================================================================
# ラップ統計
# ============================================================================

def lap_stats():
    """ラップログから統計を計算 (rms_e, max_e, v_avg, dist_cm, mx_p99)"""
    n = log_idx
    if n < 2:
        return 0, 0, 0, 0, 0
    rms = 0
    mx = 0
    vsum = 0
    max_s = 0
    ehist = [0] * 128   # |e8| ヒストグラム (p99 用)
    for i in range(n):
        si = i * 2
        s16 = log_s[si] | (log_s[si + 1] << 8)
        if s16 > max_s:
            max_s = s16
        e8 = log_e[i]
        if e8 > 127:
            e8 -= 256
        rms += e8 * e8
        if e8 < 0:
            e8 = -e8
        if e8 > mx:
            mx = e8
        ehist[e8] += 1
        vsum += log_v[i]
    rms_e = isqrt(rms // n) * 1000 // 127   # i8(±127) → 制御スケール(±1000)へ
    mx = mx * 1000 // 127
    # p99 (上位1%誤差。単発ノイズに影響されない)
    target = n * 99 // 100
    acc = 0
    p99 = 0
    for i in range(128):
        acc += ehist[i]
        if acc >= target:
            p99 = i
            break
    mx_p99 = p99 * 1000 // 127
    return rms_e, mx, vsum // n, max_s, mx_p99


# ============================================================================
# ラップ終了処理 (学習コア)
# ============================================================================

def smooth_i16(ba, half):
    """±half ビンの箱型平滑化 (in-place)"""
    tmp = bytearray(len(ba))
    n = len(ba) // 2
    for i in range(n):
        acc = 0
        c = 0
        for j in range(i - half, i + half + 1):
            if 0 <= j < n:
                acc += i16at(ba, j)
                c += 1
        i16set(tmp, i, acc // c)
    ba[:] = tmp


def smooth_u8(ba, half):
    """±half ビンの箱型平滑化 (u8, in-place)"""
    tmp = bytearray(len(ba))
    n = len(ba)
    for i in range(n):
        acc = 0
        c = 0
        for j in range(i - half, i + half + 1):
            if 0 <= j < n:
                acc += ba[j]
                c += 1
        tmp[i] = acc // c
    ba[:] = tmp


def finalize_lap(t_lap):
    """ラップ終了時: 統計・キャリブレーション・学習更新・ベストタイム管理"""
    global track_len, track_drift, best_time, best_lap, disp_hold, new_best_flag
    global calL_w, calL_b, calR_w, calR_b, A_LAT, KP_cur, KD_cur, v_glob_max
    global dump_idx, dump_end, gate_gap_cm, bad_streak
    global explore_active, explore_s0, explore_s1, slow_mode
    global best_kp, best_kd, best_a_lat, best_vglob   # 代入あり → 必須

    rms_e, mx_e, v_avg, dist, mx_p99 = lap_stats()

    # 発振指数 (tick で計測したゼロクロス数から)
    total_zc = 0
    for _i in range(N_SEC):
        total_zc += u16at(sec_zc, _i)
    osc = total_zc * 1000 // max(dist, 1)

    # ---- 局所探索の判定 (vmax 再構築の前に行う) ----
    if explore_active:
        if t_lap <= best_time * 101 // 100:
            print("explore accepted")
        else:
            vmax[explore_s0:explore_s1] = explore_saved[0:explore_s1 - explore_s0]
            print("explore rejected")
        explore_active = False

    # ---- キャリブレーション更新 (ヒストグラム p2/p98) ----
    wL = percentile(histL, 98)
    bL = percentile(histL, 2)
    wR = percentile(histR, 98)
    bR = percentile(histR, 2)
    if wL - bL >= 20:
        if lap_no == 1:
            calL_w, calL_b = wL, bL
        else:
            calL_w = (calL_w + wL) // 2
            calL_b = (calL_b + bL) // 2
    if wR - bR >= 20:
        if lap_no == 1:
            calR_w, calR_b = wR, bR
        else:
            calR_w = (calR_w + wR) // 2
            calR_b = (calR_b + bR) // 2

    if not lap_valid:
        # ゲート読み飛ばし: 学習せず状態リセットのみ (タイムも記録しない)
        print("LAP %d invalid (gate miss) - learning skipped" % lap_no)
        if dump_enable and log_idx > 0:
            dump_idx = 0
            dump_end = log_idx
            print("#LAP lap_no %d time_ms %d best_ms %d dist_cm %d rms_e %d max_e %d osc %d vavg %d rate_hz %d" %
                  (lap_no, t_lap, best_time, dist, rms_e, mx_e, osc, v_avg, rate_hz))
            print("#DATA s_cm,e,v_cmps,w_mdeg_s")
        slow_mode = False
        return

    # ---- ゲート間隔の学習 (2本線の白ギャップ) ----
    if gate_gap_last > 0:
        if gate_gap_cm == 0:
            gate_gap_cm = gate_gap_last
        else:
            gate_gap_cm = (gate_gap_cm * 3 + gate_gap_last) // 4

    # ---- トラック長 (ドリフト検出付き EMA) ----
    if lap_no == 1:
        track_len = gate_s
        track_drift = 0
    else:
        d = gate_s - track_len
        if d > 20:
            track_drift += 1
        elif d < -20:
            track_drift -= 1
        else:
            track_drift = 0
        if track_drift > 3:
            track_drift = 3
        elif track_drift < -3:
            track_drift = -3
        w = 4 if abs(track_drift) >= 3 else 20
        track_len = (track_len * (w - 1) + gate_s) // w
    tb = int(track_len) // BIN_CM
    if tb >= N_BINS:
        tb = N_BINS - 1

    # ---- 曲率確定 + 平滑化 (κ×1000、i16 クランプ付き) ----
    for i in range(N_BINS):
        c = kappa_cnt[i]
        if c > 0:
            v = i32at(kappa_acc, i) // c
            if v > 32000:          # 低速急旋回で i16 を壊さないようクランプ (B1)
                v = 32000
            elif v < -32000:
                v = -32000
            old = i16at(kappa, i)
            i16set(kappa, i, (old + v * 3) // 4)  # 新測定に重み (1周目は75%反映)
    smooth_i16(kappa, 5)

    # ---- ILC 更新 (速度正規化 + 適応α): ff += α * e_avg * 100 / v_bin ----
    if rms_e > 250:
        alpha = ILC_ALPHA_HI
    elif rms_e > 150:
        alpha = ILC_ALPHA_MID
    else:
        alpha = ILC_ALPHA_LO
    ilc_rms = 0
    for i in range(N_BINS):
        c = err_cnt[i]
        if c >= 2:
            e_avg = i16at(err_sum, i) // c          # ±1000
            vb = i16at(vsum_bin, i) // c            # ビン平均速度 cm/s
            if vb < 10:
                vb = 10
            f = i16at(ff, i)
            f += e_avg * alpha // (10 * vb)         # v=100cm/s 基準に正規化
            if f > FF_MAX:
                f = FF_MAX
            elif f < -FF_MAX:
                f = -FF_MAX
            i16set(ff, i, f)
            ef = e_avg * 127 // 1000
            if ef > 127:
                ef = 127
            elif ef < -127:
                ef = -127
            err_filt[i] = ef & 255
            ilc_rms += f * f
    smooth_i16(ff, 2)
    ilc_rms = isqrt(ilc_rms // max(N_BINS, 1))

    # ---- セグメント分類 (0=直線 1=カーブ 2=ヘアピン) ----
    for i in range(N_BINS):
        k = i16at(kappa, i)
        if k < 0:
            k = -k
        if k < 600:
            seg[i] = 0
        elif k < 2500:
            seg[i] = 1
        else:
            seg[i] = 2

    # ---- セクション局所ゲイン学習 (ラップ3以降、P3) ----
    if lap_no >= 3:
        for i in range(N_SEC):
            c = u16at(sec_cnt, i)
            if c < 20:
                continue
            osc_s = u16at(sec_zc, i) * 1000 // c
            rms_s = i32at(sec_esum, i) // c
            if osc_s > 150:          # この区間で発振 → KP減 / KD増
                kp_mult[i] = kp_mult[i] * 9 // 10
                if kp_mult[i] < GAIN_MIN:
                    kp_mult[i] = GAIN_MIN
                kd_mult[i] = kd_mult[i] * 11 // 10
                if kd_mult[i] > GAIN_MAX:
                    kd_mult[i] = GAIN_MAX
            elif rms_s > 300:        # 追従遅れ → KP増
                kp_mult[i] = kp_mult[i] * 105 // 100
                if kp_mult[i] > GAIN_MAX:
                    kp_mult[i] = GAIN_MAX

    # ---- 横加速度限界の適応 (グリップ学習、p99誤差ベース) ----
    # 実機のセンサノイズ/ブーム遅延を考慮し、余裕判定は p99<250 で成長
    if mx_p99 > 450:                 # 外れが大きい = グリップ超過
        A_LAT = A_LAT * 92 // 100
    elif mx_p99 < 250 and osc < 2:   # 余裕あり → 成長
        A_LAT = A_LAT * 103 // 100
    if A_LAT < A_LAT_MIN:
        A_LAT = A_LAT_MIN
    elif A_LAT > A_LAT_MAX:
        A_LAT = A_LAT_MAX

    # ---- スリップ検出 → A_LAT 抑制 (H3) ----
    if slip_cnt > SLIP_CNT_LIMIT:
        A_LAT = A_LAT * 95 // 100
        if A_LAT < A_LAT_MIN:
            A_LAT = A_LAT_MIN
        print("slip detected: %d ticks" % slip_cnt)

    # ---- ベスト相当判定 (移動中央値ベース、S5/S6) ----
    recent_times.append(t_lap)
    if len(recent_times) > RECENT_N:
        recent_times.pop(0)
    if len(recent_times) >= 3:
        med = sorted(recent_times)[len(recent_times) // 2]
        near_best = t_lap <= med * NEAR_BEST_MARGIN // 100
    else:
        near_best = (best_time == 0) or (t_lap <= best_time * 103 // 100)

    # ---- 速度プロファイル再構築 ----
    # v_calc = sqrt(100*A_LAT*1000/|κ|)。κ=0 は上限
    for i in range(N_BINS):
        k = i16at(kappa, i)
        if k < 0:
            k = -k
        if k > 0:
            vc = isqrt(100 * A_LAT * 1000 // k)
            if vc > 255:
                vc = 255     # u8 バッファ (bytearray) を壊さないようクランプ
        else:
            vc = 255
        vmax[i] = (vmax[i] + vc * 2) // 3   # 新計算に重み (1周目は67%反映)
    # 誤差ベースの局所適応 (加速はベスト相当ラップのみ)
    for i in range(N_BINS):
        c = err_cnt[i]
        if c >= 2:
            e_avg = i16at(err_sum, i) // c
            k = i16at(kappa, i)
            if k < 0:
                k = -k
            if e_avg < 0:
                e_avg = -e_avg
            if e_avg < 150 and k < 800:
                if near_best and vmax[i] < 250:
                    vmax[i] += 2              # 直線で余裕があれば加速
            elif e_avg > 400:
                vmax[i] = vmax[i] * 9 // 10   # 外れが大きい区間は減速 (常時)
    # 加速/減速制約 (前方・後方パス×2)
    for _pass in range(2):
        for i in range(1, N_BINS):
            lim = isqrt(vmax[i - 1] * vmax[i - 1] + 2 * A_ACC * BIN_CM)
            if vmax[i] > lim:
                vmax[i] = lim
        for i in range(N_BINS - 2, -1, -1):
            lim = isqrt(vmax[i + 1] * vmax[i + 1] + 2 * A_BRK * BIN_CM)
            if vmax[i] > lim:
                vmax[i] = lim
    smooth_u8(vmax, 2)
    # ゲート前減速 (読み飛ばし防止)
    dip0 = tb - GATE_DIP_CM // BIN_CM
    if dip0 < 0:
        dip0 = 0
    for i in range(dip0, min(tb + 1, N_BINS)):
        if vmax[i] > GATE_DIP:
            vmax[i] = GATE_DIP
    for i in range(N_BINS):
        if vmax[i] < V_MIN:
            vmax[i] = V_MIN

    # ---- ゲイン適応 (発振/追従性、ラップ全体) ----
    if osc > 3:
        KP_cur = KP_cur * 9 // 10
        KD_cur = KD_cur * 11 // 10
    elif osc < 1 and rms_e > 150:
        KP_cur = KP_cur * 105 // 100
    if KP_cur < KP_MIN:
        KP_cur = KP_MIN
    elif KP_cur > KP_MAX:
        KP_cur = KP_MAX
    if KD_cur < KD_MIN:
        KD_cur = KD_MIN
    elif KD_cur > KD_MAX:
        KD_cur = KD_MAX

    # ---- 速度上限適応 (成長はベスト相当ラップのみ) ----
    # 良好時は +4cm/s/周 とやや積極的に (毎周のタイム改善を狙う)。
    # 実機のノイズを考慮し成長閾値は rms_e<130 (誤差13%未満) に緩和
    if near_best and rms_e < 130 and osc < 2:
        v_glob_max += 4
    elif rms_e > 300 or osc > 4:
        v_glob_max = v_glob_max * 95 // 100
    if v_glob_max > MOTOR_MAX:
        v_glob_max = MOTOR_MAX
    elif v_glob_max < V_LAP1:
        v_glob_max = V_LAP1

    # ---- ベストタイム / ロールバック (S3) ----
    new_best_flag = False
    if best_time == 0 or t_lap < best_time:
        best_time = t_lap
        best_lap = lap_no
        best_ff[:] = ff
        best_vmax[:] = vmax
        best_kp_mult[:] = kp_mult
        best_kd_mult[:] = kd_mult
        best_kp = KP_cur
        best_kd = KD_cur
        best_a_lat = A_LAT
        best_vglob = v_glob_max
        bad_streak = 0
        new_best_flag = True
        disp_hold = sw.time() + NEW_BEST_HOLD_MS
        hub.light.on(Color.GREEN)
        hub.speaker.beep(1200, 120)
        hub.speaker.beep(1600, 120)
    else:
        if t_lap > best_time * ROLLBACK_MARGIN // 100:
            bad_streak += 1
        else:
            bad_streak = 0
        if lap_no > 2 and bad_streak >= BAD_STREAK:
            ff[:] = best_ff
            vmax[:] = best_vmax
            kp_mult[:] = best_kp_mult
            kd_mult[:] = best_kd_mult
            KP_cur = best_kp
            KD_cur = best_kd
            A_LAT = best_a_lat
            v_glob_max = best_vglob
            bad_streak = 0
            print("rollback to best lap state")

    # ---- 局所探索の予約 (S4: 直線区間の vmax を次ラップ +10% 試す) ----
    if lap_no >= EXPLORE_START and (lap_no - EXPLORE_START) % EXPLORE_EVERY == 0:
        best_start = -1
        best_end = 0
        best_avg = 0
        i = 0
        while i < N_BINS - 1:
            k = i16at(kappa, i)
            if k < 0:
                k = -k
            if k < 300:
                j = i
                while j < N_BINS:
                    k2 = i16at(kappa, j)
                    if k2 < 0:
                        k2 = -k2
                    if k2 >= 300:
                        break
                    j += 1
                # ゲート帯を除外 (tb±100ビン) し、長さ制限
                if j - i >= EXPLORE_MIN_BINS and (j <= tb - 100 or i >= tb + 100) \
                        and j - i <= EXPLORE_MAX_BINS:
                    avg = 0
                    for kk in range(i, j):
                        avg += vmax[kk]
                    avg //= (j - i)
                    if avg > best_avg:
                        best_avg = avg
                        best_start = i
                        best_end = j
                i = j
            else:
                i += 1
        if best_start >= 0:
            n_save = best_end - best_start
            explore_saved[0:n_save] = vmax[best_start:best_end]
            for kk in range(best_start, best_end):
                v = vmax[kk] * (100 + EXPLORE_LIFT) // 100
                if v > 250:
                    v = 250
                vmax[kk] = v
            explore_s0 = best_start
            explore_s1 = best_end
            explore_active = True
            print("explore: section %d-%dcm vmax+%d%%" %
                  (best_start * BIN_CM, best_end * BIN_CM, EXPLORE_LIFT))

    # ---- 出力 ----
    print("LAP %d done: t=%dms best=%dms dist=%dcm rms_e=%d max_e=%d p99=%d osc=%d vavg=%d rate=%dHz" %
          (lap_no, t_lap, best_time, dist, rms_e, mx_e, mx_p99, osc, v_avg, rate_hz))
    print("  a_lat=%d kp=%d kd=%d vmax_glob=%d track=%d gap=%d ilc_rms=%d slip=%d cal=(%d,%d,%d,%d)" %
          (A_LAT, KP_cur, KD_cur, v_glob_max, track_len, gate_gap_cm, ilc_rms,
           slip_cnt, calL_w, calL_b, calR_w, calR_b))

    # ダンプ準備 (手動 or ベスト更新時の自動ダンプ)
    if (dump_enable or (new_best_flag and AUTO_DUMP_BEST)) and log_idx > 0:
        dump_idx = 0
        dump_end = log_idx
        print("#LAP lap_no %d time_ms %d best_ms %d dist_cm %d rms_e %d max_e %d osc %d vavg %d rate_hz %d" %
              (lap_no, t_lap, best_time, dist, rms_e, mx_e, osc, v_avg, rate_hz))
        print("#CFG wheel %d track_b %d bin %d bins %d log_n %d a_lat %d kp %d kd %d vmax_glob %d" %
              (int(WHEEL_D_CM * 10), int(TRACK_B_CM * 10), BIN_CM, N_BINS, LOG_N,
               A_LAT, KP_cur, KD_cur, v_glob_max))
        print("#DATA s_cm,e,v_cmps,w_mdeg_s")


# ネイティブコード化 (失敗時はインタープリタ実行にフォールバック)
try:
    tick = native(_tick_impl)
except Exception:
    tick = _tick_impl
try:
    finalize_lap = native(finalize_lap)
except Exception:
    pass

# ============================================================================
# メイン状態機械
# ============================================================================

def reset_lap_state():
    """ラップ開始時の状態リセット (テーブル蓄積部のみクリア)"""
    global log_idx, log_full, last_log, gm_state, gm_armed, gm_last_s
    global intg, e_prev, v_cmd, lost_ms, lap_done, force_lap_end, s_base_deg, t_prev
    global slow_mode, lap_valid, zc_sign, slip_cnt, gate_f, gate_dt, n_min_prev, de_f
    log_idx = 0
    log_full = False
    last_log = -1000
    gm_state = 0
    gm_armed = True
    gm_last_s = -1000
    intg = 0
    e_prev = 0
    de_f = 0
    v_cmd = GATE_DIP       # ゲート通過速度から再加速 (B2)
    lost_ms = 0
    lap_done = False
    force_lap_end = False
    slow_mode = False
    lap_valid = True
    zc_sign = 0
    slip_cnt = 0
    gate_f = 0
    gate_dt = 0
    n_min_prev = 1000
    t_prev = sw.time()
    s_base_deg = (mL.angle() + mR.angle()) // 2 * CMD_FLIP
    kappa_acc[:] = ZERO4
    kappa_cnt[:] = ZERO1
    err_sum[:] = ZERO2
    err_cnt[:] = ZERO1
    vsum_bin[:] = ZERO2
    sec_zc[:] = b"\x00" * (N_SEC * 2)
    sec_esum[:] = b"\x00" * (N_SEC * 4)
    sec_cnt[:] = b"\x00" * (N_SEC * 2)
    histL[:] = [0] * 101
    histR[:] = [0] * 101


def check_devices():
    """起動時: 各ポートの接続確認 (配線ミスの早期発見)"""
    checks = (("LeftMotor", mL.angle), ("RightMotor", mR.angle),
              ("LeftSensor", csL.reflection), ("RightSensor", csR.reflection))
    ok = True
    for name, fn in checks:
        try:
            fn()
            print("%s: OK" % name)
        except Exception:
            print("%s: NOT CONNECTED!" % name)
            ok = False
    return ok


def auto_sign_test():
    """起動時: モーター方向(CMD_FLIP)とジャイロ符号(SIGN_GYRO)を自動判定"""
    global CMD_FLIP, SIGN_GYRO_CUR
    print("auto sign test - keep clear space around robot")
    # 1) 前進/後退テスト (加速度積分の差で方向判定、重力オフセットは差で相殺)
    a_plus = 0
    t0 = sw.time()
    while sw.time() - t0 < TEST_MS:
        mL.run(TEST_SPEED)
        mR.run(TEST_SPEED)
        a_plus += hub.imu.acceleration(Axis.X)
        wait(2)
    a_minus = 0
    t0 = sw.time()
    while sw.time() - t0 < TEST_MS:
        mL.run(-TEST_SPEED)
        mR.run(-TEST_SPEED)
        a_minus += hub.imu.acceleration(Axis.X)
        wait(2)
    mL.run(0)
    mR.run(0)
    net = a_plus - a_minus
    if net < -TEST_ACC_TH:
        CMD_FLIP = -1
        print("motors reversed -> CMD_FLIP=-1")
    else:
        CMD_FLIP = 1
        if net < TEST_ACC_TH:
            print("WARN: forward test ambiguous (net=%d)" % net)
    # 2) スピンテスト (st>0 で左旋回するはず。wz の符号からジャイロ配置を判定)
    w1 = 0
    n1 = 0
    t0 = sw.time()
    while sw.time() - t0 < TEST_MS:
        mL.run(-TEST_SPEED * CMD_FLIP)
        mR.run(TEST_SPEED * CMD_FLIP)
        w1 += hub.imu.angular_velocity(Axis.Z)
        n1 += 1
        wait(2)
    w2 = 0
    n2 = 0
    t0 = sw.time()
    while sw.time() - t0 < TEST_MS:
        mL.run(TEST_SPEED * CMD_FLIP)
        mR.run(-TEST_SPEED * CMD_FLIP)
        w2 += hub.imu.angular_velocity(Axis.Z)
        n2 += 1
        wait(2)
    mL.run(0)
    mR.run(0)
    w1 = w1 // max(n1, 1)
    w2 = w2 // max(n2, 1)
    if w1 > 5000 and w2 < -5000:
        SIGN_GYRO_CUR = 1
    elif w1 < -5000 and w2 > 5000:
        SIGN_GYRO_CUR = -1
    else:
        print("WARN: spin test ambiguous (w1=%d w2=%d) - using defaults" % (w1, w2))
        return
    print("sign test: CMD_FLIP=%d SIGN_GYRO=%d" % (CMD_FLIP, SIGN_GYRO_CUR))


def wait_for_start():
    """中央ボタンで開始 (10秒待つと自動開始)"""
    print("Press CENTER to start (auto in 10s)")
    t0 = sw.time()
    while sw.time() - t0 < 10000:
        if Button.CENTER in hub.buttons.pressed():
            break
        wait(50)


def find_gate():
    """スタートゲート(2本の垂直ライン)を探して通過する"""
    global in_lap, t_prev
    in_lap = False
    t_prev = sw.time()   # 最初の tick で巨大な dt が積分に跳ねないように初期化
    print("finding gate...")
    hub.light.on(Color.ORANGE)
    t0 = sw.time()
    while True:
        t_ms = sw.time()
        dt = t_ms - t_prev
        if dt < 1:
            dt = 1
        t_prev = t_ms
        tick(dt, t_ms)
        if t_ms - last_hk >= 10:
            hk(t_ms)
        if lap_done:
            break
        if t_ms - t0 > 60000:      # 60秒で見つからなければ中断→再開
            print("gate not found - check sensor placement/polarity")
            hub.light.on(Color.RED)
            wait(5000)
            t0 = sw.time()
    print("gate found at s=%d" % gate_s)


def run_laps():
    """無限周回: 1周目=エクスプローラ、2周目以降=学習レーシング"""
    global in_lap, lap_no, lap_start_t, t_prev, gate_s, slow_mode, lap_valid
    global lap_done
    lap_no = 0
    while True:
        lap_no += 1
        in_lap = True
        reset_lap_state()
        lap_start_t = sw.time()
        hub.light.on(Color.GREEN)
        print("=== LAP %d start ===" % lap_no)
        while not lap_done:
            t_ms = sw.time()
            dt = t_ms - t_prev
            if dt < 1:
                dt = 1
            t_prev = t_ms
            tick(dt, t_ms)
            if t_ms - last_hk >= 10:
                hk(t_ms)
            # ゲート読み飛ばし: 予測位置を過ぎたら低速探索モードへ (G4)
            if in_lap and track_len > 0 and not slow_mode and s_cm > track_len + GATE_MISS_CM:
                slow_mode = True
                lap_valid = False
                print("gate missed - slow search mode")
            # 最終手段: 1周分以上過ぎても検出不能なら強制終了
            if in_lap and slow_mode and s_cm > track_len * 2 + 900:
                gate_s = track_len
                lap_done = True
                print("emergency lap end (gate never found)")
        # ゲート通過時刻の補間 (G5)
        t_lap = sw.time() - lap_start_t - gate_f * gate_dt // 100
        finalize_lap(t_lap)


def main():
    print("=== SPIKE LINE RACER ===")
    mem_info()
    hub.system.set_stop_button((Button.CENTER, Button.LEFT, Button.RIGHT))
    hub.display.off()
    if not check_devices():
        print("!! wiring error detected - fix and restart")
        hub.light.on(Color.RED)
        wait(5000)
    bench()
    calib_stationary()
    auto_sign_test()
    wait_for_start()
    find_gate()
    run_laps()


# ハブでは __main__ として実行される。PC テスト (pybricks モック) からの
# import 時は本体を実行しない (__name__ ガード)
if __name__ == "__main__":
    main()
