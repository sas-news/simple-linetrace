# ============================================================================
# LINE RACER SLIM  "main.py"  v2.0  (Pybricks 用)
# ----------------------------------------------------------------------------
# 目的: SPIKE Prime でランダム曲線の黒ライン(1本の無限周回)を安定追従し続け、
#       ラップタイムを計測・ベスト記録する。設計は「走りながら最適化」をやめ、
#       実測キャリブレーションに基づく堅牢なシンプル制御を優先する。
#
# v2.0 の変更点 (v1.x からの刷新):
#   - 2本線ゲート検出と複雑な学習(ILC/vmax/セクションゲイン/探索/ロールバック)を
#     廃止。代わりに「走路上1点の横断マーカー線」をラップ境界に使う単一マーカー方式。
#   - 符号は起動時自動テストを省略し、実測値に固定 (CMD_FLIP=-1, SIGN_GYRO_CUR=-1)。
#   - 反射光キャリブレーションは実測値に固定 (L:w96/b13, R:w95/b12)。
#   - センサー実効更新レート ~19Hz を前提とし、微分(KD)はブームD項正帰還回避のため
#     0 固定 (de_f ローパスは実装済み)。
#
# ハードウェア (実測確定):
#   Port A: 左モーター   Port E: 右モーター
#   Port B: 左カラーセンサー Port F: 右カラーセンサー (ラインを挟む差動配置)
#
# ラップ境界: コース上の1箇所に、進行方向に垂直な「マーカー線」を引く。
#   両センサが同時に暗くなる(交差)を検出してラップ境界とする。
#   主ラインはセンサで挟んで追従するため同時暗にならない → 誤検出を排除。
#
# 使い方:
#   Pybricks Code で書き込んで実行。中央ボタンでスタート(10秒で自動)。
#   停止: 中央+左+右 同時押し。 左: ダンプ切替。 右: 統計表示。
# ============================================================================

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

# ---------------- 機体 / 配線 (実測確定) ----------------
PORT_LM = Port.A
PORT_RM = Port.E
PORT_LS = Port.B
PORT_RS = Port.F

# ホイール円周・トレッド (実測 2026-08):
#   外周 27.6cm, トレッド 16cm (20ポッチ)。
#   変換定数 WMUL/SMUL は円周27.6cmから導出: WMUL=36000/27.6, SMUL=27.6*10000/360
WHEEL_C_CM = 27.6
TRACK_B_CM = 16.0       # 左右駆動輪 接地中心間 トレッド幅 [cm]
WMUL = 1304             # deg/s = cm/s*WMUL/100 (円周27.6cm基準)
SMUL = 767              # cm = deg*SMUL/10000 (円周27.6cm基準)
VERSION = "2.0"

# ---------------- 符号 (実測・整合セットで確定) ----------------
# ☑ CMD_FLIP を +1 にした場合は GYRO_SIGN も必ず合わせて反転させること。
#   モーター方向マッピングが反転するため、ジャイロ減衰の符号も連動して変わる。
CMD_FLIP = +1           # 前進(実測)。後退する場合のみ -1
SIGN_GYRO_CUR = -1      # ジャイロ符号 (spin test 確定。Z が旋回軸)
# ステアリング極性。物理導出(配線: 左S=B・左M=A)で +1 が正。
# ライン左=左Sが黒=nL低 → e>0 → st>0=左旋回=追従。整合。
SIGN_ST = +1
# ジャイロ減衰方向 (GYRO=0のため現在は不使用。復帰時に CMD_FLIP と連動させる)
GYRO_SIGN = +1

# ---------------- 反射光キャリブレーション (実測) ----------------
calL_w, calL_b = 96, 13
calR_w, calR_b = 95, 12
wz_bias = 0

# ---------------- 制御 ----------------
# 実測振り返り: KP=40/GYRO=1000 は「過減衰」—ジャイロ減衰がラインへの修正旋回まで
# 打ち消し、弱くて離脱。ジャイロは1000Hzで読む高周波ノイズをGain増幅してプルプル。
# 対策: KPを回復 + GYROは控えめ + ジャイロ/誤差をローパス(ノイズだけ除去)。
KP = 35                 # 比例: cm/s per 1000e (カーブを曲がり切る強さ。15→35)
KD = 0                  # 微分: ブームD項正帰還発振回避のため 0 固定
# ジャイロ減衰(減衰の主役)。KPを上げてもプルプルしないよう減衰を強化。
# ウォッシュアウト(wz_d = wz_f - wz_base)でバイアスに騙されず、速い回転だけ減衰に。
GYRO = 650
ST_MAX = 150            # ステアリングクランプ cm/s
A_ACC = 220             # 加速リミット cm/s^2
A_BRK = 350             # 減速リミット cm/s^2

# ------------- 速度 -------------
V_FIND = 18             # マーカー探索速度 cm/s
V_EXPLORE = 30          # 1周目(距離測定)速度 cm/s
# 巡航速度: 実測最高速77cm/sよりは低めに。19Hzセンサーとブーム輸送遅延に対し、
# 速度を下げると「検出→ホイール到達」の遅れ距離が減り、突然のラインロストを抑える。
V_CRUISE = 35
V_MIN = 12              # 最低速度 cm/s
TURN_FULL = 45000       # |ジャイロ| がこの値で V_MIN まで減速 (~45deg/s)
HARD_MAX = 250          # 安全リミット cm/s

# ------------- マーカー検出 -------------
MARK_MIN = 0.5          # マーカー線の最小幅 cm
MARK_MAX = 10           # マーカー線の最大幅 cm
MARK_REARM = 40         # 検出後、再アームまでの距離 cm
MARK_WIN = 60           # track_len 確定後、境界を受け付ける位置窓 +/-cm
MARK_MISS = 300         # この距離過ぎて未検出なら緊急ラップ確定
MIN_LAP_CM = 40         # これ未満の距離ラップは「スピンによる仮マーカー」とみなし却下

# ------------- 学習FF (ILC, ジャイロ非依存) -------------
# 1周目の追従誤差からビン毎に必要なステアリングFFを学習し、2周目以降 s に応じて
# 事前にステアリングを再現(曲率FF)。バイアス・センサー極性非依存で曲線を事前対応。
# FFは v=100cm/s 基準に正規化して保持し、実行時 st += ff[s]*v//100 で適用。
BIN_CM = 2              # 弧長ビン幅 cm
N_BINS = 1500           # ビン数 (3000cm 分)
FF_MAX = 40             # FFクランプ cm/s @ v=100
ILC_ALPHA = 0           # 学習率 ×1000。※0=FF無効(ベース隔離用に仮設定。確認後に20へ戻す)
ILC_MIN_CNT = 2         # ビンの学習に必要なサンプル数下限
ILC_TRACK_OK = 250      # 前ラップの rms_e がこの値以下なら FF を「適用」許可
                        # (1周目がフラフラだとゴミを学習→暴れるため。追従が安定して
                        #  初めてコースを事前に再現する)

# ------------- レーシング (片輪最高速 + 学習速度計画) -------------
# 学習マップ(ff_armed)が確立したラップのみ有効。外輪(fast)を V_plan まで引き上げ、
# 内輪は 2|st| だけ落として旋回。直線で最高速、カーブは学習した速度計画と
# ルックアヘッドで事前減速。未アーム時は従来の V_CRUISE 制御のまま(何周でも安定)。
RACE_MODE = False        # レーシング有効フラグ。※False=ベース隔離用(純比例+ジャイロ)。確認後にTrueへ
V_RACE_MAX = 60          # レーシング最高速 cm/s (モーター最高速77の約78%。確認後に70へ)
A_LAT_MAX = 200          # 横加速度限界 cm/s^2 (速度計画の曲率制限)
V_MARK = 30              # マーカー前後の低速速度 cm/s (高速時のマーカー見逃し防止)
LA_BINS = 15             # 速度計画のルックアヘッド ビン数 (~30cm)

# ------------- バッテリー補償 -------------
BATT_NOM = 7600
COMP_MAX = 1300

# ------------- ログ/表示 -------------
LOG_N = 8192            # ラップログ数 (25ms→204秒分)
LOG_MS = 25
DISPLAY_MS = 200
NEW_BEST_HOLD_MS = 2500
AUTO_DUMP_BEST = True

# ============================================================================
# 固定テーブル
# ============================================================================
log_s = bytearray(LOG_N * 2)
log_e = bytearray(LOG_N)
log_v = bytearray(LOG_N)
log_w = bytearray(LOG_N * 2)

# 学習FFテーブル (ILC: ビン毎に速度正規化ステアリングFFを保持)
# ff: i16 [cm/s @ v=100], err_sum: i16 誤差累積, vsum_bin: i16 速度累積, err_cnt: u8
ff = bytearray(N_BINS * 2)
err_sum = bytearray(N_BINS * 2)
vsum_bin = bytearray(N_BINS * 2)
err_cnt = bytearray(N_BINS)
# レーシング: 速度計画 vmax (u8) + 実行ステアリング積算 st_sum (i16) + 平滑用スクラッチ
vmax = bytearray(N_BINS)
st_sum = bytearray(N_BINS * 2)
vmax_scr = bytearray(N_BINS)

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


def u16at(ba, i):
    j = i * 2
    return ba[j] | (ba[j + 1] << 8)


def i16at(ba, i):
    """i16 読み出し (2の補数, ビン配列用)。"""
    j = i * 2
    v = ba[j] | (ba[j + 1] << 8)
    if v >= 32768:
        v -= 65536
    return v


def i16set(ba, i, v):
    """i16 書き込み (クランプ付き)。"""
    if v > 32767:
        v = 32767
    elif v < -32768:
        v = -32768
    j = i * 2
    ba[j] = v & 255
    ba[j + 1] = (v >> 8) & 255


def ba_clear(ba, n):
    for i in range(n):
        ba[i] = 0


def ba_fill(ba, v, n):
    for i in range(n):
        ba[i] = v


def race_wheels(v_plan, st, comp, v_min):
    """レーシング則: 外輪(方向に応じた片側)を v_fast まで, 内輪を v_fast-2|st| にする。
    st>0=左旋回 → 右輪が外輪。内輪下限 = -v_min。戻り: (vL, vR) cm/s(comp前)"""
    ve_fast = v_plan * comp // 1000
    sa = st
    if sa < 0:
        sa = -sa
    vs = ve_fast - 2 * sa
    if vs < -v_min:
        vs = -v_min
    if st >= 0:
        return vs, ve_fast
    return ve_fast, vs


def vmax_from_stn(stn, a_lat_max, v_race_max):
    """速度計画のビン毎上限: 実行ステアリング stn(=|st|*100/ve) → κ1000=125*stn から導出。
    v_curv=√(a_lat/κ), v_kin(内輪≧0制約)=481250/κ の小さい方, V_RACE_MAX上限"""
    kk = 125 * stn
    if kk > 12500:
        kk = 12500
    if kk < 1:
        kk = 1
    vc = isqrt(a_lat_max * 100000 // kk)
    vk = 481250 // kk
    vm = vc if vc < vk else vk
    if vm > v_race_max:
        vm = v_race_max
    return vm


# vmax の初期化 (0 のままだとルックアヘッドの min が V_MIN に張り付く)
ba_fill(vmax, V_RACE_MAX, N_BINS)


# ============================================================================
# 状態
# ============================================================================
hub = PrimeHub()
mL = Motor(PORT_LM, positive_direction=Direction.COUNTERCLOCKWISE)
mR = Motor(PORT_RM, positive_direction=Direction.CLOCKWISE)
csL = ColorSensor(PORT_LS)
csR = ColorSensor(PORT_RS)
sw = StopWatch()

e_prev = 0
e_fil = 0
wz_f = 0
wz_base = 0
de_f = 0
v_cmd = 0
s_cm = 0
s_base_deg = 0
vbat = BATT_NOM
comp = 1000

# ラップ状態
in_lap = False
lap_no = 0
lap_done = False
lap_valid = True
track_len = 0
mark_s = 0
mark_f = 0          # マーカー通過時刻の補間係数 x100
mark_dt = 0
lap_start_t = 0
best_time = 0
best_lap = 0
n_min_prev = 1000
ilc_bin = 0              # tick内で使うビン番号 (FF適用/蓄積)
ff_weight = 0            # ILC更新後の平滑用 (ff_learnで使う)
ff_armed = False         # 追従が安定した(低rms_e)ラップ後のみ FF適用許可

# マーカー状態
mk_armed = True
mk_state = 0
mk_s0 = 0
mk_last = -1000

# ループ/ログ
t_prev = 0
last_hk = 0
last_log = -1000
last_rate = -1000
last_disp = -1000
last_batt = -1000
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
disp_hold = 0
new_best_flag = False

# ============================================================================
# マーカー検出 (単一横断線: 白→暗→白)
# ============================================================================

def marker_tick(s, both_dark):
    """s: 弧長 cm, both_dark: 両センサ同時暗。通過位置(cm)を返す。"""
    global mk_armed, mk_state, mk_s0, mk_last
    if not mk_armed:
        if s - mk_last > MARK_REARM:
            mk_armed = True
        else:
            return -1
    hit = -1
    if mk_state == 0:
        if both_dark:
            mk_s0 = s
            mk_state = 1
    elif mk_state == 1:
        if not both_dark:
            w = s - mk_s0
            if MARK_MIN <= w <= MARK_MAX:
                mk_state = 0
                mk_armed = False
                mk_last = s
                hit = s
            else:
                mk_state = 0
    return hit

# ============================================================================
# 制御ティック
# ============================================================================

@native
def tick(dt_ms, t_ms):
    global e_prev, e_fil, wz_f, wz_base, de_f, v_cmd, s_cm, s_base_deg, last_log
    global log_idx, log_full, tick_cnt, mark_f, mark_dt, n_min_prev, lap_done
    global mark_s, ilc_bin, ff_armed

    rl = csL.reflection()
    rr = csR.reflection()

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

    # 誤差: e<0 = ラインが右寄り → 右に曲がる (SIGN_ST で反転)
    e = (nR - nL) * 1000 // (nL + nR + 1)
    if SIGN_ST < 0:
        e = -e
    # 19Hzセンサーの跳び・バウンドを抑える1次ローパス (制御用のみ。ログは生 e)
    e_fil = (e_fil * 2 + e) // 3

    wz = int(hub.imu.angular_velocity(Axis.Z) * 1000) - wz_bias
    if SIGN_GYRO_CUR < 0:
        wz = -wz
    # ジャイロ: ローパス + ウォッシュアウト(バイアス/ゆっくり変動を除去)
    wz_f = (wz_f * 3 + wz) // 4
    wz_base = (wz_base * 31 + wz_f) // 32
    wz_d = wz_f - wz_base

    # オドメトリ
    aL = mL.angle() * CMD_FLIP
    aR = mR.angle() * CMD_FLIP
    s_cm = ((aL + aR) // 2 - s_base_deg) * SMUL // 10000

    # マーカー(両センサ同時暗)判定
    both_dark = (nL < 350) and (nR < 350)

    # 微分 (ローパス)
    de = (e - e_prev) * 1000 // max(dt_ms, 1)
    de_f = (de_f * 5 + de) // 6
    e_prev = e

    # ステアリング: 平滑化誤差 + ジャイロ減衰(ウォッシュアウト)
    #                + 学習FF (2周目以降, s に応じて事前ステアリング再現)
    b = s_cm // BIN_CM
    if b >= N_BINS:
        b = N_BINS - 1
    if b < 0:
        b = 0
    ilc_bin = b
    st = (KP * e_fil) // 1000
    if KD:
        st += (KD * de_f) // 1000
    st += (GYRO_SIGN * GYRO * wz_d) // 1000000
    if in_lap and lap_no >= 2 and track_len > 0 and ff_armed:
        # 速度正規化FF (v=100基準で保持 → 現在速度でスケール)
        st += i16at(ff, b) * v_cmd // 100
    if st > ST_MAX:
        st = ST_MAX
    elif st < -ST_MAX:
        st = -ST_MAX

    # レーシングは2周目以降常に有効 (vmaxはラップ1の実走能力から安全に決まり、誤差減速
    # とマーカー低速ゾーンが安全網。FFステアリングは別途 ff_armed で良学習時のみ適用)
    racing = in_lap and lap_no >= 2 and track_len > 0 and RACE_MODE
    if racing:
        # 速度計画: ルックアヘッド(vmax の前方最小) + 誤差による減速 + 加減速ランプ
        racnb = track_len // BIN_CM + 2
        if racnb > N_BINS:
            racnb = N_BINS
        vp = V_RACE_MAX
        for j in range(LA_BINS):
            bj = b + j
            if bj >= racnb:
                bj = racnb - 1
            vc = vmax[bj]
            if vc < vp:
                vp = vc
        ae = e_fil
        if ae < 0:
            ae = -ae
        if ae > 600:
            ae = 600
        # 誤差減速を穏やかに (//600→//800): 多少揺れても(追従できている限り)速度を保てる
        slow = (vp - V_MIN) * ae // 800
        vt = vp - slow
        if vt < V_MIN:
            vt = V_MIN
        acc = A_ACC if vt >= v_cmd else A_BRK
        dv = acc * dt_ms // 1000
        if vt > v_cmd + dv:
            v_cmd += dv
        elif vt < v_cmd - dv:
            v_cmd -= dv
        # レーシング差分: 外輪を v_cmd (fast), 内輪を 2*|st| 落とす
        vL, vR = race_wheels(v_cmd, st, comp, V_MIN)
    else:
        # 従来控制 (FF アーム前 / 探索中): 中央 ve ± st
        if not in_lap:
            vt = V_FIND
        else:
            vt = V_CRUISE
            if lap_no == 1:
                if vt > V_EXPLORE:
                    vt = V_EXPLORE
        ae = e_fil
        if ae < 0:
            ae = -ae
        if ae > 600:
            ae = 600
        slow = (V_CRUISE - V_MIN) * ae // 600
        if vt - slow < V_MIN:
            vt = V_MIN
        else:
            vt -= slow
        if vt > V_CRUISE:
            vt = V_CRUISE
        if vt < V_MIN:
            vt = V_MIN
        acc = A_ACC if vt >= v_cmd else A_BRK
        dv = acc * dt_ms // 1000
        if vt > v_cmd + dv:
            v_cmd += dv
        elif vt < v_cmd - dv:
            v_cmd -= dv
        ve = v_cmd * comp // 1000
        vL = ve - st
        vR = ve + st

    # ステアリング積算 (速度計画の学習用: 実行ステアリングを実測速度正規化)
    # ※内外輪のうち速い方が ve、遅い方が ve-2|st| なので実測中心速度≈v_cmd-|st|
    if in_lap and lap_valid:
        sa = st
        if sa < 0:
            sa = -sa
        ve_act = v_cmd - sa
        if ve_act < 10:
            ve_act = 10
        stn = sa * 100 // ve_act
        x = i16at(st_sum, ilc_bin) + stn
        if x > 60000:
            x = 60000
        i16set(st_sum, ilc_bin, x)

    if vL > HARD_MAX:
        vL = HARD_MAX
    elif vL < -HARD_MAX:
        vL = -HARD_MAX
    if vR > HARD_MAX:
        vR = HARD_MAX
    elif vR < -HARD_MAX:
        vR = -HARD_MAX
    mL.run(vL * WMUL // 100 * CMD_FLIP)
    mR.run(vR * WMUL // 100 * CMD_FLIP)

    # マーカー検出 (ラップ境界)。find_marker(in_lap=False) でも検出する
    if not lap_done:
        run = True
        if lap_no > 1 and track_len > 0 and abs(s_cm - track_len) > MARK_WIN:
            run = False
        if run:
            g = marker_tick(s_cm, both_dark)
            if g > 0:
                if lap_no <= 1 or (track_len > 0 and abs(g - track_len) <= MARK_WIN):
                    mark_s = g
                    lap_done = True
                    n_min = nL if nL < nR else nR
                    den = n_min - n_min_prev
                    if den > 0:
                        f = (350 - n_min_prev) * 100 // den
                    elif den < 0:
                        f = 100
                    else:
                        f = 100
                    if f < 0:
                        f = 0
                    elif f > 100:
                        f = 100
                    mark_f = f
                    mark_dt = dt_ms

    # オドメトリによる確実なラップ境界 (マーカー見逃し・ゆらゆら中でも1周ごとに閉じる)
    # マーカーboth_darkが頼れない瞬間があっても、s_cm が track_len を跨げば閉じる。
    if in_lap and not lap_done and lap_no >= 2 and track_len > 0 and s_cm >= track_len:
        mark_s = track_len
        mark_f = 100
        mark_dt = dt_ms
        lap_done = True

    n_min = nL if nL < nR else nR
    n_min_prev = n_min

    # 学習FF蓄積 (ビン毎に誤差/速度を平均するため累積。ラップ後に ILC更新)
    if in_lap and lap_valid:
        x = i16at(err_sum, ilc_bin) + e_fil
        i16set(err_sum, ilc_bin, x)
        vx = i16at(vsum_bin, ilc_bin) + v_cmd
        if vx > 60000:
            vx = 60000
        i16set(vsum_bin, ilc_bin, vx)
        c = err_cnt[ilc_bin]
        if c < 250:
            err_cnt[ilc_bin] = c + 1

    # ラップログ
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

# ============================================================================
# ハウスキーピング
# ============================================================================

def hk(t_ms):
    global last_batt, comp, vbat, last_disp, last_rate, rate_hz, tick_cnt
    global dump_enable, dump_idx, dump_end, prev_left, prev_right

    if t_ms - last_batt >= 100:
        last_batt = t_ms
        vbat = hub.battery.voltage()
        c = BATT_NOM * 1000 // max(vbat, 6000)
        if c > COMP_MAX:
            c = COMP_MAX
        elif c < 1000:
            c = 1000
        comp = c

    if t_ms - last_rate >= 1000:
        last_rate = t_ms
        rate_hz = tick_cnt
        tick_cnt = 0

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

    if t_ms - last_disp >= DISPLAY_MS:
        last_disp = t_ms
        if t_ms < disp_hold:
            hub.display.number(best_time // 100)
        elif in_lap:
            n = (t_ms - lap_start_t) // 100
            if n > 9999:
                n = 9999
            hub.display.number(n)
        else:
            hub.display.off()

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
    print("stats: kp=%d kd=%d v_cruise=%d track=%d best=%d lap=%d rate=%dHz comp=%d vbat=%d" %
          (KP, KD, V_CRUISE, track_len, best_time, lap_no, rate_hz, comp, vbat))

# ============================================================================
# 起動時
# ============================================================================

def check_devices():
    print("-- device check --")
    ok = True
    for name, fn in (("LeftMotor", mL.angle), ("RightMotor", mR.angle),
                     ("LeftSensor", csL.reflection), ("RightSensor", csR.reflection)):
        try:
            fn()
            print("%s: OK" % name)
        except Exception:
            print("%s: NOT CONNECTED!" % name)
            ok = False
    return ok

def calib_stationary(ms=1200):
    global wz_bias
    wzb = 0
    n = 0
    t0 = sw.time()
    while sw.time() - t0 < ms:
        wzb += int(hub.imu.angular_velocity(Axis.Z) * 1000)
        n += 1
    wz_bias = wzb // max(n, 1)
    # 静止時のバイアスが5deg/s超は測定不良(ロボットが動いた/振動)。上限クランプ
    if wz_bias > 5000:
        wz_bias = 5000
    elif wz_bias < -5000:
        wz_bias = -5000
    print("calib: bias=%d n=%d" % (wz_bias, n))

# ============================================================================
# ラップ統計
# ============================================================================

def lap_stats():
    n = log_idx
    if n < 2:
        return 0, 0, 0, 0
    rms = 0
    mx = 0
    vsum = 0
    max_s = 0
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
        vsum += log_v[i]
    rms_e = isqrt(rms // n) * 1000 // 127
    mx = mx * 1000 // 127
    return rms_e, mx, vsum // n, max_s

# ============================================================================
# ラップ終了
# ============================================================================

@native
def finalize_lap(t_lap):
    global track_len, best_time, best_lap, disp_hold, new_best_flag
    global dump_idx, dump_end, ff_armed

    rms_e, mx_e, v_avg, dist = lap_stats()

    if not lap_valid or dist < MIN_LAP_CM:
        # スピンによる仮マーカー連発など、距離が極端に短いラップは記録/学習を却下
        print("LAP %d skipped (invalid or too short dist=%d)" % (lap_no, dist))
        return

    if track_len == 0:
        track_len = mark_s
        print("track len (lap1) = %d cm" % track_len)

    # 追従が安定したラップのみ FF適用を許可 (ゴミ学習による暴れを防ぐ)
    ff_armed = ff_armed or (rms_e <= ILC_TRACK_OK)

    # ---- 学習FF更新 (ILC: ff += e_avg * alpha / (10 * v_bin), v=100基準) ----
    # 1周目の追従誤差から曲線の必要なステアリングを学習し、2周目以降に事前適用する。
    ilc_rms = 0
    nb = track_len // BIN_CM + 2
    if nb > N_BINS:
        nb = N_BINS

    # ---- 速度計画 vmax 更新 (レーシング用。毎ラップ。ILCクリア前に実行) ----
    # vmax は実行ステアリングから導出 → ラップ1がフラフラでも「その地点の実力」に
    # 応じた安全な速度になり、ラップが良くなるほど高くなる。
    for i in range(nb):
        c = err_cnt[i]
        if c >= ILC_MIN_CNT:
            stn = i16at(st_sum, i) // c
            vmax[i] = vmax_from_stn(stn, A_LAT_MAX, V_RACE_MAX)
        else:
            vmax[i] = V_RACE_MAX
    # マーカー前後を低速化 (高速時にマーカー見逃し→ラップ境界破損するのを防ぐ)
    m0 = (track_len - 10) // BIN_CM
    if m0 < 0:
        m0 = 0
    m1 = (track_len + 10) // BIN_CM + 1
    if m1 > nb:
        m1 = nb
    for i in range(m0, m1):
        if V_MARK < vmax[i]:
            vmax[i] = V_MARK
    # 周辺ビンの min で平滑化 (単発ノイズによる急減速を防ぐ)
    for i in range(nb):
        m = vmax[i]
        j0 = i - 3
        if j0 < 0:
            j0 = 0
        j1 = i + 4
        if j1 > nb:
            j1 = nb
        for j in range(j0, j1):
            if vmax[j] < m:
                m = vmax[j]
        vmax_scr[i] = m
    for i in range(nb):
        vmax[i] = vmax_scr[i]

    # ---- 学習FF更新 (ILC: ff += e_avg * alpha / (10 * v_bin), v=100基準) ----
    # 1周目の追従誤差から曲線の必要なステアリングを学習し、2周目以降に事前適用する。
    ilc_rms = 0
    for i in range(nb):
        c = err_cnt[i]
        if c >= ILC_MIN_CNT:
            e_avg = i16at(err_sum, i) // c
            vb = i16at(vsum_bin, i) // c
            if vb < 10:
                vb = 10
            f = i16at(ff, i)
            f += e_avg * ILC_ALPHA // (10 * vb)
            if f > FF_MAX:
                f = FF_MAX
            elif f < -FF_MAX:
                f = -FF_MAX
            i16set(ff, i, f)
            ilc_rms += f * f
            err_cnt[i] = 0
            i16set(err_sum, i, 0)
            i16set(vsum_bin, i, 0)
    ilc_rms = isqrt(ilc_rms // max(N_BINS, 1))

    new_best_flag = False
    if best_time == 0 or t_lap < best_time:
        best_time = t_lap
        best_lap = lap_no
        new_best_flag = True
        disp_hold = sw.time() + NEW_BEST_HOLD_MS
        hub.light.on(Color.GREEN)
        hub.speaker.beep(1200, 120)
        hub.speaker.beep(1600, 120)
        print("NEW BEST: lap %d t=%dms" % (lap_no, t_lap))
    else:
        hub.light.on(Color.GREEN)

    print("LAP %d done: t=%dms best=%dms dist=%dcm rms_e=%d max_e=%d vavg=%d rate=%dHz track=%d" %
          (lap_no, t_lap, best_time, dist, rms_e, mx_e, v_avg, rate_hz, track_len))

    if (dump_enable or (new_best_flag and AUTO_DUMP_BEST)) and log_idx > 0:
        dump_idx = 0
        dump_end = log_idx
        print("#LAP lap_no %d time_ms %d best_ms %d dist_cm %d rms_e %d max_e %d vavg %d rate_hz %d" %
              (lap_no, t_lap, best_time, dist, rms_e, mx_e, v_avg, rate_hz))
        print("#CFG wheel %d track_b %d cruise %d kp %d kd %d gyrosign %d flipsign %d" %
              (int(WHEEL_C_CM * 10), int(TRACK_B_CM * 10), V_CRUISE, KP, KD,
               SIGN_GYRO_CUR, CMD_FLIP))
        print("#DATA s_cm,e,v_cmps,w_mdeg_s")

def reset_lap_state():
    global log_idx, log_full, last_log, mk_state, mk_armed, mk_last
    global e_prev, e_fil, wz_f, wz_base, de_f, v_cmd, lap_done, lap_valid, s_base_deg, t_prev
    global mark_f, mark_dt, n_min_prev
    log_idx = 0
    log_full = False
    last_log = -1000
    mk_state = 0
    mk_armed = True
    mk_last = -1000
    e_prev = 0
    e_fil = 0
    wz_f = 0
    # wz_base (ジャイロ・バイアス基準) はリセットしない: 0にすると旋回中のラップ境界で
    # wz_d が一時巨大になり、あらぬ方向へ走る。ラップ間で継続させる。
    de_f = 0
    v_cmd = V_MIN
    lap_done = False
    lap_valid = True
    mark_f = 0
    mark_dt = 0
    n_min_prev = 1000
    t_prev = sw.time()
    s_base_deg = (mL.angle() + mR.angle()) // 2 * CMD_FLIP
    # 学習蓄積は毎ラップ最後にクリア (ff / vmax テーブル自体はラップ間で保持する)
    ba_clear(err_sum, N_BINS * 2)
    ba_clear(vsum_bin, N_BINS * 2)
    ba_clear(err_cnt, N_BINS)
    ba_clear(st_sum, N_BINS * 2)

# ============================================================================
# メイン状態機械
# ============================================================================

def wait_for_start():
    print("Press CENTER to start (auto in 10s)")
    t0 = sw.time()
    while sw.time() - t0 < 10000:
        if Button.CENTER in hub.buttons.pressed():
            break
        wait(50)

def find_marker():
    """最初のマーカーを探して通過し、弧長原点を確定する"""
    global in_lap, t_prev, s_base_deg, lap_done, mk_armed, mk_state
    in_lap = False
    lap_done = False
    t_prev = sw.time()
    s_base_deg = (mL.angle() + mR.angle()) // 2 * CMD_FLIP
    mk_armed = True
    mk_state = 0
    hub.light.on(Color.ORANGE)
    print("finding first marker...")
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
        if t_ms - t0 > 30000:
            print("marker not found - check robot on line & marker placement")
            hub.light.on(Color.RED)
            wait(5000)
            t0 = sw.time()
    print("first marker passed - s=0")

def run_laps():
    global in_lap, lap_no, lap_start_t, t_prev, mark_s, lap_done
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
            if in_lap and not lap_done and track_len > 0 and s_cm > track_len + MARK_MISS:
                print("marker missed - forcing lap end")
                mark_s = track_len
                lap_done = True
        # ラップ終了処理(finalize)中にモーターが無制御のまま走り去らないよう停止
        mL.run(0)
        mR.run(0)
        t_lap = sw.time() - lap_start_t - mark_f * mark_dt // 100
        finalize_lap(t_lap)
        wait(100)   # finalize対象の残留入力を吸収してから次ラップ開始

def main():
    print("=== LINE RACER SLIM v%s ===" % VERSION)
    mem_info()
    hub.system.set_stop_button((Button.CENTER, Button.LEFT, Button.RIGHT))
    hub.display.off()
    if not check_devices():
        print("!! wiring error detected - fix and restart")
        hub.light.on(Color.RED)
        wait(5000)
    calib_stationary()
    wait_for_start()
    find_marker()
    run_laps()

if __name__ == "__main__":
    main()
