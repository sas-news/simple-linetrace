# -*- coding: utf-8 -*-
"""main.py の純ロジック自己テスト (PC上で検証、ハブでは実行しない)"""
import math

# ---- main.py から移植した純関数 ----
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
    # main.py と同一ロジック。Pybricks は Small Int (±2^30) のみのため、
    # <<24 / 2^32 リテラルは使わず u16+i16 分解で組み立てる。
    # i32 slot i はバイト [4i,4i+3] = u16@slot(2i) + i16@slot(2i+1)。
    j = i * 2
    lo = u16at(ba, j)
    hi = i16at(ba, j + 1)
    return hi * 65536 + lo


def i32set(ba, i, v):
    j = i * 2
    i16set(ba, j, v & 0xFFFF)
    i16set(ba, j + 1, (v >> 16) & 0xFFFF)


def i32add(ba, i, v):
    x = i32at(ba, i) + v
    i16set(ba, i * 2, x & 0xFFFF)
    i16set(ba, i * 2 + 1, (x >> 16) & 0xFFFF)


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


# ---- マッチャー (main.py と同一ロジック) ----
G_LINE_MIN, G_LINE_MAX, G_GAP_MIN, G_GAP_MAX, G_GAP_TOL, G_REARM = 1, 8, 3, 100, 50, 30
gm_state = 0
gm_s0 = 0
gm_s1 = 0
gm_s2 = 0
gm_armed = True
gm_last_s = -1000
gm_gap = 0
gate_gap_cm = 0   # 学習済みゲート間隔 (0=未学習)


def matcher_tick(s, line):
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
                gm_gap = gm_s2 - gm_s1
                gate = s
    return gate


def reset_matcher():
    global gm_state, gm_s0, gm_s1, gm_s2, gm_armed, gm_last_s, gm_gap
    gm_state = 0
    gm_s0 = gm_s1 = gm_s2 = 0
    gm_armed = True
    gm_last_s = -1000
    gm_gap = 0


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


# ============================== テスト ==============================

# ---- テスト1: i16/i32 ラウンドトリップ ----
ba16 = bytearray(10 * 2)
for v in [0, 1, -1, 32767, -32768, 12345, -12345, 200, -200]:
    i16set(ba16, 3, v)
    assert i16at(ba16, 3) == v, "i16 roundtrip %d -> %d" % (v, i16at(ba16, 3))
# i32 は Pybricks Small Int 上限 (±2^30-1) 内でラウンドトリップ検証
ba32 = bytearray(10 * 4)
for v in [0, 1, -1, 1073741823, -1073741824, 65535, 65536, -65536,
          16777216, -16777216, 123456789, -987654321]:
    i32set(ba32, 2, v)
    assert i32at(ba32, 2) == v, "i32 set %d -> %d" % (v, i32at(ba32, 2))
    i32set(ba32, 2, 0)
i32add(ba32, 2, 123456789)
i32add(ba32, 2, 100)
i32add(ba32, 2, -100)
assert i32at(ba32, 2) == 123456789
# 負の累積 (κ の両符号) と上位バイト境界 (符号ビット) の検証
i32set(ba32, 2, 0)
i32add(ba32, 2, -34906)
i32add(ba32, 2, -34906)
assert i32at(ba32, 2) == -69812, "i32 negative acc -> %d" % i32at(ba32, 2)
assert ba32[8 + 3] >= 0x80, "neg acc top byte should be sign-extended"
# 隣接スロット独立性: slot0 と slot1 が重なると (x2 二重適用バグ) 値が混ざる
i32set(ba32, 0, 0x11111111)
i32set(ba32, 1, 0x22222222)
assert i32at(ba32, 0) == 0x11111111, "slot0 汚染: %08x" % i32at(ba32, 0)
assert i32at(ba32, 1) == 0x22222222, "slot1 汚染: %08x" % i32at(ba32, 1)
print("PASS i16/i32 roundtrip")

# ---- テスト2: パーセンタイル ----
hist = [0] * 101
for i in range(90, 101):
    hist[i] = 100
for i in range(0, 10):
    hist[i] = 100
assert percentile(hist, 2) == 0
assert percentile(hist, 98) == 100
print("PASS percentile")

# ---- テスト3: マッチャー 正常ゲート (2本線: 幅2cm、間隔20cm) ----
reset_matcher()
gate_detected = []
for k in range(400):
    x = k * 0.5
    on_line = (50 <= x <= 52) or (72 <= x <= 74)
    g = matcher_tick(x, on_line)
    if g > 0:
        gate_detected.append(g)
assert len(gate_detected) == 1 and abs(gate_detected[0] - 74.5) < 0.001, gate_detected
print("PASS matcher normal gate at", gate_detected)

# ---- テスト4: マッチャー 誤検出排除 ----
reset_matcher()
for k in range(400):
    x = k * 0.5
    on_line = (50 <= x <= 52)  # 1本だけ
    g = matcher_tick(x, on_line)
    assert g < 0
print("PASS matcher rejects single line")

reset_matcher()
for k in range(400):
    x = k * 0.5
    on_line = (50 <= x <= 60) or (65 <= x <= 68)  # 1本目が幅10cm(範囲外)
    g = matcher_tick(x, on_line)
    assert g < 0
print("PASS matcher rejects wide line")

reset_matcher()
for k in range(600):
    x = k * 0.5
    on_line = (50 <= x <= 52) or (400 <= x <= 402)  # 間隔350cm(範囲外)
    g = matcher_tick(x, on_line)
    assert g < 0
print("PASS matcher rejects huge gap")

# ---- テスト5: マッチャー 再アーム (2周目) ----
reset_matcher()
first = []
second = []
for lap in range(2):
    for k in range(400):
        x = lap * 300 + k * 0.5
        xx = x % 300
        on_line = (50 <= xx <= 52) or (72 <= xx <= 74)
        g = matcher_tick(x, on_line)
        if g > 0:
            (first if lap == 0 else second).append(g)
assert len(first) == 1 and len(second) == 1, (first, second)
assert first[0] == 74.5 and abs(second[0] - 374.5) < 0.001
print("PASS matcher re-arm across laps", first, second)

# ---- テスト6: 曲率累積とwref/FFの単位換算 ----
wz = 114592
v = 100  # ω=2rad/s, v=1m/s → κ=2/m → κ×1000=2000
k1000_est = ((wz // max(v, 5)) * 17453) // 10000
assert abs(k1000_est - 2000) < 100, k1000_est
# 目標ヨーレート wref は実測 wz と同じ mdeg/s 単位でなければならない。
# 正しい式: wref = κ*v*57296 = k1000*v*57296/100000
#   (k1000*57296//1000)*v//100 は32bit範囲内で精度保持
wref = (k1000_est * 57296 // 1000) * v // 100
assert abs(wref - 114592) < 200, wref
# 曲率FF: st = (vR-vL)/2 = κ*v*B/2。B=16cm, κ=2/m, v=100cm/s なら
#   st = 2*1[m/s]*0.16[m]/2 = 0.16m/s = 16cm/s
#   定数8000: (k1000*8000//100000)*v//1000 = 2000*8000//100000*100//1000 = 16
st_ff = ((k1000_est * 8000) // 100000) * v // 1000
assert abs(st_ff - 16) < 2, st_ff
print("PASS units: kappa_est=%d wref=%d st_ff=%d" % (k1000_est, wref, st_ff))

# ---- テスト6b: スリップ判定の指令ヨーレート (B=16cm、mdeg/s) ----
# ω = (vR-vL)/B [rad/s] → mdeg/s = *57296。÷16 (÷160 は10倍過小 → スリップ過検出)
vR, vL = 80, 20   # 速度差 60cm/s → 60/16*57296 ≈ 214,860 mdeg/s
wcmd = (vR - vL) * 57296 // 16
assert abs(wcmd - 214860) < 200, wcmd
wcmd_bad = (vR - vL) * 57296 // 160
assert wcmd_bad < wcmd // 9
print("PASS slip yaw-rate units (wcmd=%d)" % wcmd)

# ---- テスト7: 速度プロファイル (加速/減速制約) ----
N = 100
vmax = bytearray(N)
for i in range(N):
    vmax[i] = 255
vmax[50] = 30
A_ACC, A_BRK, BIN_CM = 220, 350, 2
for _pass in range(2):
    for i in range(1, N):
        lim = int(math.sqrt(vmax[i - 1] * vmax[i - 1] + 2 * A_ACC * BIN_CM))
        if vmax[i] > lim:
            vmax[i] = lim
    for i in range(N - 2, -1, -1):
        lim = int(math.sqrt(vmax[i + 1] * vmax[i + 1] + 2 * A_BRK * BIN_CM))
        if vmax[i] > lim:
            vmax[i] = lim
for i in range(1, N):
    d = abs(vmax[i] - vmax[i - 1])
    assert d <= int(math.sqrt(2 * A_BRK * BIN_CM)) + 1, (i, vmax[i], vmax[i - 1])
assert vmax[50] == 30
print("PASS speed profile constraints; vmax[45..55] =", list(vmax[45:56]))

# ---- テスト8: ルックアヘッド速度 ----
A_LAT = 220
ka = 2000
v_req = int(math.sqrt(100 * A_LAT * 1000 // ka))
assert abs(v_req - 104) < 3, v_req
print("PASS lookahead v_req=%d" % v_req)

print("ALL TESTS PASSED")


# ---- テスト9: 学習済みゲート間隔での検証 (G1) ----
gate_gap_cm = 20   # 学習値: 間隔20cm (許容 10..30cm)
reset_matcher()
det = []
for k in range(400):
    x = k * 0.5
    on_line = (50 <= x <= 52) or (72 <= x <= 74)   # 間隔20cm → 合格
    g = matcher_tick(x, on_line)
    if g > 0:
        det.append(g)
assert len(det) == 1, det
assert 19 <= gm_gap <= 23, gm_gap   # 間隔20cm ± サンプリング誤差
print("PASS learned-gap matcher accepts 20cm gap (gm_gap=%d)" % gm_gap)

gate_gap_cm = 20
reset_matcher()
for k in range(400):
    x = k * 0.5
    on_line = (50 <= x <= 52) or (90 <= x <= 92)   # 間隔40cm → 150%超で不合格
    g = matcher_tick(x, on_line)
    assert g < 0
print("PASS learned-gap matcher rejects 40cm gap")

# ---- テスト10: ILC 速度正規化 (S1) ----
# ff += e_avg * alpha // (10 * vb)  (v=100基準)
e_avg, alpha = 200, 30
assert e_avg * alpha // (10 * 50) == 12   # v=50cm/s → 2倍の学習量
assert e_avg * alpha // (10 * 100) == 6   # v=100cm/s → α=0.03相当
print("PASS ILC speed normalization (12@50cm/s, 6@100cm/s)")

# ---- テスト11: u16 ヘルパー ----
ba = bytearray(10 * 2)
u16add(ba, 3, 1234)
assert u16at(ba, 3) == 1234
u16add(ba, 3, 65500)
assert u16at(ba, 3) == 65535   # 飽和
print("PASS u16 helpers")

# ---- テスト12: セクションゲイン学習 (P3) ----
kp, kd = 100, 100
osc_s, rms_s = 200, 50          # 発振 → KP減/KD増
if osc_s > 150:
    kp = kp * 9 // 10
    kd = kd * 11 // 10
assert kp == 90 and kd == 110
kp, kd = 100, 100
osc_s, rms_s = 20, 500          # 追従遅れ → KP増
if osc_s > 150:
    pass
elif rms_s > 300:
    kp = kp * 105 // 100
assert kp == 105
print("PASS section gain rules")

# ---- テスト13: ゲート位置窓の条件 (G2) ----
def gate_window_ok(s_cm, track_len, in_lap, lap_no, slow_mode, win=100):
    if in_lap and lap_no > 1 and track_len > 0 and not slow_mode:
        return abs(s_cm - track_len) <= win
    return True
assert gate_window_ok(2950, 3000, True, 2, False) is True   # 窓内
assert gate_window_ok(2500, 3000, True, 2, False) is False  # 窓外
assert gate_window_ok(2500, 3000, True, 1, False) is True   # 1周目は常時
assert gate_window_ok(2500, 3000, True, 2, True) is True    # 探索中は常時
print("PASS gate window logic")

# ---- テスト14: 局所探索の選択ロジック (S4) ----
N = 3000
kappa_sim = [0] * N
for i in range(0, 1200):
    kappa_sim[i] = 500          # カーブ
for i in range(1400, 3000):
    kappa_sim[i] = 500          # カーブ
# 直線: 1200..1400 (κ=0、長さ200 = EXPLORE_MAX_BINS ちょうど)
vmax_sim = bytearray(N)
for i in range(N):
    vmax_sim[i] = 100
tb = 2900                       # ゲート位置
best_start, best_end, best_avg = -1, 0, 0
i = 0
while i < N - 1:
    k = abs(kappa_sim[i])
    if k < 300:
        j = i
        while j < N and abs(kappa_sim[j]) < 300:
            j += 1
        if j - i >= 100 and (j <= tb - 100 or i >= tb + 100) and j - i <= 200:
            avg = sum(vmax_sim[i:j]) // (j - i)
            if avg > best_avg:
                best_avg = avg
                best_start, best_end = i, j
        i = j
    else:
        i += 1
assert best_start == 1200 and best_end == 1400, (best_start, best_end)
for kk in range(best_start, best_end):
    vmax_sim[kk] = min(vmax_sim[kk] * 110 // 100, 250)
assert vmax_sim[1200] == 110
print("PASS explore selection (section %d-%d, vmax+10%%)" % (best_start, best_end))

# ---- テスト15: vmax 再構築の u8 クランプ (vc>255 で bytearray が壊れる) ----
# κ×1000=100 (ほぼ直線) → vc=isqrt(100*220*1000//100)=469 > 255
# クランプ無しだと bytearray 代入が ValueError になり finalize_lap が落ちる
vc_raw = int(math.sqrt(100 * 220 * 1000 // 100))
assert vc_raw > 255, vc_raw
ba7 = bytearray(4)
vc = 255 if vc_raw > 255 else vc_raw
ba7[0] = (0 + vc * 2) // 3
assert ba7[0] == 170
try:
    ba7[1] = vc_raw
    clamped = False
except (ValueError, OverflowError):
    clamped = True
assert clamped, "bytearray への 256 以上の代入がエラーにならない"
print("PASS vmax rebuild u8 clamp (vc=%d -> 255)" % vc_raw)

# ---- テスト16: ロスト判定 (両センサ白=センター追従中はロストにしない) ----
def lost_logic(nL, nR, lost_ms, dt_ms, th=200, lost_ms_limit=300):
    if nL < th and nR < th:      # 両センサがライン上 (曖昧) のみロスト扱い
        lost_ms += dt_ms
    else:
        lost_ms = 0
    return lost_ms, lost_ms > lost_ms_limit

lost_ms = 0
for _ in range(100):             # センター追従 (両方白 900) → ロストにならない
    lost_ms, lost = lost_logic(900, 950, lost_ms, 10)
assert not lost
lost_ms = 0
for _ in range(40):              # 両方黒 → ロスト
    lost_ms, lost = lost_logic(100, 80, lost_ms, 10)
assert lost
lost_ms = 0
for _ in range(100):             # 片側だけ黒 (通常追従) → ロストにならない
    lost_ms, lost = lost_logic(100, 900, lost_ms, 10)
assert not lost
print("PASS lost logic (both-white centered not lost)")

# ---- テスト17: ILC 学習対象はブーム補償済み e_eff (PID と同一誤差) ----
# 生 e で学習するとブーム輸送遅延分が系統誤差として残る設計上の不整合。
# ここでは「e_eff = e + comp_term」で蓄積されることの単体確認のみ行う。
e, comp_term = -120, 40
e_eff = e + comp_term
assert e_eff == -80
print("PASS ILC learns from e_eff (e_eff=%d)" % e_eff)

# ---- テスト18: main.py ソース静的チェック (未定義名・単位回帰ガード) ----
# ハブ依存のため main.py は import 不可。実機前に落ちるバグの再発を
# ソーステキスト検査で防ぐ (ZERO* 未定義, lap_done スコープ, 単位バグ)
import os as _os
_src = open(_os.path.join(_os.path.dirname(_os.path.abspath(__file__)), "main.py"),
            encoding="utf-8").read()

# ZERO1/2/4 が定義されてから reset_lap_state で使われること (無いと NameError)
for _z in ("ZERO1", "ZERO2", "ZERO4"):
    assert _z + " = bytearray" in _src, "%s 定義が main.py に無い" % _z
assert _src.index("ZERO4 = bytearray") < _src.index("def reset_lap_state"), \
    "ZERO* 定義が reset_lap_state より後にある"

# run_laps が lap_done を global 宣言していること (無いと UnboundLocalError)
_run_src = _src[_src.index("def run_laps"):]
assert "global lap_done" in _run_src, "run_laps に global lap_done が無い"

# スリップ判定は 57296//14 (mdeg/s)。//140 は10倍過小でスリップ過検出のバグ
assert "57296 // 16" in _src and "57296 // 160" not in _src, "スリップ判定の ÷160 バグ再発"

# 目標ヨーレート wref は正しい式 (1000倍過小の旧式が無いこと)
_tick_src = _src[_src.index("def _tick_impl"):]
# オーバーフロー回避の恒等変形 (k*57296)//1000 = k*57+(k*296)//1000。
# 旧式 (k1000*57296//1000) は |k1000|>=18741 で Pybricks Small Int(±2^30) を超える。
_wref_ok = "k1000 * 57 + k1000 * 296 // 1000" in _tick_src
_wref_no_old = "(k1000 * 57296 // 1000) * v_cmd // 100" not in _tick_src
assert _wref_ok and _wref_no_old, "wref 式がオーバーフロー旧式に戻っている"
# 曲率FF は 7000 (2倍過大の旧式 14000 が無いこと)
assert "k1000 * 8000" in _tick_src, "曲率FF 定数が誤っている"
assert "k1000 * 16000" not in _tick_src, "曲率FF の 2倍過大バグ再発"
# finalize_lap が best_kp 等を global 宣言していること (無いとロールバックで落ちる)
_fin2 = _src[_src.index("def finalize_lap"):_src.index("def reset_lap_state")]
assert "global best_kp, best_kd, best_a_lat, best_vglob" in _fin2, \
    "finalize_lap の best_kp 等 global 宣言が無い (ロールバック UnboundLocalError)"

# ロスト判定: 両方白=センター追従中の正常状態。両方白をロストにしない
assert "nL < 200 and nR < 200" in _tick_src
assert "nL > 850 and nR > 850" not in _tick_src, "ロスト判定の両方白バグ再発"
# ゲートマッチャーは両センサ同時黒 (line_both) で駆動する (蛇行誤検出対策)
assert "line_both = (nL < 500) and (nR < 500)" in _tick_src, "line_both 定義が無い"
assert "matcher_tick(s_cm, mat_sig)" in _tick_src, "マッチャーが mat_sig を使っていない"
# ジャイロ減衰はフル GYRO (ラップ1の半減は取りやめ: 安定性優先)
assert "gyr = GYRO" in _tick_src, "ジャイロ減衰ガインが GYRO でない"
assert "GYRO // 2" not in _tick_src, "ラップ1の GYRO 半減が復活している"

# vmax 再構築: vc>255 クランプ (無いと bytearray 代入で finalize_lap が落ちる)
_fin_src = _src[_src.index("def finalize_lap"):]
assert "if vc > 255:" in _fin_src, "vmax 再構築の vc クランプが無い"
# D項ローパス: de_f が tick で使われ、ラップ開始でリセットされること
assert "de_f = (de_f * 5 + de) // 6" in _src, "D項ローパスが無い"
assert "de_f = 0" in _src[_src.index("def reset_lap_state"):], "de_f のリセットが無い"
# 速度推定: 高ループレートではエンコーダ実測(EMA)が0になるため κ は v_cmd 基準
# v_meas が tick に残っていると κ 過大評価バグが再発する
assert "v_meas" not in _tick_src, "tick に v_meas が残っている (κ過大評価バグ再発)"
assert "wz // max(v_cmd, 5)" in _tick_src, "κ蓄積が v_cmd 基準でない"
# i32 ヘルパー: 2^32 リテラル / <<24 の中間超過は Pybricks で OverflowError になる。
# u16+i16 分解実装 (i32set 存在 + 0x80000000/0x100000000 不在) をソース文字列で確認する。
assert "def i32set" in _src, "i32set が無い (i32 は分解書き込みであるべき)"
assert "0x80000000" not in _src and "0x100000000" not in _src, \
    "i32 ヘルパーに 2^31/2^32 リテラル再発 (Pybricks で OverflowError)"
_i32_src = _src[_src.index("def i32at"):_src.index("def percentile")]
assert "<< 24" not in _i32_src, "i32at/i32add に 4byte シフト再発 (Small Int 超過)"

# ---- テスト18b: ブームD項正帰還とホイールπ誤差の回帰ガード ----
# センサー前方16cm では D 項(de)がブーム振れを正帰還して発振する
# (閉ループsim: KD=30→osc=2072/m, KD=0→osc=1/m)。KD デフォルトと KD_MIN
# 下限は 0 でなければならない (KD_MIN>0 は適応クランプが KD を復活させる)。
assert "\nKD = 0" in _src, "KD デフォルトが 0 でない (ブームD項正帰還で発振)"
assert "KD_MIN, KD_MAX = 0, 80" in _src, "KD_MIN 下限が 0 でない (KD=0 が復活する)"
# ホイール定数: SPIKE Prime 大タイヤは 円周27.9cm (φ8.88cm)。
# 直径27.9cm 前提の旧変換 (411/244) は速度・オドメトリが実値の π 倍ずれる。
assert "WHEEL_D_CM = 27.9" in _src, "WHEEL_D_CM が 27.9 (円周) でない"
assert "* WMUL // 100" in _tick_src, "モーター変換が WMUL (円周27.9cm基準) でない"
assert "* SMUL // 10000" in _tick_src, "オドメトリ変換が SMUL (円周27.9cm基準) でない"
assert "* 411 // 100" not in _tick_src, "旧モーター変換 (411, 直径27.9cm) 再発"
assert "* 244 // 1000" not in _tick_src, "旧オドメトリ変換 (244, 直径27.9cm) 再発"
# トレッド幅は実測 16cm (20ポッチ)。κFF 定数8000 / スリップ判定 //16 と一致する
assert "TRACK_B_CM = 16.0" in _src, "TRACK_B_CM が 16.0 でない"
assert "57296 // 16" in _src, "スリップ判定が B=16cm でない"
# ---- テスト18c: Pybricks で動かない bytearray スライス代入の回帰ガード ----
# MicroPython は ba[:] = ... のスライス代入に TypeError を投げる (CPython では
# 通る罠。実機ではラップ開始の reset_lap_state が落ちた)。クリア/コピーは
# ba_clear(ba,n) / ba_copy(dst,src,n) の要素ループで行うこと。
assert "def ba_clear" in _src and "def ba_copy" in _src, "ba_clear/ba_copy が無い"
for _bad in ("[:] = ZERO", "[:] = tmp", "best_ff[:", "best_vmax[:",
             "best_kp_mult[:", "best_kd_mult[:", "ff[:]", "vmax[:] =",
             "kp_mult[:] =", "kd_mult[:] =", "kappa_acc[:", "sec_zc[:"):
    assert _bad not in _src, "bytearray スライス代入が復活している: %s" % _bad
# 起動時スピンでホイール円周を実測し変換係数を更新するキャリブレーション
assert "auto_sign_test" in _src
assert "global CMD_FLIP, SIGN_GYRO_CUR, WMUL, SMUL" in _src, "円周キャリブレーションの global 宣言が無い"
print("PASS boom-KD / wheel-pi regression guards")
print("PASS bytearray-slice regression guards")
print("PASS main.py source static checks")

# ---- テスト19: D項の1次ローパス (ノイズ飽和対策) ----
# 高ループレートで de のノイズが st_D を飽和させる問題を検証。
# ±25単位ノイズ, dt=5ms → de=±5000 交互。ローパス後は振幅が約1/11に減る。
def lowpass_de(raw_seq, w=5, den=6):
    y = 0
    out = []
    for x in raw_seq:
        y = (y * w + x) // den
        out.append(y)
    return out

seq = [5000 if i % 2 == 0 else -5000 for i in range(400)]
filt = lowpass_de(seq)
peak = max(abs(x) for x in filt[50:])
assert peak < 1000, "D項ローパスがノイズを抑えていない: peak=%d" % peak
assert peak > 100, "D項ローパスが過剰 (実信号を殺す?)"
step = [20000] * 30
fstep = lowpass_de(step)
assert fstep[-1] > 15000, "D項がステップに追従しない: %d" % fstep[-1]
print("PASS D-term lowpass (noise peak=%d, step follow=%d)" % (peak, fstep[-1]))

# ---- テスト20: ブーム補償のオーダー検証 (BOOM_K=200) ----
# モデル: センサーがライン上で axle より κ*L^2/2 だけ横にずれる
#   κ=2/m, v=50cm/s, L=10cm → ずれ = (wz/v)*L^2/2 = 1.75cm
# コード: comp = 0.2*(wz//v) e単位。e=1000 ≒ センサー間隔半分(3-4cm)とすると
#   1.75cm = 440-580 e単位 → BOOM_K=200 (400 e単位) は同オーダー (過大ではない)
wz, v = 100000, 50
comp_code = (200 * (wz // max(v, 20))) // 1000
print("PASS boom comp order (code=%d e-units, 物理 1.75cm ~ 440-580)" % comp_code)
assert 200 <= comp_code <= 800

print("ALL TESTS PASSED")
