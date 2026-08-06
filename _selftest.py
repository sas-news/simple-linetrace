# -*- coding: utf-8 -*-
"""main.py (LINE RACER SLIM) の純ロジック自己テスト (PC上で検証、ハブでは実行しない)
main.py の純関数と同一ロジックを保つこと。main.py を変えたら必ずここも更新する。"""
import math

# ---- main.py から移植した純関数 ----
def clamp(v, lo, hi):
    if v < lo:
        return lo
    if v > hi:
        return hi
    return v


def isqrt(x):
    return int(math.sqrt(x))


def u16at(ba, i):
    j = i * 2
    return ba[j] | (ba[j + 1] << 8)


def marker_tick(s, both_dark):
    global mk_armed, mk_state, mk_s0, mk_last
    MARK_MIN, MARK_MAX, MARK_REARM = 0.5, 10, 40
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


def reset_marker():
    global mk_armed, mk_state, mk_s0, mk_last
    mk_armed = True
    mk_state = 0
    mk_s0 = 0
    mk_last = -1000


# 正規化 (main.py tick と同一式)
def norm(raw, cal_w, cal_b):
    w = cal_w - cal_b
    if w < 1:
        w = 1
    n = (raw - cal_b) * 1000 // w
    if n < 0:
        return 0
    if n > 1000:
        return 1000
    return n


def calc_e(nL, nR, sign_st=1):
    e = (nR - nL) * 1000 // (nL + nR + 1)
    if sign_st < 0:
        e = -e
    return e


def slow_in_curve(wz, v_cruise=70, v_min=15, turn_full=30000):
    a_wz = wz
    if a_wz < 0:
        a_wz = -a_wz
    if a_wz > turn_full:
        a_wz = turn_full
    slow = (v_cruise - v_min) * a_wz // turn_full
    vt = v_cruise - slow
    if vt < v_min:
        vt = v_min
    if vt > v_cruise:
        vt = v_cruise
    return vt


# ============================== テスト ==============================

# ---- テスト1: u16at ----
ba = bytearray(10 * 2)
ba[6] = 0x34
ba[7] = 0x12
assert u16at(ba, 3) == 0x1234, u16at(ba, 3)
print("PASS u16at")

# ---- テスト2: isqrt / clamp ----
assert isqrt(0) == 0 and isqrt(225) == 15 and isqrt(26) == 5
assert clamp(5, 0, 10) == 5 and clamp(-3, 0, 10) == 0 and clamp(99, 0, 10) == 10
print("PASS isqrt/clamp")

# ---- テスト3: 正規化 (実測 cal) ----
assert norm(96, 96, 13) == 1000    # 白 → 1000
assert norm(13, 96, 13) == 0       # 黒 → 0
assert norm(95, 95, 12) == 1000    # R白
assert norm(12, 95, 12) == 0       # R黒
print("PASS normalization (cal 96/13, 95/12)")

# ---- テスト4: 誤差 e ----
# 中心追従 (両センサ高) → e 小
assert abs(calc_e(900, 900)) < 10, calc_e(900, 900)
# ライン右 → nR低, nL高 → e<0 (右に曲がる向き)
e_r = calc_e(900, 100)
assert e_r < 0, e_r
# SIGN_ST 反転
assert calc_e(900, 100, -1) > 0
print("PASS error polarity (e_right=%d)" % e_r)

# ---- テスト5: マーカー検出 正常 (単一横断: 幅2cm) ----
reset_marker()
det = []
for k in range(200):
    x = k * 0.5
    dark = (50 <= x <= 52)
    g = marker_tick(x, dark)
    if g > 0:
        det.append(g)
assert len(det) == 1 and abs(det[0] - 52.5) < 0.001, det
print("PASS marker normal at", det)

# ---- テスト6: マーカー誤検出排除 ----
reset_marker()
rej = 0
for k in range(700):
    x = k * 0.1
    dark = (50 <= x <= 50.2)   # 幅0.2cm → MARK_MIN(0.5)未満で拒否されるはず
    g = marker_tick(x, dark)
    if g > 0:
        rej += 1
assert rej == 0, "narrow marker accepted, rej=%d" % rej
print("PASS marker rejects too-narrow")

reset_marker()
for k in range(200):
    x = k * 0.5
    dark = (50 <= x <= 65)     # 幅15cm → 範囲外
    g = marker_tick(x, dark)
    assert g < 0, "wide marker accepted"
print("PASS marker rejects too-wide")

# ---- テスト7: マーカー再アーム (連続ラップ) ----
reset_marker()
first = []
second = []
for lap in range(2):
    for k in range(400):
        x = lap * 300 + k * 0.5
        xx = x % 300
        dark = (50 <= xx <= 52)
        g = marker_tick(x, dark)
        if g > 0:
            (first if lap == 0 else second).append(g)
assert len(first) == 1 and len(second) == 1, (first, second)
assert first[0] == 52.5 and abs(second[0] - 352.5) < 0.001, (first, second)
print("PASS marker re-arm across laps", first, second)

# ---- テスト8: 旋回減速 (slow-in-curve) ----
assert slow_in_curve(0) == 70          # 直線 → 巡航
assert slow_in_curve(30000) == 15      # 全旋回 → V_MIN
assert slow_in_curve(15000) == 43      # 中間
print("PASS slow-in-curve (70@0, 43@15000, 15@30000)")

# ---- テスト9: main.py ソース静的チェック ----
import os as _os
_src = open(_os.path.join(_os.path.dirname(_os.path.abspath(__file__)), "main.py"),
            encoding="utf-8").read()

# 符号は実測値に固定 (起動時自動テスト廃止)
assert "CMD_FLIP = +1" in _src, "CMD_FLIP が +1 固定でない"
assert "SIGN_GYRO_CUR = -1" in _src, "SIGN_GYRO_CUR が -1 固定でない"
assert "auto_sign_test" not in _src, "起動時自動テストが残っている"
# 反射光キャリブレーションは実測値
assert "calL_w, calL_b = 96, 13" in _src, "calL 実測値が無い"
assert "calR_w, calR_b = 95, 12" in _src, "calR 実測値が無い"
# 旧2本線マッチャーと複雑な学習の撤去
assert "def matcher_tick" not in _src, "旧2本線マッチャーが残っている"
assert "gm_state" not in _src, "旧マッチャー状態が残っている"
assert "kappa =" not in _src, "旧 κ 学習テーブルが残っている"
assert "def finalize_lap" in _src  # (finalize_lap は残す)
# マーカーは両センサ同時暗で駆動
assert "both_dark = (nL < 350) and (nR < 350)" in _src, "both_dark 判定が無い"
assert "marker_tick(s_cm, both_dark)" in _src, "marker_tick 呼び出しが無い"
# bytearray スライス代入の禁止 (Pybricks で TypeError)
for _bad in ("[:] =", "[:]="):
    assert _bad not in _src, "bytearray スライス代入が残っている: %s" % _bad
# ポート実測確定
assert "PORT_LM = Port.A" in _src and "PORT_RM = Port.E" in _src
assert "PORT_LS = Port.B" in _src and "PORT_RS = Port.F" in _src
# 学習FF(ILC)が有る (補正なし:Fはmain.pyと同一ロジックにすること)
assert "def i16at" in _src and "def i16set" in _src, "i16 ヘルパーが無い"
assert "err_cnt" in _src, "ILC 蓄積テーブルが無い"
assert "ILC_ALPHA" in _src, "ILC_ALPHA が無い"
assert "st += i16at(ff, b) * v_cmd // 100" in _src, "FF 適用式が無い"
assert "f +=" in _src, "ILC 更新式が無い"
print("PASS main.py source static checks")

# ---- テストX: i16 ヘルパー + ILC 更新ロジック (main.py と同一式) ----
def i16at(ba, i):
    j = i * 2
    v = ba[j] | (ba[j + 1] << 8)
    if v >= 32768:
        v -= 65536
    return v


def i16set(ba, i, v):
    if v > 32767:
        v = 32767
    elif v < -32768:
        v = -32768
    j = i * 2
    ba[j] = v & 255
    ba[j + 1] = (v >> 8) & 255


ba = bytearray(10 * 2)
for val in (0, 1, -1, 32767, -32768, 200, -200, 12345):
    i16set(ba, 3, val)
    assert i16at(ba, 3) == val, "i16 roundtrip %d->%d" % (val, i16at(ba, 3))
i16set(ba, 3, 40000)
assert i16at(ba, 3) == 32767, "i16 clamp hi"
i16set(ba, 3, -40000)
assert i16at(ba, 3) == -32768, "i16 clamp lo"
print("PASS i16 helpers (roundtrip+clamp)")


# ILC 更新: ff += e_avg * ILC_ALPHA // (10 * vb), クランプ ±FF_MAX
def ilc_update(f, e_avg, vb, alpha=20, ff_max=40):
    if vb < 10:
        vb = 10
    f += e_avg * alpha // (10 * vb)
    if f > ff_max:
        f = ff_max
    elif f < -ff_max:
        f = -ff_max
    return f


assert ilc_update(0, 500, 50) == 20, ilc_update(0, 500, 50)      # 500*20//500=20
assert ilc_update(0, 500, 100) == 10, ilc_update(0, 500, 100)
assert ilc_update(30, 500, 50) == 40, "FF_MAXでクランプ"          # 30+20=50>40→40
assert ilc_update(-30, -500, 50) == -40, "負側クランプ"
assert ilc_update(0, 500, 5) == 40, "v<10→10で計算, 100→FF_MAX=40"
print("PASS ILC update (speed-normalized ff += e*alpha/(10*v))")

# ---- テストY: レーシング則 (race_wheels) + 速度計画 (vmax_from_stn) ----
def race_wheels(v_plan, st, comp, v_min):
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


# st>0 = 左旋回 → 右輪が外輪(fast)
vL, vR = race_wheels(70, +20, 1000, 12)
assert vL == 30 and vR == 70, (vL, vR)
vL, vR = race_wheels(70, -20, 1000, 12)
assert vL == 70 and vR == 30, (vL, vR)
vL, vR = race_wheels(70, 0, 1000, 12)
assert vL == 70 and vR == 70, (vL, vR)
# 急旋回: 内輪 = 30-40 = -10 (>= -V_MIN なのでクランプ不要)
vL, vR = race_wheels(30, +20, 1000, 12)
assert vL == -10 and vR == 30, (vL, vR)
# 内輪 < -V_MIN になるときのみクランプ
vL, vR = race_wheels(20, +20, 1000, 12)
assert vL == -12 and vR == 20, (vL, vR)
# comp 適用
vL, vR = race_wheels(40, 10, 1250, 12)
assert vR == 50 and vL == 50 - 20 == 30, (vL, vR)
print("PASS race_wheels (one-wheel-fast, inner clamp, comp)")


def vmax_from_stn(stn, a_lat_max, v_race_max):
    kk = 125 * stn
    if kk > 12500:
        kk = 12500
    if kk < 1:
        kk = 1
    vc = int(math.sqrt(a_lat_max * 100000 // kk))
    vk = 481250 // kk
    vm = vc if vc < vk else vk
    if vm > v_race_max:
        vm = v_race_max
    return vm


assert vmax_from_stn(40, 200, 70) == 63      # κ1000=5000, v_curv=63
assert vmax_from_stn(80, 200, 70) == 44      # κ1000=10000, v_curv=44 (v_kin=48)
assert vmax_from_stn(100, 200, 70) == 38     # 急曲線 → v_kin=38
assert vmax_from_stn(300, 200, 70) == 38     # 更に急でも κ1000クランプ
assert vmax_from_stn(0, 200, 70) == 70       # 直線 → 上限
print("PASS vmax_from_stn (curvature-based speed limit)")

print("ALL TESTS PASSED")
