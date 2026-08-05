# -*- coding: utf-8 -*-
"""
main.py の PC 統合スモークテスト (pybricks モック)
==================================================
実機なしで main.py の tick / ゲート検出 / finalize_lap / 学習更新が
実行時エラーなしで動くことを検証する。ハブ依存の API はすべてモックし、
モータ位置からスクリプトしたセンサ/ジャイロを返す簡易閉ループで走らせる。

実行:  python _smoke_test.py   (main.py / _selftest.py と同じディレクトリ)

検証項目:
  - find_gate (ゲート探索) がクラッシュせずゲートを検出する
  - 6ラップがクラッシュせず完了し、学習テーブルが更新される
  - 強制ロールバック (2連続悪化) がクラッシュしない
  - ゲート読み飛ばし → 低速探索 → 緊急ラップ終了がクラッシュしない
"""
import math
import os
import sys

# ============================================================================
# pybricks モック (PC 上で main.py を import するための最小実装)
# ============================================================================

LAP_LEN = 1200.0       # コース長 cm (ゲート周期)
TRACK_B_SIM = 16.0     # トレッド幅 cm (姿勢モデル用、実機20ポッチ=16cmと同期)
SENSOR_OFF = 3.0       # センサーのライン中心からの横オフセット cm


class _Sim:
    """閉ループ状態 (横ずれ・方位) とモータ実体への参照"""
    lat = 0.0
    head = 0.0
    motors = None       # (mL, mR) モックモータ (import 後に設定)


SIM = _Sim()


class _Motor:
    def __init__(self, port, positive_direction=None):
        self.angle_deg = 0.0
        self.speed = 0.0          # deg/s (run で指定)

    def run(self, spd):
        self.speed = float(spd)

    def angle(self):
        return int(self.angle_deg)

    def stop(self):
        self.speed = 0.0

    def _advance(self, dt_ms):
        self.angle_deg += self.speed * dt_ms / 1000.0


def _pos_cm():
    """モータ平均角から絶対位置 (cm) を推定 (main.py のオドメトリと同式)"""
    a = (SIM.motors[0].angle_deg + SIM.motors[1].angle_deg) / 2.0
    return a * 775.0 / 10000.0


def _sensor_val(dist):
    """センサーとライン中心の距離 [cm] → 反射率 0..100 (連続モデル)"""
    if dist < 1.5:
        return 10
    if dist < 4.0:
        return int(10 + (dist - 1.5) * 80.0 / 2.5)
    return 90


class _Sensor:
    def __init__(self, port):
        # main.py の PORT_LS/PORT_RS と同期 (現在: 左=B / 右=F、旧: 左=C / 右=D)
        self.side = "L" if str(port) in ("B", "C") else "R"

    def reflection(self):
        x = _pos_cm()
        xm = x % LAP_LEN
        # ゲート: 絶対位置の周期 LAP_LEN ごとに「線-白-線」パターン
        if (2.0 <= xm < 4.0) or (22.0 <= xm < 24.0):
            return 10                     # 両センサが垂直ライン上
        if 4.0 <= xm < 22.0:
            return 90                     # ゲートの白ギャップ
        # 通常ライン追従: 中心 (y=0) に半幅 1.5cm のライン
        off = SENSOR_OFF if self.side == "L" else -SENSOR_OFF
        return _sensor_val(abs(SIM.lat + off))


class _IMU:
    def angular_velocity(self, axis):
        # 実機 Pybricks は deg/s の float を返す。main.py 側で int(値*1000) と
        # mdeg/s に変換するため、ここも deg/s の float を返す (内部計算は mdeg/s)。
        x = _pos_cm()
        xm = x % LAP_LEN
        if 200.0 <= xm < 300.0:
            k = 1500.0                    # カーブ1 (κ×1000)
        elif 700.0 <= xm < 800.0:
            k = -2000.0                   # カーブ2 (逆方向)
        else:
            k = 0.0
        # 中心速度基準: mdeg/s = κ*v*57296/100000 (v = 両輪の平均)→ deg/s で返す
        # 左輪だけを使うとステアリング中に κ が歪む (系統誤差になる)
        v_cm = ((SIM.motors[0].speed + SIM.motors[1].speed) / 2.0) * 100.0 / 1290.0
        return (k * v_cm * 57296 / 100000) / 1000.0

    def acceleration(self, axis):
        return 0.0


class _Battery:
    def voltage(self):
        return 7600


class _Light:
    def on(self, color):
        pass


class _Speaker:
    def beep(self, *a):
        pass


class _Display:
    def number(self, n):
        pass

    def off(self):
        pass


class _System:
    def set_stop_button(self, *a):
        pass


class _Buttons:
    def pressed(self):
        return []


class _Hub:
    def __init__(self):
        self.imu = _IMU()
        self.battery = _Battery()
        self.light = _Light()
        self.speaker = _Speaker()
        self.display = _Display()
        self.system = _System()
        self.buttons = _Buttons()   # hub.buttons.pressed() 用 (属性)


_CLK = 0   # モック時計 (time() と wait() が共有。sim_loop も進める)


class _StopWatch:
    def __init__(self):
        self.t = 0

    def time(self):
        return int(_CLK)


def _native(f):
    return f


def _mem_info(*a, **k):
    pass


def _wait(ms):
    global _CLK
    _CLK += ms


# ---- パラメータモック ----
class _Port:
    A, B, C, D, E, F = "A", "B", "C", "D", "E", "F"


class _Dir:
    COUNTERCLOCKWISE, CLOCKWISE = "CCW", "CW"


class _Btn:
    CENTER, LEFT, RIGHT = "CENTER", "LEFT", "RIGHT"


class _Color:
    GREEN, ORANGE, RED = "GREEN", "ORANGE", "RED"


class _Axis:
    X, Z = "X", "Z"


def _install_mocks():
    """pybricks / micropython モジュールを sys.modules に注入してから import"""
    import types
    px = {}
    for name in ("pybricks", "pybricks.hubs", "pybricks.pupdevices",
                 "pybricks.parameters", "pybricks.tools", "micropython"):
        m = types.ModuleType(name)
        m.__file__ = "<mock>"
        px[name] = m
    px["pybricks"].hubs = px["pybricks.hubs"]
    px["pybricks"].pupdevices = px["pybricks.pupdevices"]
    px["pybricks"].parameters = px["pybricks.parameters"]
    px["pybricks"].tools = px["pybricks.tools"]
    px["pybricks.hubs"].PrimeHub = _Hub
    px["pybricks.pupdevices"].Motor = _Motor
    px["pybricks.pupdevices"].ColorSensor = _Sensor
    p = px["pybricks.parameters"]
    p.Port, p.Direction, p.Button, p.Color, p.Axis = _Port, _Dir, _Btn, _Color, _Axis
    t = px["pybricks.tools"]
    t.StopWatch, t.wait = _StopWatch, _wait
    px["micropython"].native, px["micropython"].mem_info = _native, _mem_info
    sys.modules.update(px)


_install_mocks()

# main.py を import (pybricks モック経由、__main__ ガードで本体は実行されない)
here = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, here)
import main as M

SIM.motors = (M.mL, M.mR)

STEP_MS = 5


def advance(dt_ms):
    """モータ位置・姿勢を進める (tick の後に呼ぶ)"""
    M.mL._advance(dt_ms)
    M.mR._advance(dt_ms)
    vL = M.mL.speed * 100.0 / 1290.0        # deg/s → cm/s (円周27.9cm)
    vR = M.mR.speed * 100.0 / 1290.0
    v = (vL + vR) / 2.0
    omega = (vR - vL) / TRACK_B_SIM        # rad/s
    SIM.head += omega * dt_ms / 1000.0
    SIM.lat += v * math.sin(SIM.head) * dt_ms / 1000.0


def sim_loop(cond, guard_limit, on_step):
    """cond() が True の間 tick を回す (find_gate / ラップ の共通ループ)"""
    global _CLK
    guard = 0
    while cond() and guard < guard_limit:
        _CLK += STEP_MS                # 時計を固定周期で進める
        t_ms = M.sw.time()
        dt = max(t_ms - M.t_prev, 1)
        M.t_prev = t_ms
        M.tick(dt, t_ms)
        if t_ms - M.last_hk >= 10:
            M.hk(t_ms)
        on_step()
        advance(dt)
        guard += 1
    if guard >= guard_limit:
        raise RuntimeError("sim loop guard exceeded")


def sim_find_gate():
    print("-- find_gate --")
    M.in_lap = False
    M.lap_done = False
    M.t_prev = 0
    M.last_hk = 0
    t0 = M.sw.time()
    sim_loop(lambda: not M.lap_done and M.sw.time() - t0 < 60000,
             20000, lambda: None)
    if not M.lap_done:
        raise RuntimeError("gate not found in find_gate")
    print("gate found at s=%d (gap=%d)" % (M.gate_s, M.gate_gap_last))


def sim_laps(n_laps):
    M.lap_no = 0
    explore_ever = [False]   # ラップ4以降のどこかで探索が予約されればOK
    for _ in range(n_laps):
        M.lap_no += 1
        M.in_lap = True
        M.reset_lap_state()
        M.lap_start_t = M.sw.time()
        M.last_hk = 0
        print("-- LAP %d start --" % M.lap_no)

        def on_tick():
            if M.in_lap and M.track_len > 0 and not M.slow_mode \
                    and M.s_cm > M.track_len + M.GATE_MISS_CM:
                M.slow_mode = True
                M.lap_valid = False
            if M.in_lap and M.slow_mode and M.s_cm > M.track_len * 2 + 900:
                M.gate_s = M.track_len
                M.lap_done = True

        sim_loop(lambda: not M.lap_done, 200000, on_tick)
        t_lap = M.sw.time() - M.lap_start_t - M.gate_f * M.gate_dt // 100
        M.finalize_lap(t_lap)
        if M.lap_no >= 4 and M.explore_active:
            explore_ever[0] = True
        if M.lap_no == 8:
            assert explore_ever[0], "explore never scheduled by lap 8"
    print("laps done: %d, track=%d best=%d" % (M.lap_no, M.track_len, M.best_time))


def sim_boot_sign_test():
    """起動時自動テスト (符号判定 + ホイール円周キャリブレーション) が
    クラッシュせず終了すること。ジャイロは実際のモーター速度差から模擬し、
    スピンテスト→円周キャリブレーションの測定経路を通す (モックはホイール角が
    進まないため「did not measure」の WARN 経路で終了する)。"""
    print("-- boot sign test (no-crash) --")
    M.CMD_FLIP = 1
    M.SIGN_GYRO_CUR = 1
    M.wz_bias = 0
    M.WMUL = 1290
    M.SMUL = 775

    def fake_gyro(axis):
        d = M.mR.speed - M.mL.speed     # スピン時の実ヨーレート (deg/s、符号付き)
        if d > 0:
            return 66.6
        if d < 0:
            return -66.6
        return 0.0

    M.hub.imu.angular_velocity = fake_gyro   # クラスメソッドをインスタンス属性で影
    try:
        M.auto_sign_test()
    finally:
        del M.hub.imu.angular_velocity
    assert M.CMD_FLIP in (1, -1)
    assert M.SIGN_GYRO_CUR in (1, -1)
    print("boot sign test ok (CMD_FLIP=%d SIGN_GYRO=%d WMUL=%d)" %
          (M.CMD_FLIP, M.SIGN_GYRO_CUR, M.WMUL))


def sim_rollback():
    print("-- forced rollback --")
    M.best_time = 10000
    M.lap_no = 5
    M.bad_streak = 0
    M.lap_valid = True       # invalid 状態だと finalize が早期 return するため
    M.slow_mode = False
    M.finalize_lap(15000)   # 15000 > 10000*103//100 → bad_streak=1
    M.finalize_lap(15000)   # bad_streak=2 → ロールバック発動
    assert M.bad_streak == 0, "rollback did not reset bad_streak"
    print("rollback ok (bad_streak reset)")


def sim_gate_miss():
    print("-- gate miss (emergency lap end) --")
    M.lap_no += 1
    M.in_lap = True
    M.reset_lap_state()
    M.lap_start_t = M.sw.time()
    M.slow_mode = True      # 強制的に読み飛ばし状態へ
    M.lap_valid = False

    def on_tick():
        if M.in_lap and M.slow_mode and M.s_cm > M.track_len * 2 + 900:
            M.gate_s = M.track_len
            M.lap_done = True

    sim_loop(lambda: not M.lap_done, 300000, on_tick)
    t_lap = M.sw.time() - M.lap_start_t - M.gate_f * M.gate_dt // 100
    M.finalize_lap(t_lap)   # invalid lap → 学習スキップで return
    assert M.lap_valid is False
    print("gate miss handled ok")


def main():
    print("=== smoke test: main.py on PC (pybricks mocked) ===")
    sim_boot_sign_test()
    sim_find_gate()
    sim_laps(6)
    sim_rollback()
    sim_gate_miss()

    # ---- 学習テーブルの健全性 ----
    nz = sum(1 for i in range(M.N_BINS) if M.kappa[i * 2] or M.kappa[i * 2 + 1])
    assert nz > 100, "kappa table empty"
    assert M.track_len > 0, "track_len not set"
    assert M.best_time > 0, "best_time not set"
    assert max(M.vmax) > M.V_MIN, "vmax not grown"
    print("table health: kappa_nz=%d track=%d best=%d vmax_max=%d" %
          (nz, M.track_len, M.best_time, max(M.vmax)))
    print("ALL SMOKE TESTS PASSED")


if __name__ == "__main__":
    main()
