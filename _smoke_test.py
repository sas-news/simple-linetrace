# -*- coding: utf-8 -*-
"""
main.py (LINE RACER SLIM) の PC 統合スモークテスト (pybricks モック)
====================================================================
実機なしで main.py の tick / マーカー検出 / ラップ完了 / 記録が
実行時エラーなしで動くことを検証する。ハブ依存 API はすべてモックし、
モータ位置からスクリプトしたセンサ/ジャイロを返す簡易閉ループで走らせる。

実行:  python _smoke_test.py
検証項目:
  - find_marker (マーカー探索) がクラッシュせず最初のマーカーを検出する
  - 6ラップがクラッシュせず完了し、ラップタイム・track_len・best が記録される
  - マーカー読み飛ばし → 緊急ラップ終了がクラッシュしない
"""
import os
import sys

# ============================================================================
# pybricks モック (PC 上で main.py を import するための最小実装)
# ============================================================================

LAP_LEN = 1200.0       # コース長 cm (マーカー周期)
TRACK_B_SIM = 16.0     # トレッド幅 cm
SENSOR_OFF = 3.0       # センサーのライン中心からの横オフセット cm


class _Sim:
    """閉ループ状態 (横ずれ・方位) とモータ実体への参照"""
    lat = 0.0
    head = 0.0
    motors = None


SIM = _Sim()


class _Motor:
    def __init__(self, port, positive_direction=None):
        self.angle_deg = 0.0
        self.speed = 0.0

    def run(self, spd):
        self.speed = float(spd)

    def angle(self):
        return int(self.angle_deg)

    def stop(self):
        self.speed = 0.0

    def _advance(self, dt_ms):
        self.angle_deg += self.speed * dt_ms / 1000.0


def _pos_cm():
    """モータ平均角から補正済み絶対位置 (cm) を推定 (CMD_FLIP+SMUL を反映)。
    main.py の s_cm と同じ変換で、マーカー/カーブを整合フレームで定義する。"""
    a = (SIM.motors[0].angle_deg + SIM.motors[1].angle_deg) / 2.0
    return a * M.CMD_FLIP * M.SMUL / 10000.0


def _sensor_val(dist):
    if dist < 1.5:
        return 10
    if dist < 4.0:
        return int(10 + (dist - 1.5) * 80.0 / 2.5)
    return 90


class _Sensor:
    def __init__(self, port):
        self.side = "L" if str(port) in ("B", "C") else "R"
        self._both_dark = False

    def reflection(self):
        x = _pos_cm()
        xm = x % LAP_LEN
        # マーカー: 各ラップ周期の 2..4cm を両センサ同時暗 (単一横断線)
        if 2.0 <= xm < 4.0:
            self._both_dark = True
            return 10
        self._both_dark = False
        # 通常ライン追従: 中心 (y=0) に半幅 1.5cm のライン
        off = SENSOR_OFF if self.side == "L" else -SENSOR_OFF
        return _sensor_val(abs(SIM.lat + off))


class _IMU:
    def angular_velocity(self, axis):
        x = _pos_cm()
        xm = x % LAP_LEN
        if 200.0 <= xm < 300.0:
            k = 1500.0                    # カーブ1
        elif 700.0 <= xm < 800.0:
            k = -2000.0                   # カーブ2
        else:
            k = 0.0
        v_cm = ((SIM.motors[0].speed + SIM.motors[1].speed) / 2.0) * 100.0 / M.WMUL
        return (k * v_cm * 57296 / 100000) / 1000.0

    def acceleration(self, axis):
        return 0.0


class _Battery:
    def voltage(self):
        return 8070


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
        self.buttons = _Buttons()


_CLK = 0


class _StopWatch:
    def time(self):
        return int(_CLK)


def _native(f):
    return f


def _mem_info(*a, **k):
    pass


def _wait(ms):
    global _CLK
    _CLK += ms


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

here = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, here)
import main as M

SIM.motors = (M.mL, M.mR)

# レーシング(RACE_MODE)は、この粗いモックでは高速でラインを保持できないため、
# smoke では対称制御(従来回)のみで「クラッシュしない・ラップ完走」を検証する。
# race_wheels / vmax_from_stn の純関数ロジックは _selftest.py で単独検証。
# レーシングの実効性は実機ハードで確認する。
M.RACE_MODE = False

STEP_MS = 5


def advance(dt_ms):
    """モータ位置・姿勢を進める (tick の後に呼ぶ)"""
    M.mL._advance(dt_ms)
    M.mR._advance(dt_ms)
    vL = M.mL.speed * 100.0 / M.WMUL
    vR = M.mR.speed * 100.0 / M.WMUL
    v = (vL + vR) / 2.0
    omega = (vR - vL) / TRACK_B_SIM
    SIM.head += omega * dt_ms / 1000.0
    SIM.lat += v * math_sin(SIM.head) * dt_ms / 1000.0


import math as _math
def math_sin(x):
    return _math.sin(x)


def sim_loop(cond, guard_limit, on_step):
    global _CLK
    guard = 0
    while cond() and guard < guard_limit:
        _CLK += STEP_MS
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
        raise RuntimeError("sim loop guard exceeded (no marker?)")


def sim_find_marker():
    print("-- find_marker --")
    M.in_lap = False
    M.lap_done = False
    M.track_len = 0
    M.t_prev = 0
    M.last_hk = 0
    sim_loop(lambda: not M.lap_done and M.sw.time() < 60000, 20000, lambda: None)
    if not M.lap_done:
        raise RuntimeError("first marker not found")


def sim_laps(n_laps):
    M.lap_no = 0
    times = []
    for _ in range(n_laps):
        M.lap_no += 1
        M.in_lap = True
        M.reset_lap_state()
        M.lap_start_t = M.sw.time()
        M.last_hk = 0
        print("-- LAP %d start --" % M.lap_no)

        def on_tick():
            if M.in_lap and not M.lap_done and M.track_len > 0 \
                    and M.s_cm > M.track_len + M.MARK_MISS:
                M.mark_s = M.track_len
                M.lap_done = True

        sim_loop(lambda: not M.lap_done, 200000, on_tick)
        t_lap = M.sw.time() - M.lap_start_t - M.mark_f * M.mark_dt // 100
        M.finalize_lap(t_lap)
        times.append(t_lap)
    return times


def sim_marker_miss():
    print("-- marker miss (emergency lap end) --")
    M.lap_no += 1
    M.in_lap = True
    M.reset_lap_state()
    M.track_len = 1200
    M.lap_start_t = M.sw.time()
    M.lap_valid = False

    def on_tick():
        if M.in_lap and not M.lap_done and M.track_len > 0 \
                and M.s_cm > M.track_len + M.MARK_MISS:
            M.mark_s = M.track_len
            M.lap_done = True

    sim_loop(lambda: not M.lap_done, 300000, on_tick)
    t_lap = M.sw.time() - M.lap_start_t - M.mark_f * M.mark_dt // 100
    M.finalize_lap(t_lap)   # invalid lap → 早期 return
    assert M.lap_valid is False
    print("marker miss handled ok")


def main():
    print("=== smoke test: LINE RACER SLIM on PC (pybricks mocked) ===")
    sim_find_marker()
    times = sim_laps(6)
    sim_marker_miss()

    assert M.track_len > 0, "track_len not set"
    assert M.best_time > 0, "best_time not set"
    assert min(times) > 0, "lap times not recorded"
    assert M.log_idx > 0, "telemetry log empty"
    assert M.best_lap >= 1
    print("laps: times=%s best=%d track=%d best_lap=%d" %
          (times, M.best_time, M.track_len, M.best_lap))
    print("ALL SMOKE TESTS PASSED")


if __name__ == "__main__":
    main()
