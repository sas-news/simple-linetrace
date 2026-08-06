# ============================================================================
# ハブ向き + 符号 最小テスト  "MEASURE ORIENT"  (Pybricks 用)
# ----------------------------------------------------------------------------
# main.py が「高速に片回転する/蛇行する」原因になる、実装を見極めるための最小ツール。
# 測るもの:
#   1. 静止重力 -> どの IMU 軸が「上(垂直=旋回軸)」か
#   2. スピン -> 旋回時にどの軸が回転し、その符号(回転方向マッピング)
#   3. 前進 -> どの軸が「前方加速度」か、その符号 -> CMD_FLIP
# 出力をコピペしてもらえれば、main.py の Axis と符号を確定できる。
#
# 使い方: Pybricks Code で書き込み実行。数十秒で終わる。
#   ロボットは短いスピン・前進バーストを起こすが、床に置いたまま数秒で止まる。
# ============================================================================

try:
    from micropython import native
    def native(f):
        return f
except ImportError:
    def native(f):
        return f

from pybricks.hubs import PrimeHub
from pybricks.pupdevices import Motor, ColorSensor
from pybricks.parameters import Port, Direction, Button, Axis
from pybricks.tools import StopWatch, wait

PORT_LM = Port.A
PORT_RM = Port.E
PORT_LS = Port.B
PORT_RS = Port.F

hub = PrimeHub()
mL = Motor(PORT_LM, positive_direction=Direction.COUNTERCLOCKWISE)
mR = Motor(PORT_RM, positive_direction=Direction.CLOCKWISE)
csL = ColorSensor(PORT_LS)
csR = ColorSensor(PORT_RS)
sw = StopWatch()


def avg_acc(axis, ms=500):
    s = 0
    n = 0
    t0 = sw.time()
    while sw.time() - t0 < ms:
        s += hub.imu.acceleration(axis)
        n += 1
    return s // max(n, 1)


def avg_omega_mdeg(axis, ms=500):
    s = 0
    n = 0
    t0 = sw.time()
    while sw.time() - t0 < ms:
        s += int(hub.imu.angular_velocity(axis) * 1000)
        n += 1
    return s // max(n, 1)


def wait_center(msg):
    print("-- %s --" % msg)
    from pybricks.parameters import Button
    while not (Button.CENTER in hub.buttons.pressed()):
        wait(20)
    while Button.CENTER in hub.buttons.pressed():
        wait(20)
    wait(200)


def test_static():
    print("==== 1) 静止(水平)素材時の重力方向 ====")
    wait_center("ロボットを平らな床に置き、中央ボタン")
    ax = avg_acc(Axis.X)
    ay = avg_acc(Axis.Y)
    az = avg_acc(Axis.Z)
    print("RESULT gravity: accX=%d accY=%d accZ=%d (mm/s2)" % (ax, ay, az))
    print("  -> 値が ~1000 の軸が「上(垂直=旋回軸)」。~0 の軸は水平。")


def test_spin():
    print("==== 2) スピン(その場旋回)時の回転軸と符号 ====")
    wait_center("中央ボタンで左方向スピン(smL=+120 smR=-120)")
    mL.run(120)
    mR.run(-120)
    w1 = (avg_omega_mdeg(Axis.X), avg_omega_mdeg(Axis.Y), avg_omega_mdeg(Axis.Z))
    mL.run(0)
    mR.run(0)
    wait(200)
    wait_center("中央ボタンで右方向スピン(smL=-120 smR=+120)")
    mL.run(-120)
    mR.run(120)
    w2 = (avg_omega_mdeg(Axis.X), avg_omega_mdeg(Axis.Y), avg_omega_mdeg(Axis.Z))
    mL.run(0)
    mR.run(0)
    print("RESULT spin L(mL=+): X=%d Y=%d Z=%d (mdeg/s)" % w1)
    print("RESULT spin R(mL=-): X=%d Y=%d Z=%d (mdeg/s)" % w2)
    print("  -> |値|最大の軸が旋回軸。符号は物理旋回方向との対応。")


def test_forward():
    print("==== 3) 前進時の加速度方向と符号 ====")
    wait_center("中央ボタンで前方バースト (mL=mR=+120)")
    bx = avg_acc(Axis.X)
    by = avg_acc(Axis.Y)
    bz = avg_acc(Axis.Z)
    mL.run(120)
    mR.run(120)
    fx = avg_acc(Axis.X, 400)
    fy = avg_acc(Axis.Y, 400)
    fz = avg_acc(Axis.Z, 400)
    mL.run(0)
    mR.run(0)
    print("RESULT fwd base: X=%d Y=%d Z=%d" % (bx, by, bz))
    print("RESULT fwd run : X=%d Y=%d Z=%d (delta: %+d %+d %+d)" %
          (fx, fy, fz, fx - bx, fy - by, fz - bz))
    print("  -> 前進で正に増える軸が「前方」。負に増えるなら符号反転。")


def sensor_rate():
    """センサーの真の実効更新レートを動的入力を与えて測る。
    静止面の変化回数では不正確(ノイズ)なので、ロボットをその場でスピンさせ、
    センサーをラインに繰り返し横断させる。値が1秒間に何回変わるかで更新レートを推定。"""
    global mL, mR, csL, csR
    print("==== センサー実効更新レート (動的) ====")
    print("  ロボットをライン上に置き、センサーがラインを横切るようにします")
    wait_center("中央ボタンでスピン計測開始 (約2秒)")
    t0 = sw.time()
    n = 0
    chg = 0
    lastL = csL.reflection()
    lastR = csR.reflection()
    mL.run(150)
    mR.run(-150)
    while sw.time() - t0 < 2000:
        l = csL.reflection()
        r = csR.reflection()
        n += 1
        if l != lastL or r != lastR:
            chg += 1
        lastL = l
        lastR = r
    mL.run(0)
    mR.run(0)
    print("RESULT loop_calls=%d  value_changes=%d/s (L+R)=" % (n, chg))
    print("  -> 変化回数がループ回数に近い=センサー高速。19前後=低速(キャッシュ/更新制限).")
    print("  (静止面の変化回数はノイズになるため、これは動的入力での測定)")


def main():
    print("=== MEASURE ORIENT ===")
    # 中央ボタン単独では停止しないよう、3ボタン同時で停止に設定
    # (デフォルトは中央1個=停止で、テストが進められないため)
    hub.system.set_stop_button((Button.CENTER, Button.LEFT, Button.RIGHT))
    test_static()
    test_spin()
    test_forward()
    sensor_rate()
    print("=== DONE (この RESULT をコピペしてください) ===")


if __name__ == "__main__":
    main()
