# -*- coding: utf-8 -*-
"""
ライントレーサーテレメトリ解析ツール
=====================================
SPIKE ハブの USB コンソールにダンプされたラップログを解析する。

使い方:
  1. ハブの左ボタンを押してダンプモードを有効化 (周回終了時に自動ダンプ)
  2. Pybricks Code / ターミナルで USB コンソール出力を dump.csv に保存
  3. 解析:  python analyze_laps.py dump.csv
     グラフ: python analyze_laps.py dump.csv --plot
     レポート: python analyze_laps.py dump.csv --report report.txt

出力:
  - ラップタイム表 (ベストタイム・各区間統計)
  - 軌跡再構成 (オドメトリ + ジャイロ)
  - 曲率プロファイル・セグメント分類
  - 推奨パラメータ (A_LAT / 速度上限 / 減速距離)

ログ形式 (ハブ側出力):
  #LAP <n> time_ms <t> best_ms <b> dist_cm <d> rms_e ...   (メタ)
  #CFG ...                                                  (メタ)
  #DATA s_cm,e,v_cmps,w_mdeg_s                              (ヘッダ)
  s,e,v,w                                                   (データ行)
"""

import argparse
import math
import sys

# Windows コンソール (cp932) での日本語文字化け対策
try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    sys.stderr.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    pass

try:
    import matplotlib.pyplot as plt
    HAS_MPL = True
except ImportError:
    HAS_MPL = False


# ============================================================================
# パーサ
# ============================================================================

def parse_dump(path):
    """ダンプファイルを読み込み (meta, laps, rows) を返す"""
    meta = {}
    laps = []
    rows = []
    in_data = False
    with open(path, "r", encoding="utf-8", errors="replace") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            if line.startswith("#LAP"):
                parts = line.split()
                d = {}
                for i in range(1, len(parts) - 1, 2):
                    try:
                        d[parts[i]] = int(parts[i + 1])
                    except (ValueError, IndexError):
                        pass
                laps.append(d)
            elif line.startswith("#CFG"):
                parts = line.split()
                for i in range(1, len(parts) - 1, 2):
                    try:
                        meta[parts[i]] = int(parts[i + 1])
                    except (ValueError, IndexError):
                        pass
            elif line.startswith("#DATA"):
                in_data = True
            elif line.startswith("#END"):
                in_data = False
            elif line.startswith("#"):
                continue
            elif in_data:
                try:
                    s, e, v, w = (int(x) for x in line.split(","))
                except ValueError:
                    continue
                rows.append((s, e, v, w))
    return meta, laps, rows


# ============================================================================
# 解析
# ============================================================================

def analyze(meta, laps, rows):
    """解析を行い、結果ディクショナリを返す"""
    res = {}

    # ---- ラップタイム表 ----
    res["laps"] = laps
    if laps:
        res["best_lap"] = min(laps, key=lambda d: d.get("time_ms", 1 << 60))
    else:
        res["best_lap"] = None

    # ---- 軌跡再構成 (s, v, ω から x, y, θ) ----
    if len(rows) >= 3:
        xs, ys = [0.0], [0.0]
        s_prev = rows[0][0]
        th = 0.0
        for i in range(1, len(rows)):
            s, e, v, w = rows[i]
            ds = s - s_prev
            if v > 1:
                dt = ds / v          # s[cm]/v[cm/s] → s
                th += math.radians(w / 1000.0) * dt
                xs.append(xs[-1] + math.cos(th) * ds)
                ys.append(ys[-1] + math.sin(th) * ds)
            else:
                xs.append(xs[-1])
                ys.append(ys[-1])
            s_prev = s
        res["path"] = (xs, ys)

        # 曲率 κ(s) = ω/v (平滑化)
        kappas = []
        for i in range(len(rows)):
            s, e, v, w = rows[i]
            k = w / 1000.0 * math.pi / 180.0 / max(v / 100.0, 0.05)  # 1/m
            kappas.append(k)
        kappas = smooth(kappas, 9)
        res["kappas"] = kappas
        res["s_list"] = [r[0] for r in rows]
        res["v_list"] = [r[2] for r in rows]
        res["e_list"] = [r[1] for r in rows]
        res["w_list"] = [r[3] for r in rows]

        # ---- 横加速度の実測最大 ----
        a_max = 0.0
        for i in range(len(rows)):
            s, e, v, w = rows[i]
            a = abs(kappas[i]) * (v / 100.0) ** 2  # m/s^2
            if a > a_max:
                a_max = a
        res["a_lat_observed"] = a_max * 100.0  # cm/s^2

        # ---- 誤差統計 ----
        # ダンプの e はハブ側で i8 量子化 (±127)。制御スケール(±1000)に換算して
        # ハブの lap_stats() が出力する rms_e/max_e と同一スケールにする。
        es = [abs(r[1]) for r in rows]
        res["rms_e"] = math.sqrt(sum(x * x for x in es) / len(es)) * 1000 / 127
        res["max_e"] = max(es) * 1000 / 127

        # ---- セグメント分類 ----
        res["segments"] = classify_segments(res["s_list"], kappas)
    return res


def smooth(vals, n):
    """移動平均 (端は縮小ウィンドウ)"""
    out = list(vals)
    half = n // 2
    for i in range(len(vals)):
        lo = max(0, i - half)
        hi = min(len(vals), i + half + 1)
        out[i] = sum(vals[lo:hi]) / (hi - lo)
    return out


def classify_segments(s_list, kappas):
    """κ からセグメントを分類: (start_cm, end_cm, type, |κ|max)"""
    segs = []
    cur = None
    for i in range(len(kappas)):
        k = abs(kappas[i])
        if k < 0.6:
            t = "straight"
        elif k < 2.5:
            t = "curve"
        else:
            t = "hairpin"
        if cur is None or cur[2] != t:
            if cur is not None:
                cur[1] = s_list[i]
            cur = [s_list[i], s_list[i], t, 0.0]
            segs.append(cur)
        if k > cur[3]:
            cur[3] = k
    if cur is not None:
        cur[1] = s_list[-1]
    return segs


# ============================================================================
# レポート
# ============================================================================

def make_report(res):
    out = []
    out.append("==== LINE RACER 解析レポート ====")
    laps = res.get("laps", [])
    if laps:
        out.append("ラップタイム:")
        for d in laps:
            mark = " *" if d is res.get("best_lap") else ""
            out.append("  LAP %d: %.3fs  (dist=%dcm, rms_e=%d, max_e=%d, vavg=%dcm/s)%s" %
                       (d.get("lap_no", 0), d.get("time_ms", 0) / 1000.0,
                        d.get("dist_cm", 0), d.get("rms_e", 0), d.get("max_e", 0),
                        d.get("vavg", 0), mark))
    if "a_lat_observed" in res:
        out.append("実測最大横加速度: %.0f cm/s^2" % res["a_lat_observed"])
        out.append("  → A_LAT 推奨: %.0f 〜 %.0f cm/s^2" %
                   (res["a_lat_observed"] * 0.8, res["a_lat_observed"] * 0.95))
    if "max_e" in res:
        out.append("誤差: RMS=%.0f MAX=%.0f (e: ±1000)" % (res["rms_e"], res["max_e"]))
    segs = res.get("segments", [])
    if segs:
        out.append("セグメント (%d個):" % len(segs))
        for st, en, t, kmax in segs:
            out.append("  %5d-%5dcm  %-8s  |κ|max=%.2f /m" % (st, en, t, kmax))
    if "s_list" in res:
        v = res["v_list"]
        out.append("速度: 平均 %.0f cm/s, 最大 %d cm/s" %
                   (sum(v) / len(v), max(v)))
    return "\n".join(out)


# ============================================================================
# プロット
# ============================================================================

def plot(res, path):
    if not HAS_MPL:
        print("matplotlib が無いためグラフを省略します")
        return
    fig, axes = plt.subplots(2, 2, figsize=(12, 8))
    if "path" in res:
        xs, ys = res["path"]
        axes[0][0].plot(xs, ys, lw=1.2)
        axes[0][0].set_title("path (x,y)")
        axes[0][0].set_aspect("equal")
        axes[0][0].grid(alpha=0.3)
    if "s_list" in res:
        s = res["s_list"]
        axes[0][1].plot(s, res["kappas"])
        axes[0][1].set_title("curvature kappa(s) [1/m]")
        axes[0][1].grid(alpha=0.3)
        axes[1][0].plot(s, res["v_list"])
        axes[1][0].set_title("speed v(s) [cm/s]")
        axes[1][0].grid(alpha=0.3)
        axes[1][1].plot(s, res["e_list"])
        axes[1][1].set_title("error e(s)")
        axes[1][1].grid(alpha=0.3)
    fig.tight_layout()
    out = path.rsplit(".", 1)[0] + "_plot.png" if "." in path else path + "_plot.png"
    fig.savefig(out, dpi=150)
    print("saved:", out)


# ============================================================================
# main
# ============================================================================

def main():
    ap = argparse.ArgumentParser(description="ラインラップログ解析")
    ap.add_argument("dump", help="ダンプCSVファイル")
    ap.add_argument("--plot", action="store_true", help="グラフ出力")
    ap.add_argument("--report", metavar="OUT", help="レポート保存先")
    args = ap.parse_args()

    meta, laps, rows = parse_dump(args.dump)
    print("rows=%d laps=%d" % (len(rows), len(laps)))
    if not rows and not laps:
        print("データがありません。左ボタンでダンプモードを有効にしてください。")
        sys.exit(1)

    res = analyze(meta, laps, rows)
    report = make_report(res)
    print(report)

    if args.report:
        with open(args.report, "w", encoding="utf-8") as f:
            f.write(report + "\n")
        print("report saved:", args.report)

    if args.plot:
        plot(res, args.dump)


if __name__ == "__main__":
    main()
