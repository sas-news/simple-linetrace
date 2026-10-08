# AGENTS.md — 開発者向け運用規定

このファイルは、本リポジトリ（SPIKE Prime ライントレーサー "LINE RACER SLIM" v2.0）を
変更・運用するすべての開発者（人間・AI エージェント）向けのルールブックです。

---

## 1. プロジェクト概要

- **フィールド**: 白いフィールド上に、ランダムな曲線が一周した黒いライン +
  進行方向に垂直な**単一マーカー線**（スタート/ゴール）1本。
  障害物・ミッションなし。いかに速く正確にラインを周回し続けるかだけを競う。
- **目的**: LEGO SPIKE Prime（45678 + 45681）で、無限周回ライントレースの
  ベストタイムを自動記録・自動改善するロボットを動かす。
- **アーキテクチャ**（v2.0 SLIM）:
  1. 起動時: デバイス接続チェック → ジャイロバイアス測定
     （符号・反射光は実測値に固定。v1.x の起動時自動テスト/円周キャリブは廃止）
  2. マーカー探索: 「両センサ同時暗」の単一横断線を探して通過（s=0 原点を確定）
  3. 各周: 比例 + ジャイロ・ウォッシュアウト減衰で追従。誤差連動の自動減速。
     学習FF (ILC) とレーシング速度計画は実装済みだがフラグで無効化中
     （`ILC_ALPHA = 0`, `RACE_MODE = False`。ベース制御の実機確認後に復帰）
  4. ラップ境界: マーカー再検出（lap≥2 は track_len±60cm の位置窓）+
     オドメトリ強制クローズ（s_cm >= track_len）
- **実行環境**: Pybricks ファームウェア（MicroPython）on STM32F413VG
  （Cortex-M4F 100MHz / 1.5MB Flash / 320KB RAM）
- **実測パラメータの根拠は MEASURE_NOTES.md に記録されている**
  （配線 A/E/B/F、円周 27.6cm、トレッド 16cm、反射 cal、センサー実効 ~19Hz、
  符号セット CMD_FLIP=+1 / SIGN_GYRO_CUR=-1 / GYRO_SIGN=+1 / SIGN_ST=+1）

## 2. ファイル構成

| ファイル | 役割 | 変更時の注意 |
|---|---|---|
| `main.py` | ハブに書き込む唯一のプログラム | ホットパスは整数演算・動的確保禁止（§5） |
| `measure_robot.py` | ハブ向き・符号・センサーレートの実測ツール | 機体変更時に実機で実行し結果を MEASURE_NOTES.md に記録 |
| `MEASURE_NOTES.md` | 実測値・設計判断の記録 | 実機計測のたび追記（手動管理） |
| `analyze_laps.py` | PC側テレメトリ解析・グラフ・レポート | ダンプ形式を変えたら必ず同期 |
| `_selftest.py` | 純ロジックの自己テスト（PC実行） | **main.py のロジック変更時は必ず同期・更新**（§6） |
| `_smoke_test.py` | pybricks モックでの統合スモークテスト（PC実行） | tick/finalize/マーカー検出が実機なしで回ることを検証 |
| `sample_dump.csv` | 解析ツールの動作確認用サンプル | 手動編集しない |
| `README.md` | 導入・ビルド・チューニング手順 | 動作・設定変更時は必ず更新 |

## 3. 検証コマンド（変更後は必ず全部実行）

```powershell
python -m py_compile main.py analyze_laps.py _selftest.py   # 構文チェック
python _selftest.py                                          # ロジック自己テスト
python _smoke_test.py                                        # 統合スモークテスト (pybricks モック)
python analyze_laps.py sample_dump.csv                       # 解析ツール動作確認
```

実機確認はハブ上でしかできない項目（センサー実効レート、モーター実測速度、
ゲインの実効値、符号の実走検証）は、README の実機検証フローに従う。

## 4. 単位・スケール定義（変更厳禁の基準）

| 量 | スケール | 格納 | 備考 |
|---|---|---|---|
| 正規化センサー値 nL/nR | 0..1000 | int | ライン色=低値（calL/calR の実測値で正規化） |
| 制御誤差 e | ±1000 | int | e<0 = ラインが右 → 右に曲がる。制御は1次ローパス `e_fil` 使用 |
| 誤差微分 de / de_f | ±(1000 e/s) | int | KD=0 固定のため制御には未使用。ローパス `de_f=(de_f*5+de)//6` は実装済み |
| ヨーレート wz | mdeg/s | int | 符号は SIGN_GYRO_CUR（実測固定）。
  **読み取りは `int(hub.imu.angular_velocity(Axis.Z) * 1000)`**（Pybricks は deg/s の
  float を返す。float のまま使うと `//` が float を返し `&` は TypeError になる）。
  減衰はローパス `wz_f` とウォッシュアウト基準 `wz_base` の差 `wz_d` を使う |
| ステアリング st | cm/s | int | vL = ve−st, vR = ve+st（st>0 = 左旋回）。±ST_MAX クランプ |
| 学習FF ff | cm/s @ v=100 基準 | i16 | 実行時 `st += ff[s]·v//100`。±FF_MAX クランプ |
| 速度 v_cmd / vmax | cm/s | u8 (≤255) | vmax は速度計画のビン毎上限 |
| ステアリング積算 st_sum | stn=|st|·100/ve の累積 | i16 | vmax 学習用。ラップ毎クリア |
| 誤差積算 err_sum / err_cnt | e 累積 / 個数 | i16 / u8 | ILC 学習用。ラップ毎クリア |

### 重要な換算式

- 弧長: `s_cm = (aL+aR)/2·CMD_FLIP·SMUL//10000` [cm]（円周27.6cm前提で SMUL=767。
  SMUL = 円周·10000/360）
- 速度→モーター: `deg_s = v_cmps·WMUL//100`（円周27.6cm前提で WMUL=1304。
  WMUL = 36000/円周）
- 誤差: `e = (nR−nL)·1000//(nL+nR+1)`
- ILC学習: `ff += e_avg·ILC_ALPHA//(10·v_bin)`（v=100基準正規化）
- 速度計画: `stn=|st|·100/ve` → `κ1000=125·stn` → `v_curv=√(A_LAT_MAX·100000/κ)`,
  `v_kin=481250//κ` の小さい方（内輪≧−V_MIN の幾何制約。トレッド16cm前提）
- バッテリー補償: `comp = clamp(BATT_NOM·1000//vbat, 1000, COMP_MAX)`、
  `ve = v_cmd·comp//1000`

## 5. コーディング規約（MicroPython / Pybricks）

1. **tick（ホットパス）**: `@micropython.native` 化。**ループ内での動的確保禁止**
   （list 生成・文字列結合・float 演算を極力避ける）。bytearray 直アクセス。
2. **finalize_lap**: ネイティブ化済み。ラップ境界の制御停止を短く抑える。
   **finalize 中はモーターを明示停止**（run_laps が呼ぶ前に mL/mR.run(0)）。
3. **テーブル**: `bytearray` + ヘルパー（i16at/i16set/u16at/ba_clear/ba_fill）。
   符号付きアクセスは必ずヘルパーを使い、クランプを忘れない。
   **`ba[:] = ...` のスライス代入は Pybricks の MicroPython で TypeError になる
   （CPython では通ってしまうので PC テストでは検出できない）。クリア/コピーは
   要素ループで行うこと。**
4. **グローバル**: 関数内で代入する変数は必ず `global` 宣言（代入忘れは
   ローカル変数の罠になる）。
5. **符号系**: `CMD_FLIP` / `SIGN_GYRO_CUR` / `SIGN_ST` / `GYRO_SIGN` は
   **実測値に固定**（自動テストはしない）。⚠ CMD_FLIP を反転したら
   **GYRO_SIGN も一緒に反転**しないとジャイロ減衰が正帰還して暴走する
   （実機で確認済み。MEASURE_NOTES.md §5）。
6. **print**: ホットパス外（ラップ境界・hk の間引き内）のみ。
   ダンプは `hk()` 経由のトークンバケット（最大10行/10ms）。
   **hk() 内で last_hk を更新しているので、呼び出し側で再代入しないこと。**
7. **メモリバジェット**（320KB RAM 内）:
   - ラップログ: 48KB（log_s 16KB + log_e 8KB + log_v 8KB + log_w 16KB、
     LOG_N=8192 × LOG_MS=25ms → 204秒分）
   - 学習テーブル: ~16.5KB（ff/err_sum/vsum_bin/st_sum 各3KB i16 +
     err_cnt/vmax/vmax_scr 各1.5KB u8、N_BINS=1500）
   - ヒープ: 残り
   - **追加テーブルは事前にバジェット表を更新すること**
8. **finalize_lap の順序**:
   無効ラップ判定 → track_len 学習(lap1) → ff_armed 判定 → vmax 更新 →
   マーカー低速ゾーン → vmax 平滑化 → ILC 更新・テーブルクリア →
   ベスト判定 → ダンプ開始
9. **ラップ境界の不変条件**: `reset_lap_state` で `s_base_deg` を再基準化して
   s_cm はラップ毎に 0 から始まる。`wz_base`（ジャイロ基準）は**リセットしない**
   （旋回中のラップ境界で wz_d が巨大化して暴れるため。実機確認済み）

## 6. テスト規約

- `_selftest.py` は main.py の純関数（マーカーマッチャー・正規化・誤差極性・
  bytearray ヘルパー・ILC更新・race_wheels・vmax_from_stn・補間式）と
  **同一ロジックを保つ**こと。main.py を変更したら必ず _selftest.py を更新し、
  全テストを通す。
- テストは「合格条件」だけでなく「誤検出排除」も必ず含める（マッチャーの
  幅不足・幅超過テスト等）。
- `_smoke_test.py` は main.py の tick/finalize_lap を pybricks モックで実際に回す。
  実行時クラッシュ（未定義名・bytearray 越境・global 漏れ）を検出するため、
  main.py の制御構造（global 宣言・テーブルアクセス）を変えたら必ず実行する。
  モックは高速レーシングでラインを保持できないため RACE_MODE=False で
  対称制御のみ検証。純関数は _selftest で単独検証する。

## 7. 実機検証フロー（初回セットアップ）

1. Pybricks Installer でファームウェア導入 → Pybricks Code で main.py 書き込み
2. 起動ログを確認:
   - 4ポートすべて `OK`（NOT CONNECTED が出たら配線修正。実配線は A/E/B/F）
   - `calib: bias=` が ±5000 未満（張り付きは静止不足 → やり直し）
3. 機体をライン上・マーカー直前に置き、中央ボタンでスタート
4. マーカー通過 → `track len (lap1) = N cm` が妥当値か確認
   （実際の2倍前後なら1周目に読み飛ばしている → マーカー直前から再スタート）
5. 2周目以降 `LAP n done: t=...` が記録され、ベスト更新でブザー2音
6. 発振・暴走時は KP/GYRO と符号セットを見直す
   （README §7「発振するとき」参照。GYRO_SIGN は CMD_FLIP と連動）

## 8. 既知の制約・未確定事項

- **カラーセンサーの実効更新レートは ~19Hz**（実測済み）。これが最も重要な制約。
  微分項はほぼ意味を持たず KD=0 固定が妥当。高速時のマーカー読み飛ばし余裕も
  このレートで決まる（V_MARK で減速する設計）
- ジャイロはこの機体でバイアス不安定（±22000 まで変動の実例あり）。
  ウォッシュアウト減衰で対処済みだが、悪化するなら GYRO=0 で無効化も選択肢
- センサーブームの前方距離 L が大きいと D 項がブーム振れを正帰還するため
  KD=0 固定（L≦5cm に短縮した場合のみ KD を戻してよい）
- 符号系・反射光・ホイール円周は実測値固定。機体変更時は measure_robot.py で
  再計測し MEASURE_NOTES.md に記録してから main.py を更新する
- トレッド幅 TRACK_B_CM は16.0cm（20ポッチ実測）で固定。変更時は
  vmax_from_stn の v_kin 定数（481250）も更新すること
- Pybricks のファイル保存は使わない（RAM 内でラップ間データを引き継ぐ設計）
- ILC_ALPHA=0 / RACE_MODE=False はベース隔離のための仮設定。
  ベース制御の実機確認後に ILC_ALPHA=20 / RACE_MODE=True へ戻す
  （main.py のコメント参照）

## 9. 変更履歴の管理

- 機能追加・バグ修正は README の該当セクションと AGENTS.md §4（スケール表）を
  更新してから完了とする。
- 実機パラメータのチューニング結果（最適 KP/GYRO/V_CRUISE 等）は
  MEASURE_NOTES.md に記録し、README・デフォルト値に反映する。
