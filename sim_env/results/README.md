# 測試結果紀錄

開發過程中記錄下來的測試結果，`sim_env/README.md` 引用的數字都出自這裡。新跑的結果會輸出到 `sim_env/logs/`，不會覆蓋這裡的檔案。

## benchmarks/ — 避障 benchmark

由 `benchmark_nav.py` 產生：同一場地依序走完所有目標點，用 Gazebo 真值評分。強化學習的結果是用只訓練 10 萬步的權重跑的。

| 檔案 | 機器狗模型 | 場地 | 階段 |
|---|---|---|---|
| `bench_corridor_rl_baseline.csv` / `bench_clutter_rl_baseline.csv` | 官方 S1 URDF，實機重量（本體 875 g + LiDAR 45 g，LiDAR 在 x=−0.06） | 場景 2 corridor / 場景 4 clutter | RL 目前的權重（訓練 10 萬步） |
| `path_corridor_rl_baseline.csv` / `path_clutter_rl_baseline.csv` | 同上 | 場景 2 corridor / 場景 4 clutter | 上面兩次執行的真值路徑（`tools/path_record.py`；`tools/plot_scene.py` 可畫出來） |
| `bench_corridor_rpp.csv` | 官方 S1 URDF，實機重量（本體 875 g + LiDAR 45 g，LiDAR 在 x=−0.06） | 場景 2 corridor | Nav2 RPP |
| `bench_s1_875g_rpp.csv` / `_dwb` | 同上 | 場景 1 project3 | Nav2 RPP / DWB |

較早期（625 g 模型、方塊模型、放大模型）的 benchmark 與所有舊的 RL 權重、訓練紀錄已於 2026-10-10 移除。

欄位：

| 欄位 | 意義 |
|---|---|
| `status` | 控制器回報的結果（`SUCCEEDED` / `ABORTED` / `TIMEOUT`；每段上限 150 s） |
| `success` | 真值到達（離目標 < 0.35 m）為 1 |
| `time_s` / `path_m` | 用時、實際走的路徑長 |
| `final_err_m` | 結束時與目標的真實距離 |
| `min_clearance_m` | 機器狗中心離障礙物表面的最小距離 |
| `collisions` | 中心離障礙物 < 0.12 m 的次數（擦碰） |
| `slam_err_m` | 該段結束時 SLAM 定位與真值的差距（較舊的檔案沒有這欄） |
| `flipped` | 是否已翻倒（較舊的檔案沒有這欄） |

## sweeps/ — 參數掃描

| 檔案 | 內容 |
|---|---|
| `gain_sweep_pid.txt` | 官方 S1 模型的關節 PID／關節阻尼／控制頻率掃描（找出晃動原因：PID 顫振） |
| `gain_sweep.txt` | 官方 S1 模型固定 PID 後的步態掃描（抬腳高度、支撐相時間），選出目前的設定 |
| `gait_sweep.txt` | 手工方塊模型的步態掃描（站高、抬腳、支撐相、PID） |
| `lidar_sweep_before_retune.txt` | 質量改成本體 875 g + LiDAR 45 g、**還沒重調增益**（p=8）時的 LiDAR 位置掃描：各位置都走不穩，看不出差別 |
| `tune_875g_round1.txt` | 920 g 下的關節增益 / 站高 / 質心平移 / 支撐相掃描（選出 p=14）；`M=0.6115,LM=0.0141` 是舊質量對照 |
| `tune_875g_round2_lidar_x.txt` | p=14 下 **LiDAR 前後位置 6 個點**（`LX`，m）與不裝 LiDAR（`LM=0.001`）的比較，以及增益細調 |
| `tune_875g_round3_lidar_x_repeat.txt` | LiDAR 位置 −40 ~ −70 mm 的重複量測（每個位置重啟 2 次、各量 2 次），選出 −60 mm |

由 `tools/gain_sweep.sh`、`tools/gait_sweep.sh` 產生，每行一組參數，`SUMMARY` 的欄位見 `tools/gait_eval.py`。
`tune_875g_*` 與 `lidar_sweep_*` 由 `tools/tune_sweep.sh` 產生：每組參數兩行 `SUMMARY`（同一次啟動連續量兩次）加一行四條腿膝關節的扭力飽和時間比例（lf/rf/lh/rh）。

## speed_response_*.txt、startstop_test_875g.txt — 速度反應與起停測試

`tools/speed_response.py` 的輸出：各種速度指令下的實際前進 / 轉向速度與最大傾斜。`speed_response_s1.txt`、`_run2` 是 625 g 時期，
`speed_response_s1_875g_run1.txt`、`_run2` 是目前的 920 g 模型（RL 訓練環境的速度模型依此擬合）。
`startstop_test_875g.txt` 是 `tools/startstop_test.py` 的輸出：前進指令在 0 與 0.15 / 0.20 m/s 間來回時的最大傾斜（各 2 次啟動）。

## training/ — RL 訓練紀錄

| 檔案 | 內容 |
|---|---|
| `baseline_100k_eval.csv` | 目前的權重訓練時的評估（10 萬步，200 個固定場景到達 63 個） |
| `baseline_eval_2d.txt` | 目前的權重與「追路徑點」傳統基準在 2D 環境各場地的到達率（`rl/eval_compare.py` 的輸出） |
