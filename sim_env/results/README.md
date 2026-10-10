# 測試結果紀錄

開發過程中記錄下來的測試結果，`sim_env/README.md` 引用的數字都出自這裡。新跑的結果會輸出到 `sim_env/logs/`，不會覆蓋這裡的檔案。

## benchmarks/ — 避障 benchmark

由 `benchmark_nav.py` 產生：同一場地依序走 7 個目標點，用 Gazebo 真值評分。

| 檔案 | 機器狗模型 | 場地 | 階段 |
|---|---|---|---|
| `bench_s1_875g_rl_v6.csv` / `_rl_v5` / `_rpp` / `_dwb` | **官方 S1 URDF，實機重量**（本體 875 g + LiDAR 45 g，LiDAR 在 x=−0.06） | project3 | **最新**（2026-10-10）。`rl_v6` 是預設權重（`_r2` 是第二次執行，`path_s1_875g_rl_v6_r2.csv` 是該次的真值路徑），`rl_v5` 是 625 g 時訓練的舊權重 |
| `bench_s1off_dwb.csv` / `_rpp` / `_rl` | 官方 S1 URDF，官方質量 625 g | project3 | 2026-10-02，含運動安全濾波 |
| `bench_s1v2_dwb.csv` / `_rpp` / `_rl` | 手工方塊模型（`model:=s1_box`） | obstacle_test | 調整後（2026-09-24） |
| `bench_s1_dwb.csv` / `_rpp` / `_rl` | 手工方塊模型 | obstacle_test | 第一輪（2026-09-24） |
| `bench_dwb_v2.csv` / `bench_rpp_v2.csv` / `bench_rl_v3.csv` | 舊的 CHAMP 放大模型（`model:=champ`） | obstacle_test | 修正 Gazebo 崩潰、里程計、地面點之後（2026-09-23） |
| `bench_dwb.csv` / `bench_rpp.csv` / `bench_rl.csv` / `bench_rl_v2.csv` | 舊的 CHAMP 放大模型 | obstacle_test | 修正前（2026-09-21～22） |

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
`speed_response_s1_875g_run1.txt`、`_run2` 是目前的 920 g 模型（RL v6 的速度模型依此擬合）。
`startstop_test_875g.txt` 是 `tools/startstop_test.py` 的輸出：前進指令在 0 與 0.15 / 0.20 m/s 間來回時的最大傾斜（各 2 次啟動）。

## training/ — RL 訓練紀錄

| 檔案 | 內容 |
|---|---|
| `train_s1_v6_finetune_2M.log` | **目前使用的策略（v6）**：以 v5 權重為起點，在 920 g 模型的實測速度模型上接續訓練 2M 步（權重在 `rl/models/s1_v6_finetune/best/`） |
| `train_s1_v6_finetune_creep_2M.log` | v6 的對照版：速度模型多了「原地轉會後退」（權重在 `rl/models/s1_v6_finetune_creep/`，沒有在 Gazebo 實測） |
| `train_s1_v6_scratch_nocreep_4M.log` / `_creep_4M.log` | 920 g 速度模型從零訓練 4M 步（上限 0.15 m/s）：比 v5 差，沒有採用（權重在 `rl/models/s1_v6_scratch/`） |
| `eval_compare_v6.txt` / `eval_compare_v6_scratch.txt` | 2D 環境比較（都用 920 g 的速度模型）：微調版、從零版、v5 舊權重、追路徑點基準 |
| `flip_v6_vmax020.txt` | 速度上限 0.20 m/s 的版本在 Gazebo 翻倒前 6 秒的紀錄（因此上限維持 0.15 m/s；該版權重沒有保留） |
| `ppo_metrics_v6.png` | v6 微調的訓練曲線（loss、回報、到達率） |
| `train_s1_official_4M.log` | 625 g 時期的策略：官方 S1 模型，4M 步；檔尾是固定場地的離線評估（92% 到達）。權重在 `rl/models/s1_logged/` |
| `train_s1_official.log` | 官方 S1 模型，2M 步（離線評估 42%，權重在 `rl/models/s1_official_2M/`） |
| `train_s1.log` | 手工方塊模型版（權重在 `rl/models/s1_box/`） |
