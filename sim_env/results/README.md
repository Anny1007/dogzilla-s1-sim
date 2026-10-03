# 測試結果紀錄

開發過程中記錄下來的測試結果，`sim_env/README.md` 引用的數字都出自這裡。新跑的結果會輸出到 `sim_env/logs/`，不會覆蓋這裡的檔案。

## benchmarks/ — 避障 benchmark

由 `benchmark_nav.py` 產生：同一場地依序走 7 個目標點，用 Gazebo 真值評分。

| 檔案 | 機器狗模型 | 場地 | 階段 |
|---|---|---|---|
| `bench_s1off_dwb.csv` / `_rpp` / `_rl` | **官方 S1 URDF**（`model:=s1`） | project3 | **最新**（2026-10-02，含運動安全濾波） |
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

由 `tools/gain_sweep.sh`、`tools/gait_sweep.sh` 產生，每行一組參數，`SUMMARY` 的欄位見 `tools/gait_eval.py`。

## training/ — RL 訓練紀錄

| 檔案 | 內容 |
|---|---|
| `train_s1_official_4M.log` | **目前使用的策略**：官方 S1 模型，4M 步；檔尾是固定場地的離線評估（92% 到達） |
| `train_s1_official.log` | 官方 S1 模型，2M 步（離線評估 42%，權重在 `rl/models/s1_official_2M/`） |
| `train_s1.log` | 手工方塊模型版（權重在 `rl/models/s1_box/`） |
