# DOGZILLA S1 自主避障模擬環境

對應 `docs/project.pptx`（Project 3：Autonomous Obstacle Avoidance for a Quadruped Robot）的目標：

- 機器狗用新增的 **LiDAR** 偵測障礙物
- **自己走到目標點**，途中**繞過障礙物**

模擬裡實作了投影片 Software Pipeline 的每一層：

```
Environment ─► LiDAR Sensing (/scan, 360 線 3.5 m)
                   │
                   ▼
               地面點過濾 (依 IMU 傾角, /scan_filtered)
                   │
                   ▼
               SLAM Toolbox ─► /map + map→odom 位姿
                   │
                   ▼
               Nav2 Path Planning (全域路徑)
                   │
     ┌─────────────┴──────────────┐
     ▼                            ▼
 Nav2 局部控制器 (DWB / RPP)    Reinforcement Learning 避障 (PPO)
     └─────────────┬──────────────┘
                   ▼  /cmd_vel_raw  (ROS 2 Topics)
               運動安全濾波 (轉彎降速、限制轉向變化率)
                   ▼  /cmd_vel
               CHAMP 步態控制 (Motion Control) ─► 12 顆關節 (Gazebo 裡的 DOGZILLA)
```

機器人模型預設是 **Yahboom 官方的 DOGZILLA S1 URDF**（見下方「DOGZILLA S1 模型」）。

實機上 `/cmd_vel` 由 `vendor/DOGZILLA_README/Dogzilla_ros2_ws-main` 的 `driver_node` 接手（透過 `DOGZILLALib` 串口控制），
所以模擬裡驗證過的 SLAM / Nav2 / RL 節點可以原封不動搬到 Raspberry Pi 上。

## 環境需求

- Ubuntu + Docker（含 `docker compose`），映像基於 `osrf/ros:humble-desktop-full`
- 有桌面 (X11) 才能看 Gazebo / RViz 畫面；沒有也可以用無頭模式跑
- 第一次在新電腦上建置：照 [`SETUP.md`](SETUP.md) 一步步做（含驗證方法、強化學習訓練環境與常見問題）

## 快速開始

```bash
cd sim_env
./run.sh build        # 第一次: 建 Docker 映像 (dogzilla-sim:humble, 約 10~20 分鐘, 需網路)
./run.sh colcon       # 第一次: 編譯 vendor/Program/yahboomcar_ws_ros2 裡的 CHAMP 套件 (約 3 分鐘)

./run.sh sim          # 啟動: Gazebo + SLAM + Nav2(DWB) + RViz
./run.sh sim rl       # 改用 RL 當局部避障控制器
./run.sh sim rpp      # 改用 Nav2 Regulated Pure Pursuit
./run.sh stop         # 停止

./run.sh sim dwb model:=champ   # 改用舊的 CHAMP/XGO 放大模型 (約 S1 的 2~3 倍大, 僅供對照)
```

啟動約 30 秒後（SLAM 12 秒後開、Nav2 18 秒後開），在 RViz 上方工具列點 **2D Goal Pose**，
在地圖上點一下目標位置，機器狗就會自己規劃路徑、繞開障礙物走過去。三種控制器模式都用同一個按鈕。

其他常用指令：

```bash
./run.sh shell                        # 進到正在跑的模擬容器
./run.sh sim dwb gui:=false rviz:=false   # 無頭模式
./run.sh bench my_test dwb            # 自動跑 7 個目標點, 結果存 logs/bench_my_test.csv
./run.sh bench my_rl  rl              # (模擬需以 ./run.sh sim rl 啟動)

# 舊場地: 啟動與 benchmark 都要指定
./run.sh sim dwb world:=/worlds/obstacle_test.world
./run.sh bench old_dwb dwb obstacle_test
```

## 場地

預設場地 `worlds/project3.world` 照 `project.pptx` 第 1 頁的 Simulated obstacle field 與 Concept diagram 建立：

```
 y=2  +----------------------------+----+---------------+
      |                     [B]    | W  |           *   |
      |        +---+               | W  |               |
 y=0  |  S     | L |               +----+ (O)           |
      |        +---+                                    |
      |                   (C)                  (C)      |
 y=-2 +-------------------------------------------------+
    x=-0.6                                            x=6.4

 S 起點 (0,0)   L 長方塊   B 小方塊   W 從北牆伸下的隔牆
 O 橘色圓柱     C 灰色圓柱  * 目標 (5.9, 1.1)
```

- 7 m × 4 m 圍牆場地，機器狗在原點朝 +x 生成，正前方被長方塊擋住
- 投影片的任務：從起點繞過長方塊、從隔牆與橘色圓柱下方穿過，走到右上角的綠色目標（目標標記只有外觀、沒有碰撞，LiDAR 看不到）
- 障礙物高度統一 1 m（投影片示意圖較矮，但 2D LiDAR 掃描平面會隨步態傾斜，障礙物要明顯高於它，見「LiDAR 地面點過濾」）
- 目標在起點看不到的地方（被長方塊與隔牆擋住），要邊走邊用 SLAM 建圖、重新規劃

## 檔案說明

| 路徑 | 內容 |
|---|---|
| `launch/dogzilla_sim.launch.py` | 一鍵啟動整條管線，參數 `controller:=dwb\|rpp\|rl`、`model:=s1\|s1_box\|champ`、`gui`、`rviz`、`world`（預設 `/worlds/project3.world`） |
| `launch/dogzilla_sim.rviz` | RViz 設定：SLAM 地圖、costmap、LiDAR、Nav2 路徑、RL 路徑 |
| `launch/scan_ground_filter.py` | LiDAR 地面點過濾節點（見下方說明，實機也適用） |
| `launch/cmd_vel_safety.py` | 運動安全濾波 `/cmd_vel_raw` → `/cmd_vel`（轉彎降速、限制轉向變化率，實機也適用） |
| `launch/nav2_navigation.launch.py` | Nav2 navigation_launch 的複本，只把輸出改到 `/cmd_vel_raw` |
| `config/slam.yaml` | 模擬用 SLAM Toolbox 設定（改動處皆有註解） |
| `description/make_s1_official.py` → `dogzilla_s1_official.urdf.xacro` | **官方 DOGZILLA S1 URDF** 產生器與產出的模擬模型（預設） |
| `description/dogzilla_s1.urdf.xacro` | 手工方塊模型（`model:=s1_box`，對照用） |
| `config/gait_s1.yaml` / `config/ros_control_s1.yaml` | S1 的步態參數與關節 PID |
| `description/dogzilla_sim.urdf.xacro` | 舊的 CHAMP/XGO 放大模型（`model:=champ`，見「Gazebo 崩潰修正」） |
| `worlds/project3.world` | **預設場地**，照 `project.pptx` 第 1 頁的場景與 Concept diagram 建立（見下方「場地」） |
| `worlds/obstacle_test.world` | 舊的 6 m × 6 m 圍牆場地，3 個方塊 + 3 個圓柱障礙物（`world:=/worlds/obstacle_test.world`） |
| `nav2_params.yaml` / `nav2_params_rpp.yaml` | Nav2 參數 (DWB / RPP)，尺寸、速度、加速度已依 S1 調整 |
| `rl/` | RL 避障：`nav_env.py` 2D 光達環境、`train.py` PPO 訓練、`rl_controller.py` ROS 2 節點、`models/` 訓練好的權重 |
| `benchmark_nav.py` | 避障評測：用 Gazebo 真值算到達率、碰撞次數、最小離障距離（第 3 個參數選場地，目標點與障礙物表在檔頭 `WORLDS`） |
| `tools/` | 診斷與測試工具（見下方「診斷工具」） |
| `results/` | 已記錄的測試結果：`benchmarks/`（避障 benchmark CSV）、`sweeps/`（參數掃描）、`training/`（RL 訓練紀錄），說明見 `results/README.md` |
| `logs/` | 執行時輸出：`sim.log` 與新跑的 benchmark 結果（一開始是空的） |
| `maps/` | 掛載到容器 `/maps`，可用來存 SLAM 地圖（一開始是空的） |
| `DOGZILLALib/` | 原廠 `DOGZILLALib` 的複本，建 Docker 映像時安裝用 |

`vendor/Program/`、`vendor/DOGZILLA_README/` 裡的原廠程式碼**完全沒有修改**，模擬用到的改動都放在 `sim_env/`。

## DOGZILLA S1 模型（官方 URDF）

預設模型 `description/dogzilla_s1_official.urdf.xacro` 來自 **Yahboom 官方的 DOGZILLA S1 URDF**：
`vendor/Program/yahboomcar_ws_ros2/src/champ/champ_description/urdf/xgo_rviz.xacro`（SolidWorks 1:1 匯出，網格在 `meshes/XGO/`，
Yahboom 教材 *Rviz simulation in ros2 environment* 用的就是這份）。外觀網格、關節位置、質量、質心與慣量都沿用官方值，
尺寸與官方運動學參數一致（前後髖距 0.093 + 0.057 = 0.150 m、大腿 0.060 m、腳掌間距約 107 mm）。
（舊的 CHAMP 模型是把同一批網格放大 2 倍，所以比實機大。）

模型由 `description/make_s1_official.py` 從官方檔自動產生，**不要直接改產生出來的 xacro**。
為了能在 Gazebo + CHAMP 裡模擬，產生器做了以下修正（詳細原因見檔頭說明）：

| # | 修正 | 原因 |
|---|---|---|
| 1 | 左前膝關節更名為 `lf_lower_leg_joint` | 官方打錯成與連桿同名 `lf_lower_leg_link`，CHAMP 找不到 |
| 2 | 關節軸正負號統一（髖 +x、大腿/膝 +y） | 官方各腿方向不一，CHAMP 逆運動學假設四腿同向 |
| 3 | 加上 `foot_link`（在大腿關節平面上） | 官方沒有腳掌座標系；若放在網格真實腳位（往內 18 mm），CHAMP 會把腿外撇約 10° |
| 4 | 碰撞改用 box，LiDAR/相機外殼不設碰撞 | STL 碰撞會讓 gzserver 段錯誤（見「Gazebo 崩潰修正」） |
| 5 | 關節上限改成舵機規格 0.44 N·m、10.5 rad/s，加關節阻尼 | 官方填的是預設值 25 / 1.5 |
| 6 | 加 LiDAR、IMU、ros2_control、站姿初始角 | 官方模型只有外型 |
| 7 | 右側髖部質量改成與左側鏡像相同 | 官方右側是簡化網格，質量只有左側一半（9.8 vs 19.6 g）；不修的話邊走邊左轉時傾斜達 30°，右轉只有 7° |

注意：官方 URDF 的總質量約 0.6 kg，可能沒有算進舵機、樹莓派和電池，實機應該更重。

### 減少晃動

官方模型一開始走路很晃（前進時機身 roll/pitch RMS 7.0°、最大 15°，高度起伏 2 cm），前進也幾乎走不動。
原因是關節 PID 在扭力上限 ±0.44 N·m 之間**顫振**：膝關節 60–76% 的時間都處於扭力飽和，但追蹤誤差卻不到 5°。
官方小腿只有 23 g，舊的阻尼增益已接近離散控制的穩定極限。調整後：

- 控制頻率 500 → 1000 Hz；PID p=8、d=0.005；主要阻尼改由 URDF 關節阻尼 0.01 + ODE `implicitSpringDamper` 提供（物理引擎隱式求解，一定穩定）
- 步態：抬腳 0.018 m、支撐相 0.35 s、站高 0.105 m
- 參數都用 `tools/gain_sweep.sh` 掃描選出（結果在 `results/sweeps/gain_sweep_pid.txt`、`results/sweeps/gain_sweep.txt`）

| | 調整前 | 調整後 |
|---|---|---|
| 前進（指令 0.2 m/s）實際速度 | 0.025 m/s | 0.134 m/s |
| 前進晃動 RMS / 最大 | 7.0° / 15.2° | 1.8° / 3.4° |
| 原地轉（指令 0.8 rad/s） | 0.20 rad/s | 0.75 rad/s，晃動 RMS 3.6° |
| 機身高度起伏 | 18–24 mm | 約 7 mm |
| 膝關節扭力飽和時間 | 58–76% | 0% |

`tools/gait_eval.py` 可以重測這些數字。

### 其他 S1 相關設定

- **以站姿、暫停狀態啟動**：舵機只有 0.44 N·m，若以關節 0 度生成，控制器載入前機身就會趴下而且撐不起來。
  Gazebo 以暫停狀態啟動，機器狗以站姿生成（`initial_value`），控制器設定好後才開始物理模擬。
- **速度上限 0.2 m/s**：`config/gait_s1.yaml` 的 `max_linear_velocity_x`，CHAMP 會把所有來源的 `/cmd_vel` 截在這個值。
  更快的速度加上轉向容易翻倒（`tools/combo_test.py`），**實機也建議遵守**。
- **運動安全濾波**（`launch/cmd_vel_safety.py`）：這個步態邊轉邊走幾乎走不動（0.15 m/s 加 0.5 rad/s 時實際只前進 0.034 m/s），
  DWB 不知道，會一直送「0.2 m/s 加大角速度」並左右來回修正，機器狗原地踏步，最後在高速中反轉轉向時翻倒
  （`tools/flip_monitor.py` 錄到 0.6 s 內傾斜 9° → 85°）。濾波器讓轉彎時自動降速（`v × max(0.15, 1 − |ω|)`）、
  限制角速度變化率 1.5 rad/s²。Nav2（`launch/nav2_navigation.launch.py`，只改了輸出話題）與 RL 都輸出到 `/cmd_vel_raw`，
  經濾波後才送到 `/cmd_vel`。實機也適用。
- **Nav2**：`robot_radius` 0.13 m、膨脹半徑 0.40 m、到達容許誤差 0.15 m、加速度限制 0.5 m/s²。
- **LiDAR 地面點過濾**：掃描平面離地約 0.19 m，`margin` 0.5、IMU 歷史 0.3 s。
- **RL 策略**依官方模型的實測速度反應（含邊轉邊走會變慢）重新訓練 4M 步（`rl/nav_env.py`；舊場地離線評估 92% 到達）。
  舊權重在 `rl/models/s1_official_2M/`、`rl/models/s1_box/`（手工方塊模型）與 `rl/models/champ_old/`（放大模型）。

其他模型（對照用）：`model:=s1_box` 是之前依官方尺寸手工做的方塊模型（`description/dogzilla_s1.urdf.xacro`），
`model:=champ` 是舊的放大模型。

## Gazebo 崩潰修正

原本 gzserver 會在機器狗生成後隨機段錯誤（exit -11）。用 gdb 抓到的堆疊：

```
RaySensor::UpdateImpl -> ODEMultiRayShape::UpdateRays -> dSpaceCollide2 -> SIGSEGV (libgazebo_ode)
```

原因是 LiDAR 射線執行緒與物理執行緒同時存取 ODE 的三角網格 (trimesh) 碰撞模型，
而原廠 URDF 的碰撞形狀全是 STL，其中 `laser_link.STL` 有 5 萬多個三角形。
`description/dogzilla_sim.urdf.xacro` 把碰撞形狀改為 box（外觀仍用原本的 XGO 模型），
LiDAR 與相機本體不設碰撞。修正前 3/3 次崩潰，修正後連續測試皆穩定。

相機感測器預設關閉（避障只用 LiDAR），要開啟可把 xacro 參數 `camera` 預設值改成 `true`。

碰撞形狀的腳掌用平底 box（與原 `lower_leg_simple.stl` 同尺寸）。試過「小腿 box + 腳掌球」，
球只有單點接觸會打滑，前進速度只剩約 15%；平底 box 與原網格版速度相同（指令 0.2 m/s → 實際約 0.09 m/s）。

## 腿部里程計修正

CHAMP 的 `state_estimation` 需要 `/joint_states` + `/foot_contacts` 才能算出 `/odom/raw`。
在 Gazebo 模式下步態控制器不發佈接觸狀態，要靠 `champ_gazebo/contact_sensor`，
但它在 `champ_gazebo` 的 launch 裡被註解掉了（當時誤以為它造成崩潰），結果 `/odom` 速度恆為 0，
SLAM 只能純靠掃描比對、位姿漂移，Nav2 目標換算到地圖外而失敗。
`launch/dogzilla_sim.launch.py` 自己啟動 `contact_sensor`，修正後 `/odom/raw` 速度與 Gazebo 真值一致。

量測步態速度：`docker exec dogzilla-sim bash -lc 'source /opt/ros/humble/setup.bash; python3 /tools/walk_test.py'`

## LiDAR 地面點過濾與 SLAM 調整

CHAMP 步態走路時機身俯仰可達 8°、側滾 6°（`tools/tilt_test.py` 量測）。2D LiDAR 掃描平面（離地約 0.39 m）
跟著傾斜，在約 2.8 m 外會打到地面，SLAM 地圖上出現大量不存在的碎片障礙物，膨脹後把通道堵死。
實機四足機器人一樣會遇到這個問題。

`launch/scan_ground_filter.py` 讀 `/imu/data` 的 roll/pitch，算出每條射線在多遠會碰到地面，
超過該距離的回波判定為地面並丟棄，輸出 `/scan_filtered`（SLAM、Nav2 costmap、RL 都改用它）。
一圈掃描需 0.1 s，所以取最近 0.3 s 內最壞的傾角；`margin` 0.5 代表離地低於掃描高度一半的點都視為地面。機身水平時掃描原封不動。

`config/slam.yaml` 另外依里程計特性調整：EKF 融合 IMU 絕對航向所以角度很準（`angle_variance_penalty` 1.0→3.0），
腿部里程計距離低估約 30%（`distance_variance_penalty` 0.5→0.3），並更頻繁做掃描比對（`minimum_travel_*` 0.5→0.2）。

其他修正：LiDAR 射線原點原本在外殼上方 7.5 cm，掃描平面只比 0.5 m 高的障礙物低 1.6 cm，改從 LiDAR 頭部發射；
場地障礙物與圍牆加高到 1 m（2D LiDAR 需要障礙物高於晃動中的掃描平面）。

## Benchmark 結果

同一場地 7 個目標點依序走完。「控制器回報完成」代表 Nav2 或 RL 認為已到達（導航避障能力）；
「真值到達」用 Gazebo 真實位置判定（離目標 < 0.35 m），也包含 SLAM 定位誤差。各為單次執行結果。

**官方 S1 URDF（預設），`project3` 場地**

| 控制器 | 控制器回報完成 | 真值到達 | 擦碰 | 最小離障距離 | 翻倒 | CSV |
|---|---|---|---|---|---|---|
| Nav2 DWB | 4/7 | 0/7 | 1 | 0.12 m | 否 | `results/benchmarks/bench_s1off_dwb.csv` |
| Nav2 RPP | 3/7 | 1/7 | 9 | 0.07 m | 否 | `results/benchmarks/bench_s1off_rpp.csv` |
| RL (PPO，官方模型版) | 3/7 | 3/7 | 34 | 0.04 m | 是（第 7 段） | `results/benchmarks/bench_s1off_rl.csv` |

- RPP 第 1 段（就是投影片上的任務：從起點繞過障礙物走到右上角目標）成功到達，誤差 0.04 m、用時 124 s。
- DWB 在加運動安全濾波之前會翻倒（第 1 段就翻）；加上之後沒有再翻。
- 主要問題：S1 走得慢（約 0.13 m/s，轉彎時更慢），project3 場地較大、通道較窄，長的路段常在 150 s 內走不完；
  後段 SLAM 誤差會累積到 1–1.7 m（RPP），真值到達率因此偏低。RL 在窄通道裡擦碰很多，還需要針對窄通道訓練。
- 「控制器回報完成」與「真值到達」差距大的段落，多半是 SLAM 定位誤差（`slam_err_m` 欄位）。

**手工 S1 方塊模型（`model:=s1_box`），舊的 `obstacle_test` 場地**

| 控制器 | 控制器回報完成 | 真值到達 | 碰撞 | 最小離障距離 | 翻倒 | CSV |
|---|---|---|---|---|---|---|
| Nav2 DWB | 6/7 | 2/7 | 0 | 0.22 m | 否 | `results/benchmarks/bench_s1v2_dwb.csv` |
| Nav2 RPP | 6/7 | 4/7 | 0 | 0.15 m | 否 | `results/benchmarks/bench_s1v2_rpp.csv` |
| RL (PPO, S1 版) | 4/7 | 4/7 | 2（擦碰） | 0.08 m | 否 | `results/benchmarks/bench_s1v2_rl.csv` |

**舊的 CHAMP 放大模型（`model:=champ`，對照用）**

| 控制器 | 真值到達 | 碰撞 | 最小離障距離 | CSV |
|---|---|---|---|---|
| Nav2 DWB | 7/7 | 0 | 0.34 m | `results/benchmarks/bench_dwb_v2.csv` |
| Nav2 RPP | 4/7 | 0 | 0.29 m | `results/benchmarks/bench_rpp_v2.csv` |
| RL (PPO, 舊版) | 5/7 | 0 | 0.19 m | `results/benchmarks/bench_rl_v3.csv` |

重跑 benchmark：`tools/bench_all.sh <前綴> dwb rpp rl`（依序啟動無頭模擬並跑完，約 40 分鐘）。

## RL 避障控制器

- 觀測：LiDAR 360 線每 10° 取最小值（36 維）+ 與路徑前方瞄準點的距離/方位 + 上一步動作 + 目前速度
- 動作：前進速度 `v ∈ [0, 0.2] m/s`、轉向 `ω ∈ [-1, 1] rad/s`，輸出端限制加速度 0.5 m/s²、2 rad/s²
- 架構：Nav2 `planner_server` 給全域路徑，RL 瞄準路徑前方 0.8 m 的點做局部避障
  （RL 模式下不開 Nav2 的 controller_server，避免兩邊搶 `/cmd_vel`）
- 部署推理只用 numpy（`rl_policy.py`），Raspberry Pi 上不需要裝 PyTorch

模擬只需要 `models/policy.npz`（已附上），**不用另外裝任何東西**。要重新訓練時，在主機上建 Python 環境（Python 3.10 以上）：

```bash
cd sim_env/rl
python3 -m venv .venv
env -u PYTHONPATH .venv/bin/pip install --extra-index-url https://download.pytorch.org/whl/cpu -r requirements.txt
env -u PYTHONPATH .venv/bin/python train.py 4e6     # 產生 models/ppo_nav.zip 與 models/policy.npz (約 20 分鐘)
env -u PYTHONPATH .venv/bin/python eval_offline.py  # 在固定測試場地評估
```

> `env -u PYTHONPATH`：避免主機的 PYTHONPATH（例如指向 ROS 或其他 Python 版本的 dist-packages）讓 venv 載入到錯誤版本的套件。

## 診斷工具

都在 `tools/`（容器內為 `/tools`），模擬啟動後以 `docker exec dogzilla-sim bash -lc 'source /opt/ros/humble/setup.bash; python3 /tools/<工具>'` 執行：

| 工具 | 用途 |
|---|---|
| `walk_test.py` | 量測前進與原地轉的實際速度 |
| `tilt_test.py` | 量測走路時機身的 roll/pitch |
| `combo_test.py [v,w ...]` | 前進加轉向的組合是否會翻倒 |
| `wiggle_test.py <v> <週期>` | 左右來回轉向是否會翻倒 |
| `joint_track.py <v> <w>` | 各關節追蹤誤差與扭力飽和比例 |
| `leak_check.py` | 用真值檢查 `/scan_filtered` 是否還有漏網的地面點 |
| `imu_vs_truth.py` | 比對 IMU 與真值的傾角 |
| `pose_check.py` | 比對 SLAM、里程計與真值位姿 |
| `costmap_snapshot.py <png>` | 把 SLAM 地圖與全域代價地圖存成圖片 |
| `flip_monitor.py` | 監測翻倒，印出翻倒前 6 秒的位置、傾斜與速度指令 |
| `startup_trace.py` | 記錄開始模擬後幾秒內的機身高度與關節角 |
| `gait_eval.py` | 步態綜合評估：速度、晃動 RMS/最大值、高度起伏、靜止漂移 |
| `gain_sweep.sh` / `gait_sweep.sh` / `bench_all.sh` | 在主機執行：關節增益與步態掃描（官方模型）/ 步態掃描（方塊模型）/ 三種控制器依序 benchmark |

## 已知限制

- 模擬中 RL 節點與 benchmark 的位姿取自 Gazebo 真值 (`/demo/odom/ground_truth`)；實機要換成 SLAM 的 `map→base_link`。
- S1 模型的機身高度、總重與 LiDAR 位置是推估值；步態是 CHAMP 的，不是 Yahboom 韌體的步態，實機的速度與晃動需要重新量測。
- 啟動時 `gazebo_ros2_control: Parameter 'hold_joints' has already been declared` 的 ERROR 是 12 個獨立
  ros2_control 區塊造成的無害訊息，不影響控制。
- `champ_gazebo/contact_sensor` 會佔滿一顆 CPU（Gazebo 每個物理步都送出接觸資料），但模擬即時率仍約 0.9。
- ROS 2 中介層改用 CycloneDDS（`docker-compose.yml` 的 `RMW_IMPLEMENTATION`）。預設的 FastDDS 在節點多時常出現服務回應逾時，
  讓 Nav2 卡在 Configuring；改用 CycloneDDS 後連續 4 次啟動都正常。launch 內仍保留 `nav2_watchdog` 作為保險。
- Docker 內 RViz 用軟體渲染，會印出 `GLSL link result` 的 ERROR，不影響顯示。
- SLAM 地圖仍有少量零星碎片、長時間運行後有輕微旋轉漂移。
