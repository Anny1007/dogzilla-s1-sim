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
Yahboom 教材 *Rviz simulation in ros2 environment* 用的就是這份）。外觀網格、關節位置、各連桿質心都沿用官方值（質量改成實機重量，見下方 #8），
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

| 8 | 質量改為實機重量：本體 875 g + LiDAR 45 g | 官方 URDF 的本體只有 611 g（沒算進電池等），實機秤重約 870–880 g；各連桿質量與慣量等比例放大，LiDAR 連桿由 14 g 改為 45 g |
| 9 | LiDAR 後移到 `x = −0.06 m`（官方 −0.0167 m） | 對行走平衡影響最小的位置，見下方「實機重量與 LiDAR 安裝位置」 |

### 實機重量與 LiDAR 安裝位置

模型的重量是 **DOGZILLA S1 本體 875 g（實機秤重約 870–880 g）+ 加裝的 DOGZILLA S2 LiDAR 45 g，共 920 g**。
產生器的 `S1_MASS`、`LIDAR_MASS`、`LIDAR_X` 三個常數可以直接改（也可用環境變數 `S1_MASS` / `S1_LIDAR_MASS` / `S1_LIDAR_X` 暫時覆蓋）。

**LiDAR 放哪裡最不影響行走**：站姿下本體質心在四腳支撐中心**前方約 17 mm**（`python3 tools/com_check.py`），
小跑步態靠對角兩腳支撐，質心偏前會讓機身往前栽。所以 LiDAR 越往後放，整機質心越接近支撐中心；
但 45 g 只佔總重 5%，最多只能把質心拉回 4 mm 左右，再往後則後腿負擔變重。左右方向置中，高度維持官方值
（掃描平面 z = 93 mm，要高於相機外殼的 80 mm，再低前方會被機身擋住）。

在 Gazebo 裡對每個前後位置重產模型、實際走一遍（`tools/tune_sweep.sh`，每個位置前進 / 原地轉 / 邊走邊轉）：

| LiDAR 位置 x（base_link，+ 為前方） | 整機質心在支撐中心前方 | 前進晃動 RMS | 原地轉速率（指令 0.8） | 最大傾斜 | 後腿膝關節扭力飽和 | 結果 |
|---|---|---|---|---|---|---|
| +30 mm | 19.2 mm | — | — | 166° | — | 翻倒 |
| +4 mm | 17.9 mm | 3.1° / 19.7° | 0.77 / 0 rad/s | 67° | 6–7% | 第二次測試翻倒 |
| −16.7 mm（官方位置，上層平台最後緣） | 16.9 mm | 2.5–3.1°，一次 10.0° | 0.79–0.81 rad/s | 6°，一次 28° | 5–29% | 4 次測試有 1 次大幅前後晃（28°） |
| −40 mm | 15.8 mm | 1.7–2.5° | 0.54–0.85 rad/s | 5–11° | 6–19% | 穩 |
| −50 mm | 15.3 mm | 2.1–2.5° | 0.72–0.84 rad/s | 5–9° | 1–14% | 穩 |
| **−60 mm（採用）** | 14.8 mm | 2.2–2.8° | 0.84–0.89 rad/s | 4.9–5.3° | 0–11% | 穩，6 次測試最大傾斜都在 5.3° 內 |
| −70 mm | 14.3 mm | 2.3–2.7° | 0.86–0.92 rad/s | 5.1–5.4° | 7–30% | 穩，後腿負擔較重 |
| −86 mm（貼齊機尾） | 13.5 mm | 2.2–2.7° | 0.88–0.90 rad/s | 5.0° | 12–21% | 穩，後腿負擔較重 |
| （對照）不裝 LiDAR | 17.5 mm | 1.8–1.9° | 0.80–0.82 rad/s | 5–6° | 0–2% | 穩 |

- 比官方位置更前面會翻倒，官方位置偶爾大幅晃動；**−40 ~ −86 mm 都穩，彼此差異小於量測雜訊**（同一組參數重複量測，晃動差 ±0.5°）。
  取中間的 **−60 mm**：膝關節扭力飽和最低、6 次測試沒有任何一次異常，安裝時差 1–2 cm 也不影響。
- 這個位置在機身後段較低的平台上方（平台 z = 36 mm，LiDAR 底面 z = 68 mm），**實機需要一個約 32 mm 高的支架**，
  LiDAR 中心對準機身中線、距離機尾約 46 mm。
- 原始資料：`results/sweeps/tune_875g_round2_lidar_x.txt`、`tune_875g_round3_lidar_x_repeat.txt`
  （`lidar_sweep_before_retune.txt` 是還沒重調增益時的結果：875 g 直接套用舊增益，各位置都走不穩）。

**加重後重調的參數**：關節 PID 的 p 由 8 改為 **14**（`config/ros_control_s1.yaml`）。p=8 是質量 625 g 時選的，
加到 920 g 後 p=8–12 撐不住會翻倒，p=17 以上又開始顫振（`results/sweeps/tune_875g_round1.txt`、`round2`）。
站高 0.105 m、抬腳 0.018 m、支撐相 0.35 s 維持不變（改高、改低、抬高腳都變差）。調整後：

| 指令 | 實際（`results/speed_response_s1_875g_run*.txt`） | 最大傾斜 |
|---|---|---|
| 前進 0.10 / 0.15 / 0.20 m/s | 0.039 / 0.084 / 0.10–0.13 m/s（0.05 m/s 以下不會前進） | 3–6° |
| 原地轉 0.3 / 0.6 / 1.0 rad/s | 0.32 / 0.61 / 1.06 rad/s（同時以約 0.05 m/s 慢慢後退） | 4–6° |
| 前進 0.15 m/s + 轉 0.5 rad/s | 0.08 m/s、0.6 rad/s（625 g 時只剩 0.034 m/s） | 4–5° |
| 前進 0.20 m/s + 轉 0.2 rad/s | 0.10–0.13 m/s（625 g 時在這裡翻倒） | 5–7° |

下面「減少晃動」一節是 625 g 時期的紀錄（當時 p=8），方法相同、數字是舊的。

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
- **RL 策略**依 920 g 模型的實測速度反應重新訓練（v6，見「RL 避障控制器」）。
  舊權重（625 g 時期）在 `rl/models/s1_v5_measured/`、`s1_v3_shield/`、`s1_official_2M/` 等，`rl/models/s1_box/` 是手工方塊模型、`rl/models/champ_old/` 是放大模型。

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

**目前的模型（官方 S1 URDF，本體 875 g + LiDAR 45 g），`project3` 場地**（2026-10-10）

| 控制器 | 控制器回報完成 | 真值到達 | 擦碰 | 最小離障距離 | 成功段平均用時 | 翻倒 | CSV |
|---|---|---|---|---|---|---|---|
| **RL（PPO v6，預設權重）** | **7/7** | **7/7** | 0 | 0.13 m | 80.2 s | 否 | `results/benchmarks/bench_s1_875g_rl_v6.csv` |
| RL（PPO v5，625 g 時訓練的舊權重） | 7/7 | 7/7 | 0 | 0.13 m | 85.1 s | 否 | `results/benchmarks/bench_s1_875g_rl_v5.csv` |
| Nav2 RPP | 3/7 | 1/7 | 3 | 0.09 m | 98.4 s | 否 | `results/benchmarks/bench_s1_875g_rpp.csv` |
| Nav2 DWB | 1/7 | 0/7 | 6 | 0.11 m | — | 否 | `results/benchmarks/bench_s1_875g_dwb.csv` |

- RL 7 段全部到達、沒有擦碰也沒有翻倒；第 1 段就是投影片上的任務（從起點繞過障礙物走到右上角目標），用時 117 s。
  SLAM 誤差全程在 0.06–0.13 m（RL 走得平順，沒有原地打轉）。
- 同一份 v5 權重在 625 g 的舊模型上是 3/7 且翻倒（下表）：改善主要來自模型本身變穩（重量、LiDAR 位置與關節增益調整後，邊走邊轉不再翻倒），不是策略變了。
- Nav2 的 RPP / DWB 沒有為新重量重調，表現與之前差不多（走得慢、SLAM 誤差累積到 1 m 以上）。
- 各為**單次執行**結果。RL 節點與評分用的位姿是 Gazebo 真值，實機要換成 SLAM 定位（見「已知限制」）。

**625 g 時期（官方 URDF 原始質量），`project3` 場地**（2026-10-02）

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
- 動作：前進速度 `v ∈ [0, 0.15] m/s`、轉向 `ω ∈ [-1, 1] rad/s`，輸出端限制加速度 0.5 m/s²、2 rad/s²
- 安全保護（`rl/safety_shield.py`）：RL 想要的速度先經過碰撞檢查，會撞的動作換成最接近的安全動作；訓練與部署都開
- 架構：Nav2 `planner_server` 給全域路徑，RL 瞄準路徑前方 0.8 m 的點做局部避障
  （RL 模式下不開 Nav2 的 controller_server，避免兩邊搶 `/cmd_vel`）
- 部署推理只用 numpy（`rl_policy.py`），Raspberry Pi 上不需要裝 PyTorch

**目前的預設權重（v6，`rl/models/policy.npz` = `rl/models/s1_v6_finetune/best/`）** 是針對 920 g 模型訓練的：

- 訓練環境 `rl/nav_env.py` 是純 Python 的 2D 光達環境（隨機場地：零散障礙物、長牆、U 形陷阱、走廊），用 PPO 訓練（只用 CPU）。
- 機器狗在 2D 環境裡怎麼動，用的是 Gazebo 實測的速度反應（`rl_policy.speed_model_875g`，資料在 `results/speed_response_s1_875g_run*.txt`）：
  指令低於約 0.06 m/s 不會前進、0.15 m/s 實際約 0.084 m/s、運動安全濾波的轉彎降速。
- **訓練經過**（三次嘗試都留有紀錄，接手的人可以少走冤枉路）：
  1. 速度上限 0.20 m/s、從零訓練 4M 步：2D 評估很好（投影片場地 100%），但放進 Gazebo 走完前 2 段後，在障礙物旁反覆「停—衝」時翻倒
     （`results/training/flip_v6_vmax020.txt`）。起停壓力測試確認 0.20 m/s 急起急停時傾斜達 57–60°，0.15 m/s 最大只有 10°
     （`tools/startstop_test.py`，`results/startstop_test_875g.txt`），所以上限維持 0.15 m/s。
  2. 上限 0.15 m/s、從零訓練 4M 步：只到 70–87%，比舊的 v5 權重（625 g 時訓練）差（`rl/models/s1_v6_scratch/`，`results/training/eval_compare_v6_scratch.txt`）。
  3. **以 v5 權重為起點、在 920 g 的速度模型上接續訓練 2M 步**（`train.py --init=...`，約 26 分鐘）：採用這一版。
- 速度模型另有一個含「原地轉時以 0.05 m/s 後退」的版本（`s1_v6_finetune_creep/`），2D 評估略高但沒有在 Gazebo 實測。

| 2D 環境評估（`results/training/eval_compare_v6.txt`，都用 920 g 的速度模型） | v6（採用） | v6 含後退版 | v6 從零訓練 | v5（625 g 時訓練） | 追路徑點（傳統基準） |
|---|---|---|---|---|---|
| 舊測試場地 210 段 | 90% 到達、0 碰撞、平均 52.2 s | 94%、1、51.6 s | 70%、0、58.4 s | 90%、0、53.3 s | 63%、77 次碰撞 |
| 投影片場地 project3 210 段（訓練沒看過） | 99%、0 碰撞、69.9 s | 100%、0、68.7 s | 87%、0、70.6 s | 100%、0、70.1 s | 52%、100 次碰撞 |
| 隨機新場景 200 個 | 88%、3 碰撞、43.7 s | 90%、3、43.4 s | 78%、3、44.6 s | 89%、3、44.0 s | 64%、73 次碰撞 |

在 2D 環境裡 v6 與 v5 差不多（差異在雜訊範圍內）；放進 Gazebo 兩者都是 7/7，v6 平均每段快約 5 秒（見上方「Benchmark 結果」）。
訓練曲線：`results/training/ppo_metrics_v6.png`。

**接手的人可以從這裡繼續改善**：
- 獎勵與場景生成在 `rl/nav_env.py`（檔頭有 v1–v6 每一版改了什麼、為什麼），訓練參數在 `rl/train.py`，
  重新訓練、評估、放進 Gazebo 測試的步驟在 `SETUP.md` 第 6 節。
- 走得慢的主因是運動安全濾波（`launch/cmd_vel_safety.py`）的轉彎降速：它是 625 g 時「邊走邊轉會翻倒」而加的，
  920 g 下步態本身邊走邊轉已不會減速也沒翻倒（直接送 0.15 m/s + 0.5 rad/s 仍有 0.08 m/s、傾斜 5°）。
  放寬 `min_scale` / `w_full_stop`、重新量測速度反應後再訓練，有機會明顯變快（需要先用 `tools/startstop_test.py` 之類的測試確認不會翻倒）；這次沒有動它。
- Gazebo 的結果是單次執行，而且場地只有 project3 一個；要確認穩定性需要多跑幾次、換場地。

模擬只需要 `models/policy.npz`（已附上），**不用另外裝任何東西**。要重新訓練時，在主機上建 Python 環境（Python 3.10 以上）：

```bash
cd sim_env/rl
python3 -m venv .venv
env -u PYTHONPATH .venv/bin/pip install --extra-index-url https://download.pytorch.org/whl/cpu -r requirements.txt
env -u PYTHONPATH .venv/bin/python train.py 4e6 models/my_run --shield   # 訓練 4M 步 (約 45 分鐘), 存到 models/my_run/
env -u PYTHONPATH .venv/bin/python eval_compare.py "目前=models/policy.npz" "新=models/my_run/best/policy.npz"   # 與目前的權重比較
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
| `speed_response.py` | 各種速度指令下的實際前進 / 轉向速度與傾斜（RL 訓練環境的速度模型就是照這份結果擬合的） |
| `stand_drift.py` | 靜止站立時的漂移 |
| `tune_sweep.sh` | 在主機執行：質量 / LiDAR 位置 / 關節增益 / 站姿掃描（`P=14,LX=-0.06` 這種寫法，每組重產模型並重啟模擬） |
| `com_check.py` | 在主機執行：站姿下的整機質心與四腳支撐中心（靜態計算，不需要 Gazebo） |
| `gain_sweep.sh` / `gait_sweep.sh` / `bench_all.sh` | 在主機執行：關節增益與步態掃描（官方模型）/ 步態掃描（方塊模型）/ 三種控制器依序 benchmark |

## 已知限制

- 模擬中 RL 節點與 benchmark 的位姿取自 Gazebo 真值 (`/demo/odom/ground_truth`)；實機要換成 SLAM 的 `map→base_link`。
- S1 模型的總重已改成實機秤重（本體 875 g + LiDAR 45 g），但**重量在各連桿間的分布是把官方 URDF 等比例放大**，
  電池、樹莓派的實際位置沒有量；LiDAR 位置是模擬選出的建議值。步態是 CHAMP 的，不是 Yahboom 韌體的步態，實機的速度與晃動需要重新量測。
- **靜止站立時機身會以約 1 cm/s 慢慢往後滑**（`tools/stand_drift.py`；625 g 時期就有，`results/sweeps/gain_sweep_pid.txt` 的 drift 欄）。
  與關節 PID 的微幅顫振有關（d 增益 0.03 時不會滑，但走路會晃）。導航中有閉迴路修正所以不明顯，但到達目標後停著不動會慢慢離開目標點，
  啟動後不下目標放著也會退到後方的牆邊。原地轉向時以約 5 cm/s 後退也是同一類現象，RL 的速度模型有把它算進去。
- RL 訓練環境假設 LiDAR 在機身中心；實際在中心後方 6 cm，所以機身前方的障礙物實際上比 RL 以為的近 6 cm（碰撞判定距離 0.16 m 有留餘量）。
- 啟動時 `gazebo_ros2_control: Parameter 'hold_joints' has already been declared` 的 ERROR 是 12 個獨立
  ros2_control 區塊造成的無害訊息，不影響控制。
- `champ_gazebo/contact_sensor` 會佔滿一顆 CPU（Gazebo 每個物理步都送出接觸資料），但模擬即時率仍約 0.9。
- ROS 2 中介層改用 CycloneDDS（`docker-compose.yml` 的 `RMW_IMPLEMENTATION`）。預設的 FastDDS 在節點多時常出現服務回應逾時，
  讓 Nav2 卡在 Configuring；改用 CycloneDDS 後連續 4 次啟動都正常。launch 內仍保留 `nav2_watchdog` 作為保險。
- Docker 內 RViz 用軟體渲染，會印出 `GLSL link result` 的 ERROR，不影響顯示。
- SLAM 地圖仍有少量零星碎片、長時間運行後有輕微旋轉漂移。
