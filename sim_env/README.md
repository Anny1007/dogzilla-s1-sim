# DOGZILLA S1 自主避障模擬環境

對應 `docs/project.pptx`（Project 3：Autonomous Obstacle Avoidance for a Quadruped Robot）的目標：

- 機器狗用新增的 **LiDAR** 偵測障礙物
- **自己走到目標點**，途中**繞過障礙物**

**目前的狀態**：模型、整條管線、測試場地與評分程式都已備好；強化學習的避障**還沒做好**（`rl/models/` 裡是只訓練 10 萬步的權重，走得不穩、常卡在牆角）。

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
在地圖上點一下目標位置，機器狗就會自己規劃路徑往那裡走；**Gazebo 裡同一個位置會出現一支藍色旗子，代表這次的終點**（再點一次，旗子會移到新的位置）。三種控制器模式都用同一個按鈕。
（場地裡固定的綠色柱子是自動評測的第 1 個目標，和你點的目標無關。SLAM 定位有誤差時，旗子會和 RViz 上點的位置差一點，旗子的位置才是機器狗實際要去的地方。）

其他常用指令：

```bash
./run.sh shell                        # 進到正在跑的模擬容器
./run.sh sim dwb gui:=false rviz:=false   # 無頭模式
./run.sh bench my_test dwb            # 自動跑 7 個目標點, 結果存 logs/bench_my_test.csv
./run.sh bench my_rl  rl              # (模擬需以 ./run.sh sim rl 啟動)

# 換場景: 啟動與 benchmark 都要指定 (場景 1 project3 / 場景 2 corridor (預設) / 場景 3 paper_fig4 / 場景 4 clutter, 見「場地」)
./run.sh sim rl world:=/worlds/clutter.world
./run.sh bench my_rl_clutter rl clutter
```

## 場地

測試場景編號為 **場景 1–4**（另有一個最早期的 `obstacle_test`，未編號）。每個場地都是圍牆圍起來的平地，機器狗在原點朝 +x 生成，障礙物高度統一 1 m（2D LiDAR 的掃描平面會隨步態傾斜，
障礙物要明顯高於它，見「LiDAR 地面點過濾」）。啟動時用 `world:=/worlds/<名稱>.world` 選，benchmark 的第 3 個參數給同一個名稱（指令裡用的是名稱，不是編號）。

| 場景 | 名稱（指令裡用的） | 內容 | 用途 |
|---|---|---|---|
| **場景 1：PPT 第一頁** | `project3` | 7 m × 4 m。`project.pptx` 第 1 頁的場景：長方塊、小方塊、隔牆、三根圓柱，目標在右上角 | 投影片的原始場景，比較簡單 |
| **場景 2：長牆** | `corridor`（預設） | 7 m × 4 m。兩道交錯的長牆（要蛇行，與圍牆之間的缺口寬 1.2 m）、一道只留 0.6 m 門口的牆、門後一個開口朝門的 U 形死巷；第 1 個目標在死巷正後方 | **主要的測試場地**：窄通道、門口、死巷 |
| **場景 3：論文** | `paper_fig4` | 10 m × 10 m。照論文 [*Robust Path Planning for Quadruped Robots in Dynamic and Challenging Environment*](https://ieeexplore.ieee.org/document/10865160/)（IEEE，PDF 有授權限制所以沒有放進 repo）Fig. 4 的 20 × 20 格柵格地圖建立，每格 0.5 m，16 塊靜態障礙物（含論文裡讓人工勢場法卡住的 L 形陷阱）；起點、終點與論文相同（對角線，約 15 m）。**只做了靜態障礙物**，論文裡會移動的障礙物還沒做 | 與論文同一張地圖，路線長、有陷阱區 |
| **場景 4：柱子** | `clutter` | 7 m × 4 m。6 排交錯的柱子（21 根，空隙約 0.55–0.65 m），沒有一條直線可以走 | 密集障礙物 |
| （未編號） | `obstacle_test` | 6 m × 6 m。3 個方塊 + 3 個圓柱 | 最早的測試場地 |

場地圖（障礙物與目標點的順序；場景 2、4 兩張的藍線是目前的權重走過的路徑）：場景 2 `docs/scene_corridor.png`、場景 3 `docs/scene_paper_fig4.png`、場景 4 `docs/scene_clutter.png`。

`corridor`、`clutter`、`paper_fig4` 的障礙物與目標點定義在 **`rl/scenes.py`**（Gazebo 場地檔、benchmark、2D 評估共用同一份）。
**要加新場地**：在 `rl/scenes.py` 的 `SCENES` 加一項 → `python3 tools/make_world.py` 產生 `worlds/<名稱>.world` →
`python3 tools/plot_scene.py <名稱> 圖.png` 看一下長相 → `./run.sh sim rl world:=/worlds/<名稱>.world`。
起點後方要留 0.8 m 以上（機器狗靜止時會慢慢往後滑），目標點離障礙物表面至少 0.35 m。

## 檔案說明

| 路徑 | 內容 |
|---|---|
| `launch/dogzilla_sim.launch.py` | 一鍵啟動整條管線，參數 `controller:=dwb\|rpp\|rl`、`model:=s1\|s1_box\|champ`、`gui`、`rviz`、`world`（預設 `/worlds/corridor.world`） |
| `launch/dogzilla_sim.rviz` | RViz 設定：SLAM 地圖、costmap、LiDAR、Nav2 路徑、RL 路徑 |
| `launch/scan_ground_filter.py` | LiDAR 地面點過濾節點（見下方說明，實機也適用） |
| `launch/goal_flag.py` | 目標旗子：在 RViz 用 2D Goal Pose 點到哪裡，Gazebo 裡就在那裡插一支藍色旗子（只有外觀、沒有碰撞，LiDAR 看不到） |
| `launch/cmd_vel_safety.py` | 運動安全濾波 `/cmd_vel_raw` → `/cmd_vel`（轉彎降速、限制轉向變化率，實機也適用） |
| `launch/nav2_navigation.launch.py` | Nav2 navigation_launch 的複本，只把輸出改到 `/cmd_vel_raw` |
| `config/slam.yaml` | 模擬用 SLAM Toolbox 設定（改動處皆有註解） |
| `description/make_s1_official.py` → `dogzilla_s1_official.urdf.xacro` | **官方 DOGZILLA S1 URDF** 產生器與產出的模擬模型（預設） |
| `description/dogzilla_s1.urdf.xacro` | 手工方塊模型（`model:=s1_box`，對照用） |
| `config/gait_s1.yaml` / `config/ros_control_s1.yaml` | S1 的步態參數與關節 PID |
| `description/dogzilla_sim.urdf.xacro` | 舊的 CHAMP/XGO 放大模型（`model:=champ`，見「Gazebo 崩潰修正」） |
| `worlds/corridor.world`、`clutter.world`、`paper_fig4.world` | **預設場地**（窄通道 + 死巷）、密集障礙物場地、論文 Fig. 4 的地圖，由 `tools/make_world.py` 依 `rl/scenes.py` 產生（見「場地」） |
| `worlds/project3.world` | 照 `project.pptx` 第 1 頁的場景建立（`world:=/worlds/project3.world`） |
| `worlds/obstacle_test.world` | 舊的 6 m × 6 m 圍牆場地，3 個方塊 + 3 個圓柱障礙物（`world:=/worlds/obstacle_test.world`） |
| `nav2_params.yaml` / `nav2_params_rpp.yaml` | Nav2 參數 (DWB / RPP)，尺寸、速度、加速度已依 S1 調整 |
| `rl/` | RL 避障：`nav_env.py` 2D 光達環境、`train.py` PPO 訓練、`rl_controller.py` ROS 2 節點、`scenes.py` 測試場地定義、`models/` 只訓練 10 萬步的權重 |
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

**LiDAR 放哪裡最不影響行走**：這是用模擬實測選出來的，不是算出來的。左右方向置中，高度維持官方值
（掃描平面 z = 93 mm，要高於相機外殼的 80 mm，再低前方會被機身擋住），只比較前後位置。
靜態來看各位置差異很小：走路時四隻腳在大腿關節正下方，支撐中心約在 x = 0；45 g 只佔總重 5%，LiDAR 從最前移到最後，
整機質心只移動約 6 mm（都落在支撐中心後方 1–6 mm，`python3 tools/com_check.py`）。但實際走起來差別很大，**靜態質心解釋不了，原因沒有進一步查明**。

在 Gazebo 裡對每個前後位置重產模型、實際走一遍（`tools/tune_sweep.sh`，每個位置前進 / 原地轉 / 邊走邊轉）：

| LiDAR 位置 x（base_link，+ 為前方） | 整機質心在支撐中心後方 | 前進晃動 RMS | 原地轉速率（指令 0.8） | 最大傾斜 | 後腿膝關節扭力飽和 | 結果 |
|---|---|---|---|---|---|---|
| +30 mm | 0.8 mm | — | — | 166° | — | 翻倒 |
| +4 mm | 2.1 mm | 3.1° / 19.7° | 0.77 / 0 rad/s | 67° | 6–7% | 第二次測試翻倒 |
| −16.7 mm（官方位置，上層平台最後緣） | 3.1 mm | 2.5–3.1°，一次 10.0° | 0.79–0.81 rad/s | 6°，一次 28° | 5–29% | 4 次測試有 1 次大幅前後晃（28°） |
| −40 mm | 4.2 mm | 1.7–2.5° | 0.54–0.85 rad/s | 5–11° | 6–19% | 穩 |
| −50 mm | 4.7 mm | 2.1–2.5° | 0.72–0.84 rad/s | 5–9° | 1–14% | 穩 |
| **−60 mm（採用）** | 5.2 mm | 2.2–2.8° | 0.84–0.89 rad/s | 4.9–5.3° | 0–11% | 穩，6 次測試最大傾斜都在 5.3° 內 |
| −70 mm | 5.7 mm | 2.3–2.7° | 0.86–0.92 rad/s | 5.1–5.4° | 7–30% | 穩，後腿負擔較重 |
| −86 mm（貼齊機尾） | 6.5 mm | 2.2–2.7° | 0.88–0.90 rad/s | 5.0° | 12–21% | 穩，後腿負擔較重 |
| （對照）不裝 LiDAR | 2.4 mm | 1.8–1.9° | 0.80–0.82 rad/s | 5–6° | 0–2% | 穩 |

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
  更快的速度加上轉向容易翻倒（`tools/combo_test.py`，625 g 時量的），**實機也建議遵守**。
  920 g 下穩定走 0.2 m/s 沒問題，但 0.2 m/s 的急起急停會翻倒（`tools/startstop_test.py`），所以 RL 的速度指令上限是 0.15 m/s。
- **運動安全濾波**（`launch/cmd_vel_safety.py`）：以下是 625 g 時加上它的原因；920 g 下邊走邊轉已經不會變慢也沒翻倒，
  濾波器仍保留沒改（放寬它是讓 RL 走快的方向，見「RL 避障控制器」）。當時這個步態邊轉邊走幾乎走不動（0.15 m/s 加 0.5 rad/s 時實際只前進 0.034 m/s），
  DWB 不知道，會一直送「0.2 m/s 加大角速度」並左右來回修正，機器狗原地踏步，最後在高速中反轉轉向時翻倒
  （`tools/flip_monitor.py` 錄到 0.6 s 內傾斜 9° → 85°）。濾波器讓轉彎時自動降速（`v × max(0.15, 1 − |ω|)`）、
  限制角速度變化率 1.5 rad/s²。Nav2（`launch/nav2_navigation.launch.py`，只改了輸出話題）與 RL 都輸出到 `/cmd_vel_raw`，
  經濾波後才送到 `/cmd_vel`。實機也適用。
- **Nav2**：`robot_radius` 0.13 m、膨脹半徑 0.40 m、到達容許誤差 0.15 m、加速度限制 0.5 m/s²。
- **LiDAR 地面點過濾**：掃描平面離地約 0.19 m，`margin` 0.5、IMU 歷史 0.3 s。
- **RL 的訓練環境**用的是 920 g 模型的實測速度反應（見「RL 避障控制器」）。

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

`benchmark_nav.py`：同一場地依序走完所有目標點（`paper_fig4` 5 個，其他 7 個），全部用 Gazebo 真實位置評分。
「控制器回報完成」代表 Nav2 或 RL 認為已到達；「真值到達」是實際離目標 0.35 m 內。
每段限時 `corridor` 240 s、`paper_fig4` 400 s、其他 150 s。

以下是目前的起點，不是成果，各為單次執行。強化學習用的是只訓練 10 萬步的權重（見「RL 避障控制器」）。

| 場景 | 控制器 | 控制器回報完成 | 真值到達 | 擦碰 | 翻倒 | 說明 | CSV |
|---|---|---|---|---|---|---|---|
| 場景 2 `corridor` | RL（目前的權重） | 3/7 | 3/7 | 0 | 否 | 在第一個缺口前猶豫很久才過去；**過不了 0.6 m 的門口**，門後的 3 個目標都沒到 | `results/benchmarks/bench_corridor_rl_baseline.csv` |
| 場景 4 `clutter` | RL（目前的權重） | 1/7 | 1/7 | 0 | 否 | 卡在柱子之間；到達的 1 段是不用穿過柱子的那一段 | `results/benchmarks/bench_clutter_rl_baseline.csv` |
| 場景 3 `paper_fig4` | RL（目前的權重） | 4/5 | 4/5 | 0 | 否 | 這張地圖通道較寬（1 m 以上），走得到但很慢、路線來回繞（第 1 段 15 m 的路走了 29 m、396 s）；沒過的 1 段在時限內差 0.9 m | （這次的 CSV 沒有保留） |
| 場景 2 `corridor` | Nav2 RPP | 2/7 | 0/7 | 2 | 否 | 第 1 段走了 133 s 後放棄；回報完成的 2 段實際離目標 0.8–1.2 m；SLAM 誤差累積到 1.0–1.5 m | `results/benchmarks/bench_corridor_rpp.csv` |
| 場景 1 `project3` | Nav2 RPP | 3/7 | 1/7 | 3 | 否 | 走走停停，SLAM 誤差累積到 1 m 以上 | `results/benchmarks/bench_s1_875g_rpp.csv` |
| 場景 1 `project3` | Nav2 DWB | 1/7 | 0/7 | 6 | 否 | 同上 | `results/benchmarks/bench_s1_875g_dwb.csv` |

場景 3、4 還沒有跑過 Nav2 的評測；場景 1 還沒有用目前的 RL 權重在 Gazebo 跑過。

目前的權重在 2D 訓練環境裡的到達率（`results/training/baseline_eval_2d.txt`，每個場地 200–210 段）：
場景 1 `project3` 21%、場景 2 `corridor` 10%、場景 4 `clutter` 1%、`obstacle_test` 5%、隨機場景 26%（場景 3 沒有跑），幾乎都是逾時（卡住不動或原地打轉），很少碰撞（有安全保護）。

重跑：`WORLD=corridor tools/bench_all.sh <前綴> rl rpp dwb`（依序啟動無頭模擬並跑完；`corridor` 每個控制器最多約 30 分鐘）。
`tools/path_record.py` 可以同時錄下真值路徑，`tools/plot_scene.py <場地> 圖.png 路徑.csv` 把路徑畫在場地上。

## RL 避障控制器

- 觀測：LiDAR 360 線每 10° 取最小值（36 維）+ 與路徑前方瞄準點的距離/方位 + 上一步動作 + 目前速度
- 動作：前進速度 `v ∈ [0, 0.15] m/s`、轉向 `ω ∈ [-1, 1] rad/s`，輸出端限制加速度 0.5 m/s²、2 rad/s²
- 安全保護（`rl/safety_shield.py`）：RL 想要的速度先經過碰撞檢查，會撞的動作換成最接近的安全動作；訓練與部署都開
- 架構：Nav2 `planner_server` 給全域路徑，RL 瞄準路徑前方 0.8 m 的點做局部避障
  （RL 模式下不開 Nav2 的 controller_server，避免兩邊搶 `/cmd_vel`）
- 部署推理只用 numpy（`rl_policy.py`），Raspberry Pi 上不需要裝 PyTorch

**目前附的權重（`rl/models/policy.npz`、`ppo_nav.zip`）還沒練好**：在 2D 環境從零訓練 10 萬步（約 2 分鐘）的結果。
會往目標走、有安全保護所以不撞牆，但走得不穩（前進中頻繁左右修正）、常卡在牆角、缺口或門口前原地打轉，窄的地方過不去。**把避障練好是接下來的工作。**

### 待完成的工作（只需要模擬）

目標：在場景 1–4（`project3`、`corridor`、`paper_fig4`、`clutter`），Gazebo benchmark 每一段都真值到達、沒有擦碰、沒有翻倒，而且重跑幾次結果一致。

1. **先把目前的權重跑起來看**：`./run.sh sim rl` 後在 RViz 點目標，看它在哪裡走不穩、卡在哪裡；`./run.sh bench <名稱> rl` 跑一次評測。`./run.sh sim rpp` 可以看傳統方法怎麼走。
2. **訓練出能用的策略**（`SETUP.md` 第 6 節有完整步驟）：訓練久一點（幾百萬步）、看 `eval.csv` 的到達率曲線。
   2D 環境的隨機場景目前沒有 0.6 m 的窄門口和密集柱子，可以在 `rl/nav_env.py` 的 `_random_layout` 加進去，或直接把 `rl/scenes.py` 的場地混進訓練。
3. **調整獎勵與觀測**：獎勵在 `rl/nav_env.py` 的 `step`（進度、碰撞、到達、太靠近障礙物、安全保護介入）。
   安全保護的安全距離 `rl/safety_shield.py` 的 `SAFE_DIST` 是 0.22 m，0.6 m 的門口兩邊各只剩 0.08 m 的餘量，策略要學會對準了再過。
4. **放進 Gazebo 驗證**：2D 環境裡表現好不代表 Gazebo 裡好（機身會晃、SLAM 有誤差、會翻倒），每次都要跑 benchmark。
5. **比較與紀錄**：`rl/eval_compare.py` 可以把新舊權重放在同樣的場景比較；結果放 `results/`，在 `results/README.md` 補一行說明。

### 之前幾輪訓練留下的經驗

- **速度指令上限維持 0.15 m/s**：穩定走 0.2 m/s 沒問題，但 0.2 m/s 的急起急停會翻倒（`tools/startstop_test.py`，`results/startstop_test_875g.txt`：
  0.20 時傾斜達 57–60°，0.15 時最大 10°）。2D 環境裡不會翻倒，所以上限 0.20 的策略在 2D 評估很好，放進 Gazebo 卻在障礙物旁「停—衝」幾次後翻倒。
- **2D 環境的速度模型要照 Gazebo 實測**（`rl_policy.speed_model_875g`，資料在 `results/speed_response_s1_875g_run*.txt`）：
  指令低於約 0.06 m/s 不會前進、0.15 m/s 實際約 0.084 m/s、轉彎時會被運動安全濾波降速。改了機器狗模型或濾波器，就要重量、重新擬合。
- **走得慢的主因是運動安全濾波**（`launch/cmd_vel_safety.py`）的轉彎降速：它是模型還是 625 g 時加的；920 g 下步態本身邊走邊轉已不會減速也沒翻倒
  （直接送 0.15 m/s + 0.5 rad/s 仍有 0.08 m/s、傾斜 5°）。放寬 `min_scale` / `w_full_stop`、重新量測速度反應後再訓練，有機會明顯變快，但要先確認不會翻倒。
- **「轉向抖動扣分」試過沒用**：策略學會站著不動來避免被扣分。
- **接續訓練比從零快**：`train.py --init=<舊的 ppo_nav.zip>` 會以舊權重為起點、學習率降為 1e-4。
- **Gazebo 的量測雜訊很大**：同一組設定重跑，結果可能差很多，下結論前至少跑兩次。

### 訓練環境

跑模擬只需要 `models/policy.npz`（已附上），不用另外裝任何東西。要訓練時，在主機上建 Python 環境（Python 3.10 以上）：

```bash
cd sim_env/rl
python3 -m venv .venv
env -u PYTHONPATH .venv/bin/pip install --extra-index-url https://download.pytorch.org/whl/cpu -r requirements.txt
env -u PYTHONPATH .venv/bin/python train.py 4e6 models/my_run --shield   # 訓練 4M 步 (約 45~70 分鐘), 存到 models/my_run/
env -u PYTHONPATH .venv/bin/python eval_compare.py "目前=models/policy.npz" "新=models/my_run/best/policy.npz"   # 在 2D 環境與目前的權重比較 (含 corridor / clutter / paper_fig4)
./run.sh sim rl rl_policy:=/rl/models/my_run/best/policy.npz             # (在 sim_env/ 下) 放進 Gazebo
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
| `startstop_test.py <V>` | 前進指令在 0 與 V 之間來回（急起急停）時的最大傾斜 |
| `make_world.py` / `plot_scene.py` | 在主機執行：依 `rl/scenes.py` 產生 Gazebo 場地檔 / 畫場地圖（可疊上錄到的路徑） |
| `path_record.py` | 錄真值路徑（CSV） |
| `plot_lidar_positions.py` / `render_model.py` | 在主機執行：畫 LiDAR 位置示意圖、算繪模型外觀圖（簡報用的圖） |
| `gain_sweep.sh` / `gait_sweep.sh` / `bench_all.sh` | 在主機執行：關節增益與步態掃描（官方模型）/ 步態掃描（方塊模型）/ 三種控制器依序 benchmark |

## 已知限制

- 模擬中 RL 節點與 benchmark 的位姿取自 Gazebo 真值 (`/demo/odom/ground_truth`)；實機要換成 SLAM 的 `map→base_link`。
- S1 模型的總重已改成實機秤重（本體 875 g + LiDAR 45 g），但**重量在各連桿間的分布是把官方 URDF 等比例放大**，
  電池、樹莓派的實際位置沒有量；LiDAR 位置是模擬選出的建議值。步態是 CHAMP 的，不是 Yahboom 韌體的步態，實機的速度與晃動需要重新量測。
- **靜止站立時機身會以約 1 cm/s 慢慢往後滑**（`tools/stand_drift.py`；625 g 時期就有，`results/sweeps/gain_sweep_pid.txt` 的 drift 欄）。
  與關節 PID 的微幅顫振有關（d 增益 0.03 時不會滑，但走路會晃）。導航中有閉迴路修正所以不明顯，但到達目標後停著不動會慢慢離開目標點，
  啟動後不下目標放著也會退到後方的牆邊。原地轉向時以約 5 cm/s 後退也是同一類現象，RL 的速度模型有把它算進去。
- **強化學習避障還沒做好**：附的權重只訓練了 10 萬步（見「RL 避障控制器」）。
- RL 訓練環境假設 LiDAR 在機身中心；實際在中心後方 6 cm，所以機身前方的障礙物實際上比 RL 以為的近 6 cm（碰撞判定距離 0.16 m 有留餘量）。
- 啟動時 `gazebo_ros2_control: Parameter 'hold_joints' has already been declared` 的 ERROR 是 12 個獨立
  ros2_control 區塊造成的無害訊息，不影響控制。
- `champ_gazebo/contact_sensor` 會佔滿一顆 CPU（Gazebo 每個物理步都送出接觸資料），但模擬即時率仍約 0.9。
- ROS 2 中介層改用 CycloneDDS（`docker-compose.yml` 的 `RMW_IMPLEMENTATION`）。預設的 FastDDS 在節點多時常出現服務回應逾時，
  讓 Nav2 卡在 Configuring；改用 CycloneDDS 後連續 4 次啟動都正常。launch 內仍保留 `nav2_watchdog` 作為保險。
- Docker 內 RViz 用軟體渲染，會印出 `GLSL link result` 的 ERROR，不影響顯示。
- SLAM 地圖仍有少量零星碎片、長時間運行後有輕微旋轉漂移。
