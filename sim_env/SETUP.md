# 模擬環境建置指南

照這份文件，可以在一台新電腦上從零建好 DOGZILLA S1 自主避障模擬環境，並確認它正常運作。
系統設計、參數由來與除錯紀錄請看 [`README.md`](README.md)。

整個模擬（Gazebo、SLAM、Nav2、強化學習節點）都包在 Docker 容器裡，主機只需要裝 Docker。
只有要**重新訓練強化學習模型**時，主機才需要另外建 Python 環境（第 6 節）。

---

## 1. 系統需求

| 項目 | 需求 | 說明 |
|---|---|---|
| 作業系統 | Ubuntu 22.04 以上 | 開發機為 Ubuntu 24.10；其他 Linux 發行版理論上可行，但沒有測試過 |
| Docker | Docker Engine + `docker compose`（第 2 版） | 開發機為 Docker 27.3、Compose 2.29 |
| CPU / 記憶體 | 4 核心、8 GB 以上 | 開發機為 20 核心、32 GB；核心少時模擬會變慢 |
| 硬碟 | 約 10 GB 可用空間 | Docker 映像約 4.4 GB |
| 顯示卡 | 不需要 | 容器內用軟體渲染，沒有使用 GPU |
| 桌面 | 要看 Gazebo / RViz 畫面才需要 | X11 或 Wayland 都可以（Wayland 透過 XWayland）；沒有桌面可以用無頭模式 |
| 網路 | 第一次建映像時需要 | 下載 ROS 2 映像與套件 |

---

## 2. 安裝 Docker

已經裝好的話跳到第 3 節。

```bash
# 官方安裝腳本 (含 docker compose 外掛)
curl -fsSL https://get.docker.com | sudo sh

# 讓目前使用者不用 sudo 就能執行 docker, 執行後要登出再登入才會生效
sudo usermod -aG docker $USER
```

確認安裝成功：

```bash
docker --version
docker compose version      # 要能顯示版本; 舊版的 docker-compose (有連字號) 不適用
docker run --rm hello-world
```

---

## 3. 取得專案

把整個 `DOGZILLA/` 資料夾複製到主機上，路徑沒有限制。
`sim_env/docker-compose.yml` 會用相對路徑掛載 `../vendor/Program/yahboomcar_ws_ros2`，
所以 **`sim_env/` 與 `vendor/` 必須維持原本的相對位置**。

```
DOGZILLA/
├── sim_env/     ← 以下所有指令都在這裡執行
└── vendor/      ← Yahboom 原廠程式 (CHAMP 步態、官方機器狗模型)
```

---

## 4. 建置 (第一次才需要)

```bash
cd DOGZILLA/sim_env
./run.sh build      # 1. 建 Docker 映像 dogzilla-sim:humble (約 10~20 分鐘)
./run.sh colcon     # 2. 編譯 CHAMP 步態套件 (約 3 分鐘)
```

**`./run.sh build`** 以 `osrf/ros:humble-desktop-full` 為基礎，加裝 Gazebo ROS 套件、ros2_control、Nav2、
SLAM Toolbox、CycloneDDS 與 DOGZILLA 底層函式庫（`DOGZILLALib`）。完成後可以確認：

```bash
docker images dogzilla-sim     # 應該看到 dogzilla-sim  humble
```

**`./run.sh colcon`** 在容器內編譯 `vendor/Program/yahboomcar_ws_ros2/src/champ`，
產物放在 `vendor/Program/yahboomcar_ws_ros2/` 底下的 `build/`、`install/`、`log/`（留在主機上，之後不用重編）。
完成後可以確認：

```bash
ls ../vendor/Program/yahboomcar_ws_ros2/install    # 應該有 champ、champ_gazebo、champ_config 等資料夾
```

---

## 5. 啟動與驗證

### 5.1 啟動

```bash
./run.sh sim        # Gazebo + SLAM + Nav2 (DWB 控制器) + RViz
```

指令會立刻回到命令列，模擬在背景容器 `dogzilla-sim` 裡執行。約 30 秒後會出現 **Gazebo**（3D 場景）與 **RViz**（地圖、光達、路徑）兩個視窗。
預設場地是 `project.pptx` 第 1 頁的障礙物場景。

### 5.2 確認啟動成功

看啟動日誌 `logs/sim.log`，以下兩行都出現就代表正常：

```bash
grep -E "unpause|nav2_watchdog" logs/sim.log
# [unpause] 開始物理模擬, active 控制器數 = 2 (應為 2)      ← 機器狗的關節控制器啟動了
# [nav2_watchdog] Nav2 active                               ← 導航系統啟動了 (約 35 秒後出現)
```

想再確認光達與地圖有資料：

```bash
docker exec dogzilla-sim bash -lc 'source /opt/ros/humble/setup.bash;
  timeout 5 ros2 topic hz /scan_filtered;                        # 光達, 應約 10 Hz
  timeout 10 ros2 topic echo /map --once --field info.width'     # SLAM 地圖, 應印出一個數字
```

### 5.3 下目標點

1. 在 RViz 上方工具列點 **2D Goal Pose**
2. 在地圖上點一下（按住拖曳可以設定朝向）
3. 機器狗會自己規劃路徑、繞過障礙物走過去。右上角的綠色柱子是投影片的目標點

### 5.4 其他啟動方式

```bash
./run.sh sim rpp                                       # 改用 Nav2 Regulated Pure Pursuit 控制器
./run.sh sim rl                                        # 改用強化學習避障 (預設權重 rl/models/policy.npz)
./run.sh sim rl rl_policy:=/rl/models/s1_v2/policy.npz # 強化學習, 指定其他權重
./run.sh sim dwb gui:=false rviz:=false                # 無頭模式 (沒有桌面、或跑長時間測試時用)
./run.sh sim dwb world:=/worlds/obstacle_test.world    # 舊的 6 m x 6 m 場地
```

### 5.5 停止與其他指令

```bash
./run.sh stop                    # 停止模擬 (換控制器或換場地前要先停)
./run.sh shell                   # 進到正在執行的模擬容器裡
./run.sh bench test1 dwb         # 自動跑 7 個目標點, 結果存到 logs/bench_test1.csv (約 15~20 分鐘)
./run.sh bench test1 rl          # 強化學習模式要用 ./run.sh sim rl 啟動後再跑
```

---

## 6. 強化學習訓練環境 (選用)

只有要**重新訓練**或**評估**強化學習模型時才需要。訓練用的是純 Python 的 2D 光達環境（`rl/nav_env.py`），
只用 CPU，不需要 ROS 或 Gazebo，直接在主機上執行。

### 6.1 建立 Python 環境

需要 Python 3.10 以上。

```bash
cd sim_env/rl
python3 -m venv .venv
env -u PYTHONPATH .venv/bin/pip install -r requirements.txt --extra-index-url https://download.pytorch.org/whl/cpu
```

> **`env -u PYTHONPATH`**：如果主機裝過 ROS，`PYTHONPATH` 會指向系統的 Python 套件，讓虛擬環境載入錯誤版本的套件而報錯。
> 以下所有 `.venv/bin/python` 指令前面都要加。

### 6.2 訓練

```bash
env -u PYTHONPATH .venv/bin/python train.py 4e6 models/my_run --shield   # 訓練 400 萬步 (約 70 分鐘), 存到 models/my_run/
```

| 參數 | 說明 |
|---|---|
| 第 1 個 | 總訓練步數，預設 `2e6` |
| 第 2 個 | 輸出資料夾，預設 `models`。**不指定會覆蓋部署用的 `models/policy.npz`** |
| `--shield` | 訓練時加上安全保護（`rl/safety_shield.py`）。權重檔會記錄，部署時自動開啟 |

訓練環境目前（v5）使用 Gazebo 實測的速度模型（`rl/rl_policy.py` 的 `speed_model_measured`），動作範圍 0～0.15 m/s。
速度模型與動作範圍都會寫進權重檔，部署節點會照著用，所以舊權重（v1～v3）仍照原本的設定執行。

輸出資料夾裡會有：

| 檔案 | 內容 |
|---|---|
| `policy.npz` | 部署用權重（只需要 numpy，Gazebo 與實機都用這個） |
| `ppo_nav.zip` | 完整的 stable-baselines3 模型 |
| `progress.csv` | 每次更新的 loss、entropy、KL、平均回報等訓練紀錄 |
| `eval.csv` | 每 10 萬步在 200 個固定場景的到達 / 碰撞 / 逾時次數 |
| `best/` | 訓練過程中評估到達數最高的模型（`policy.npz` + `ppo_nav.zip`），通常比最後一步好 |

### 6.3 評估與畫圖

```bash
# 在 2D 環境比較多個模型 (舊測試場地、投影片場地、隨機場景各一組)
env -u PYTHONPATH .venv/bin/python eval_compare.py "v1=models/policy.npz" "新=models/my_run/best/policy.npz"
# 路徑後面可加 +shield (強制開安全保護)、+measured (改用實測速度模型), 例如 "v3=models/s1_v3_shield/best/policy.npz+measured"

# 畫訓練曲線 (用模擬映像裡的 matplotlib, 主機不用另外安裝)
cd ..
docker run --rm -v "$PWD":/w -v /usr/share/fonts/opentype/noto:/fonts:ro -w /w dogzilla-sim:humble \
  python3 tools/plot_ppo_metrics.py rl/models/my_run results/training/my_run.png
```

畫圖的中文字型取自主機的 `/usr/share/fonts/opentype/noto`（Ubuntu 套件 `fonts-noto-cjk`）。沒有這個字型時，中文會顯示成方框。

### 6.4 放進 Gazebo 測試

```bash
./run.sh sim rl rl_policy:=/rl/models/my_run/best/policy.npz
./run.sh bench my_run rl
```

部署節點（`rl/rl_controller.py`）可用的啟動參數：

| 參數 | 預設 | 說明 |
|---|---|---|
| `rl_policy` | `/rl/models/policy.npz` | 權重檔 |
| `rl_shield` | `auto` | 安全保護：`auto`（依權重檔）、`on`、`off` |
| `rl_vel_obs` | `model` | 觀測裡的目前速度：`model`（由指令推算，與訓練一致）、`odom`（腿部里程計，會隨步伐擺動） |

部署節點每 2 秒向 Nav2 重新規劃一次路徑，並發佈 `/rl_intent`（安全保護之前 RL 想要的速度），方便診斷。
S1 在 Gazebo 的實際速度反應可以用 `tools/speed_response.py` 量測（需以空場地啟動，見檔頭說明）。

確認比較好之後，再把 `policy.npz` 複製到 `rl/models/policy.npz` 成為預設權重（建議先備份舊的）。

---

## 7. 常見問題

| 狀況 | 原因與解法 |
|---|---|
| `permission denied ... docker.sock` | 使用者不在 docker 群組：`sudo usermod -aG docker $USER`，登出再登入 |
| `./run.sh sim` 顯示「模擬已在執行」 | 同一時間只能跑一個模擬（容器使用主機網路，Gazebo 埠與 ROS 話題會互相干擾）。先 `./run.sh stop` |
| 沒有出現 Gazebo / RViz 視窗 | 1. 確認有桌面且 `echo $DISPLAY` 有值（例如 `:0`）；2. 手動執行 `xhost +local:docker` 後重新啟動；3. 透過 SSH 連線時沒有畫面，改用無頭模式 |
| `logs/sim.log` 沒有出現 `Nav2 active` | 啟動時會自動重試一次。仍然失敗就 `./run.sh stop` 後重新啟動 |
| 啟動後機器狗趴在地上 | 關節控制器沒有啟動。檢查 `logs/sim.log` 的 `active 控制器數` 是不是 2，不是就重新啟動 |
| `./run.sh colcon` 失敗 | 確認 `vendor/` 與 `sim_env/` 的相對位置沒有變（第 3 節），且第 4 節的映像已經建好 |
| 日誌裡有 `GLSL link result`、`hold_joints has already been declared` 等 ERROR | 已知的無害訊息，不影響執行（見 `README.md`「已知限制」） |
| 模擬很慢、機器狗動作卡頓 | CPU 不夠時 Gazebo 會低於即時速度。改用無頭模式（`gui:=false rviz:=false`）可以省下不少資源 |
| 訓練時出現 `cv2` 或套件版本錯誤 | 指令前面漏了 `env -u PYTHONPATH`（第 6.1 節） |

---

## 8. 建置檢查清單

- [ ] `docker compose version` 有顯示版本
- [ ] `docker images dogzilla-sim` 看得到 `humble` 映像
- [ ] `vendor/Program/yahboomcar_ws_ros2/install/` 裡有 `champ` 等資料夾
- [ ] `./run.sh sim` 後，`logs/sim.log` 有 `active 控制器數 = 2` 與 `Nav2 active`
- [ ] 在 RViz 用 2D Goal Pose 下目標，機器狗會走過去
- [ ] `./run.sh stop` 能正常停止
- [ ] （選用）`rl/.venv` 建好，`eval_compare.py` 能跑出結果
