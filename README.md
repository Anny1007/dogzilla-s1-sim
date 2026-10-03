# DOGZILLA S1 自主避障專案（Project 3）

讓 Yahboom DOGZILLA S1 四足機器狗加裝 LiDAR 後，**自己建地圖、規劃路線、繞過障礙物走到目標點**。
目前以 Gazebo 模擬完成整條系統：LiDAR → SLAM → Nav2 路徑規劃 → 避障（傳統演算法 / 強化學習）→ 步態控制。

- 第一次接觸：先看 [`docs/專案說明.md`](docs/專案說明.md)（白話介紹、成果、限制）或簡報 `docs/DOGZILLA_專案簡報.pptx`
- 在新電腦上建置環境：看 [`sim_env/SETUP.md`](sim_env/SETUP.md)（從安裝 Docker 到確認模擬正常的逐步指南）
- 要執行或修改模擬：看 [`sim_env/README.md`](sim_env/README.md)（完整技術文件）

## 資料夾結構

```
DOGZILLA/
├── README.md                 ← 本檔
├── docs/                     專案文件
│   ├── project.pptx          專案投影片（目標、硬體、系統架構）
│   ├── architecture.svg      軟體架構圖
│   ├── DOGZILLA_專案簡報.pptx  成果報告簡報（12 頁，含講者備註；另附 .pdf 版）
│   └── 專案說明.md           給新成員 / 對外說明用的介紹
├── sim_env/                  模擬環境（本專案開發的部分）
│   ├── README.md             技術文件：使用方式、設計、遇到的問題與解法、測試結果
│   ├── SETUP.md              環境建置指南：安裝、建置、驗證、強化學習訓練環境、常見問題
│   ├── run.sh                所有操作的入口（build / colcon / sim / bench / shell / stop）
│   ├── Dockerfile, docker-compose.yml
│   ├── launch/               一鍵啟動檔、LiDAR 地面點過濾、運動安全濾波、RViz 設定
│   ├── description/          機器狗模型（由官方 URDF 自動產生）
│   ├── config/               步態、關節控制器、SLAM 設定
│   ├── worlds/               模擬場地（project3 = 投影片第 1 頁場景）
│   ├── rl/                   強化學習避障（訓練程式與已訓練好的權重）
│   ├── tools/                診斷與測試工具
│   ├── results/              已記錄的測試結果（有索引說明）
│   ├── logs/, maps/          執行時輸出（一開始是空的）
│   └── benchmark_nav.py, nav2_params*.yaml, DOGZILLALib/
└── vendor/                   Yahboom 原廠資料（未修改）
    ├── Program/              原廠程式：yahboomcar_ws_ros2（CHAMP 步態、官方 URDF）、DOGZILLA（Python 函式庫與範例）
    ├── DOGZILLA_README/      實機 ROS 2 控制工作區 move_control（driver_node）、MATLAB 範例
    ├── Tutorial/             DOGZILLA 操作教學投影片
    ├── 狗吉拉README.pptx     實機說明投影片（vendor/DOGZILLA_README/ 裡另有一份不同版本）
    └── Photo/, Video/        實機照片與影片
```

## 快速開始

需要：Ubuntu（建議 22.04 或 24.04）、Docker 與 `docker compose`、有桌面才看得到 Gazebo / RViz 畫面（沒有也可以無頭執行）。

```bash
cd sim_env
./run.sh build      # 第一次：建 Docker 映像（約 10~20 分鐘，需要網路）
./run.sh colcon     # 第一次：編譯 CHAMP 步態套件（約 3 分鐘）
./run.sh sim        # 啟動模擬；約 30 秒後在 RViz 用「2D Goal Pose」點目標，機器狗就會自己走過去
./run.sh stop       # 停止
```

詳細步驟、驗證方法與常見問題見 `sim_env/SETUP.md`；其他模式（強化學習避障、RPP、舊場地、benchmark）見 `sim_env/README.md`。

## 目前狀態（2026-10）

| 項目 | 狀態 |
|---|---|
| 整條軟體管線（LiDAR、SLAM、Nav2、RL、步態控制） | 模擬中完成，可一鍵啟動 |
| 機器狗模型 | 使用 Yahboom 官方 DOGZILLA S1 URDF（為模擬做的修正都有記錄） |
| 走路晃動 | 已改善：前進時機身晃動 RMS 7.0° → 1.8° |
| 投影片主要任務（繞過障礙物走到右上角目標） | 模擬中完成（RPP，最終誤差約 4 cm） |
| 連續 7 個目標點的到達率 | 仍偏低（各方法 0–3/7），主因是走得慢與 SLAM 誤差累積 |
| 實機部署 | 尚未進行 |

詳細數字與分析見 `sim_env/README.md` 的「Benchmark 結果」與 `sim_env/results/README.md`。

## 交付說明

為了讓資料夾容易傳遞，以下**可重建的檔案沒有附上**：

| 未附上 | 重建方式 |
|---|---|
| Docker 映像 `dogzilla-sim:humble` | `cd sim_env && ./run.sh build` |
| CHAMP 編譯產物（`vendor/Program/yahboomcar_ws_ros2/build`、`install`、`log`） | `cd sim_env && ./run.sh colcon` |
| RL 訓練用的 Python 環境（`sim_env/rl/.venv`，約 1.1 GB） | 只有重新訓練時才需要，見 `sim_env/README.md` 的「RL 避障控制器」；套件清單在 `sim_env/rl/requirements.txt` |

`vendor/` 裡的原廠資料都保留原樣（只刪除了 macOS 的 `._*`、`.DS_Store` 與 Jupyter 的自動存檔），
本專案的修改全部在 `sim_env/`。
