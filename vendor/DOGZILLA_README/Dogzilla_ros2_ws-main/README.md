# Dogzilla ROS 2 Workspace (move_control)

本專案為 Yahboom Dogzilla 機器狗的 ROS 2 控制工作區。
核心套件為 `move_control`，主要將底層的硬體控制 API (`DOGZILLALib`) 封裝成標準的 ROS 2 節點。開發者無需修改底層程式碼，即可透過標準的 ROS 2 主題 (Topic) 直接控制機器狗的移動。

## 系統環境
* **作業系統:** Ubuntu 20.04 
* **ROS 2 版本:** Foxy
* **程式語言:** Python 3
* **硬體依賴:** 需確保系統已正確安裝 Yahboom 的 `DOGZILLALib` 函式庫。

## 節點說明 (Nodes Overview)

### 1. `driver_node` (核心硬體驅動節點)
此節點負責與機器狗的硬體直接溝通。它會在背景持續運行，監聽速度控制指令，並將其轉換為機器狗的實際動作（前進、後退、平移、旋轉）。

#### 訂閱主題 (Subscribed Topics)
* **`/cmd_vel`** (`geometry_msgs/msg/Twist`)
  用戶只需向此主題發佈 (Publish) Twist 訊息，即可控制機器狗。對應的動作邏輯如下：
  * `linear.x > 0`: 前進
  * `linear.x < 0`: 後退
  * `linear.y > 0`: 向左平移
  * `linear.y < 0`: 向右平移
  * `angular.z > 0`: 原地左轉
  * `angular.z < 0`: 原地右轉
  * 所有數值皆為 `0`: 停止動作

*(注意：目前硬體驅動端預設的作動參數值固定為 `10`，只要判斷方向大於或小於 0 即觸發動作。)*

---

## 編譯與安裝指南
如果第一次下載或是工作區沒有`build log install`等資料夾時

在專案根目錄下執行:
```bash 
colcon build
```
編譯成功後執行:
```bash
source install/setup.bash
```

## 使用說明
啟動 driver_node 
```bash 
ros2 run move_control driver_node 
```
使用者即可使用ros2與Dogzilla 溝通與控制