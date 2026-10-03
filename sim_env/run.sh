#!/usr/bin/env bash
# 用法:
#   ./run.sh build                 建 Docker 映像 (第一次)
#   ./run.sh colcon                編譯 CHAMP 工作區 (第一次)
#   ./run.sh sim [dwb|rpp|rl] [launch 參數...]   一鍵啟動完整模擬 (Gazebo + SLAM + Nav2 + 控制器 + RViz)
#                                  例: ./run.sh sim rl      ./run.sh sim dwb gui:=false rviz:=false
#   ./run.sh bench <label> [dwb|rpp|rl] [project3|obstacle_test]
#                                  在已啟動的模擬上跑避障 benchmark, 結果存 logs/bench_<label>.csv
#                                  (場地要與啟動時一致; 預設 project3 = project.pptx 第 1 頁場景)
#   ./run.sh shell                 另開一個 shell 進到正在跑的模擬容器 (沒在跑就開新容器)
#   ./run.sh stop                  停止模擬
#
# 注意: network_mode 是 host, 同一時間只能有一個模擬容器, 否則 Gazebo 埠 11345 與 ROS 話題會互相干擾。
cd "$(dirname "$0")"
NAME=dogzilla-sim
running() { docker ps --format '{{.Names}}' | grep -qx $NAME; }
case "$1" in
  build) docker compose build ;;
  colcon) docker compose run --rm dogzilla bash -lc 'cd /ws && rosdep install --from-paths src/champ --ignore-src -r -y --skip-keys "ros2_controllers velodyne_gazebo_plugins" ; colcon build --symlink-install --base-paths src/champ' ;;
  sim)
    shift; CTRL=${1:-dwb}; [ $# -gt 0 ] && shift
    running && { echo "模擬已在執行 (./run.sh stop 先停止)"; exit 1; }
    xhost +local:docker >/dev/null 2>&1
    docker compose run -d --rm --name $NAME dogzilla bash -lc \
      "source /opt/ros/humble/setup.bash; source /ws/install/setup.bash; ros2 launch /launch/dogzilla_sim.launch.py controller:=$CTRL $* 2>&1 | tee /logs/sim.log" >/dev/null
    echo "模擬啟動中 (controller=$CTRL), 約 30 秒後可在 RViz 用「2D Goal Pose」下目標點。日誌: logs/sim.log" ;;
  bench)
    running || { echo "請先 ./run.sh sim ..."; exit 1; }
    MODE=nav2; [ "$3" = rl ] && MODE=rl
    docker exec -it $NAME bash -lc "source /opt/ros/humble/setup.bash; python3 /benchmark_nav.py ${2:-run} $MODE ${4:-project3}" ;;
  shell)
    xhost +local:docker >/dev/null 2>&1
    if running; then docker exec -it $NAME bash; else docker compose run --rm dogzilla bash; fi ;;
  stop) docker rm -f $NAME >/dev/null 2>&1 && echo "已停止" ;;
  *) sed -n 2,13p "$0" ;;
esac
