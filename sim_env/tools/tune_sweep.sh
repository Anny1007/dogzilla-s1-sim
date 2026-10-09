#!/usr/bin/env bash
# 875 g 本體 + 45 g LiDAR 的站姿 / 增益 / LiDAR 位置掃描。每組參數用 key=value 寫, 以逗號分隔, 沒寫的用預設:
#   P D (關節 PID)  H (站高 m)  SW (抬腳高度)  ST (支撐相時間)  CX (com_x_translation)  LX (LiDAR 前後位置)  M LM (本體 / LiDAR 質量)
# 每組重產模型、在空場地重啟模擬, 跑 gait_eval.py 兩次 (看重複性) 與 joint_track.py (膝關節扭力飽和比例)。
# 結果寫 logs/tune_sweep.txt; 結束後設定檔還原。   用法: tools/tune_sweep.sh P=8,H=0.105 P=12,H=0.115,LX=-0.06 ...
cd "$(dirname "$0")/.."
cp config/ros_control_s1.yaml /tmp/rc.orig; cp config/gait_s1.yaml /tmp/gait.orig
OUT=logs/${TUNE_OUT:-tune_sweep}.txt; : > $OUT
for combo in "$@"; do
  P=$(grep -o 'p: [0-9.]*' /tmp/rc.orig | head -1 | cut -d' ' -f2); D=$(grep -o 'd: [0-9.]*' /tmp/rc.orig | head -1 | cut -d' ' -f2)
  H=; SW=; ST=; CX=; LX=; M=; LM=
  for kv in $(echo $combo | tr ',' ' '); do eval "$kv"; done
  cp /tmp/gait.orig config/gait_s1.yaml; cp /tmp/rc.orig config/ros_control_s1.yaml
  [ -n "$SW" ] && sed -i "s/swing_height : [0-9.]*/swing_height : $SW/" config/gait_s1.yaml
  [ -n "$ST" ] && sed -i "s/stance_duration : [0-9.]*/stance_duration : $ST/" config/gait_s1.yaml
  [ -n "$H" ] && sed -i "s/nominal_height : [0-9.]*/nominal_height : $H/" config/gait_s1.yaml
  [ -n "$CX" ] && sed -i "s/com_x_translation: [-0-9.]*/com_x_translation: $CX/" config/gait_s1.yaml
  P=$(printf '%.4f' $P); D=$(printf '%.4f' $D)     # ROS 參數須為 double
  sed -i "s/{p: [0-9.]*, i: [0-9.]*, d: [0-9.]*,/{p: $P, i: 0.0, d: $D,/" config/ros_control_s1.yaml
  (cd description && env ${H:+S1_STAND_H=$H} ${LX:+S1_LIDAR_X=$LX} ${M:+S1_MASS=$M} ${LM:+S1_LIDAR_MASS=$LM} python3 make_s1_official.py >/dev/null)
  ./run.sh stop >/dev/null 2>&1; ./run.sh sim dwb gui:=false rviz:=false world:=/usr/share/gazebo-11/worlds/empty.world >/dev/null; sleep 32
  for k in 1 2; do
    R=$(docker exec dogzilla-sim bash -lc 'source /opt/ros/humble/setup.bash; timeout 60 python3 /tools/gait_eval.py 6' 2>&1 | grep SUMMARY)
    echo "$combo | $R" | tee -a $OUT
  done
  S=$(docker exec dogzilla-sim bash -lc 'source /opt/ros/humble/setup.bash; timeout 30 python3 /tools/joint_track.py 0.2 0 4' 2>&1 | grep lower_leg | grep -o "[0-9]*% 的" | tr -d '% 的' | tr '\n' '/')
  echo "$combo | knee_sat%=$S" | tee -a $OUT
done
./run.sh stop >/dev/null 2>&1
cp /tmp/rc.orig config/ros_control_s1.yaml; cp /tmp/gait.orig config/gait_s1.yaml; (cd description && python3 make_s1_official.py >/dev/null)
echo "== DONE" >> $OUT
