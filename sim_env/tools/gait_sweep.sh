#!/usr/bin/env bash
# S1 步態參數掃描: 每組參數重啟模擬, 跑兩次 walk_test, 結果寫到 logs/gait_sweep.txt
# 會暫時改寫 config/gait_s1.yaml 與 config/ros_control_s1.yaml, 結束時還原。
cd "$(dirname "$0")/.."
cp config/gait_s1.yaml /tmp/gait_s1.orig; cp config/ros_control_s1.yaml /tmp/ros_control_s1.orig
OUT=logs/gait_sweep.txt; : > $OUT
for P in 5 10; do for H in 0.105 0.112; do for SW in 0.025 0.035; do for ST in 0.20 0.30; do
  cp /tmp/gait_s1.orig config/gait_s1.yaml; cp /tmp/ros_control_s1.orig config/ros_control_s1.yaml
  sed -i "s/swing_height : [0-9.]*/swing_height : $SW/; s/stance_duration : [0-9.]*/stance_duration : $ST/; s/nominal_height : [0-9.]*/nominal_height : $H/" config/gait_s1.yaml
  sed -i "s/{p: 5.0,/{p: $P.0,/" config/ros_control_s1.yaml
  ./run.sh stop >/dev/null 2>&1; ./run.sh sim dwb gui:=false rviz:=false >/dev/null; sleep 28
  for k in 1 2; do
    R=$(docker exec dogzilla-sim bash -lc 'source /opt/ros/humble/setup.bash; timeout 40 python3 /tools/walk_test.py' 2>&1 | grep -E "vx=0.2 |wz=0.8" | tr '\n' ' ')
    echo "p=$P h=$H swing=$SW stance=$ST try$k | $R" | tee -a $OUT
  done
done; done; done; done
./run.sh stop >/dev/null 2>&1
cp /tmp/gait_s1.orig config/gait_s1.yaml; cp /tmp/ros_control_s1.orig config/ros_control_s1.yaml
echo "== SWEEP DONE" | tee -a $OUT
