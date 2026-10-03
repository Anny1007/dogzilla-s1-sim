#!/usr/bin/env bash
# S1 官方模型關節增益/步態掃描: 每組 p,d,關節阻尼,控制頻率[,抬腳高度,支撐相時間] 重產模型並重啟,
# 跑 gait_eval.py, 結果寫 logs/gain_sweep.txt (用法: tools/gain_sweep.sh 8,0.005,0.01,1000,0.02,0.3 ...)
cd "$(dirname "$0")/.."
cp config/ros_control_s1.yaml /tmp/rc.orig; cp config/gait_s1.yaml /tmp/gait.orig
OUT=logs/gain_sweep.txt; : > $OUT
for combo in "$@"; do
  read P D DAMP HZ SW ST <<< "$(echo $combo | tr ',' ' ')"
  cp /tmp/gait.orig config/gait_s1.yaml
  [ -n "$SW" ] && sed -i "s/swing_height : [0-9.]*/swing_height : $SW/; s/stance_duration : [0-9.]*/stance_duration : $ST/" config/gait_s1.yaml
  P=$(printf '%.4f' $P); D=$(printf '%.4f' $D)     # ROS 參數須為 double, 寫成 10 會讓控制器初始化失敗
  cp /tmp/rc.orig config/ros_control_s1.yaml
  sed -i "s/{p: [0-9.]*, i: [0-9.]*, d: [0-9.]*,/{p: $P, i: 0.0, d: $D,/; s/update_rate: [0-9]*/update_rate: $HZ/" config/ros_control_s1.yaml
  (cd description && S1_JOINT_DAMPING=$DAMP python3 make_s1_official.py >/dev/null)
  ./run.sh stop >/dev/null 2>&1; ./run.sh sim dwb gui:=false rviz:=false >/dev/null; sleep 32
  R=$(docker exec dogzilla-sim bash -lc 'source /opt/ros/humble/setup.bash; timeout 60 python3 /tools/gait_eval.py 5' 2>&1 | grep SUMMARY)
  S=$(docker exec dogzilla-sim bash -lc 'source /opt/ros/humble/setup.bash; timeout 30 python3 /tools/joint_track.py 0.2 0 4' 2>&1 | grep lower_leg | grep -o "[0-9]*% 的" | tr -d '% 的' | tr '\n' '/')
  echo "p=$P d=$D damp=$DAMP hz=$HZ swing=${SW:-預設} stance=${ST:-預設} | $R knee_sat%=$S" | tee -a $OUT
done
./run.sh stop >/dev/null 2>&1
cp /tmp/rc.orig config/ros_control_s1.yaml; cp /tmp/gait.orig config/gait_s1.yaml; (cd description && python3 make_s1_official.py >/dev/null)
echo "== DONE" >> $OUT
