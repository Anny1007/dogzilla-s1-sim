#!/usr/bin/env bash
# 依序以 dwb / rpp / rl 啟動無頭模擬並跑 benchmark_nav.py, 結果存 logs/bench_<prefix>_<ctrl>.csv
# 用法: tools/bench_all.sh <prefix> [控制器...]     例: tools/bench_all.sh s1 dwb rl
cd "$(dirname "$0")/.."
P=${1:-run}; shift; CTRLS=${@:-dwb rpp rl}
for c in $CTRLS; do
  ./run.sh stop >/dev/null 2>&1; ./run.sh sim $c gui:=false rviz:=false >/dev/null; sleep 45
  mode=nav2; [ $c = rl ] && mode=rl
  docker exec dogzilla-sim bash -lc "source /opt/ros/humble/setup.bash; timeout 1300 python3 /benchmark_nav.py ${P}_$c $mode" 2>&1 | grep "SUMMARY"
done
./run.sh stop >/dev/null 2>&1
echo "== ALL DONE"
