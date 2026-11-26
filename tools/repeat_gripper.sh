#!/usr/bin/env bash
# 그리퍼 미션 자동 반복: 시뮬 켜기 → mission_gripper.py → shutdown → 분석 → 다음 회차
#
# 사용 (터미널 하나에서, 다른 시뮬은 모두 끈 상태로)
#   bash tools/repeat_gripper.sh 3 고정          # 3회 반복, 결과 이름 고정1, 고정2, 고정3
#   bash tools/repeat_gripper.sh 3 고정 --turn   # 옛 방식(기수 회전)으로 비교
# 결과
#   results/고정1, 고정1_route ... (회차별 사진·기록·경로 그림)
#   results/고정_반복요약.txt      (회차별 H 착륙 오차 + 평균)
# 중간에 멈추려면 Ctrl+C
set -u
N=${1:-3}
NAME=${2:-반복}
EXTRA=${3:-}
REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOGROOT=~/PX4-Autopilot/build/px4_sitl_default/rootfs/log
SUM="$REPO/results/${NAME}_반복요약.txt"
mkdir -p "$REPO/results"
echo "그리퍼 미션 반복 $N회 ($(date '+%m/%d %H:%M')) $EXTRA" > "$SUM"

cleanup() {
  pkill -INT -f "px4_sitl_default/bin/px4" 2>/dev/null; sleep 3
  pkill -9 -f "px4_sitl_default/bin/px4" 2>/dev/null
  pkill -9 -f "gz sim" 2>/dev/null; sleep 2
}
trap 'cleanup; exit 1' INT

for i in $(seq 1 "$N"); do
  TAG="${NAME}${i}"
  echo; echo "========== $i / $N 회차 ($TAG) =========="
  cleanup
  rm -f /tmp/pxh_in; mkfifo /tmp/pxh_in
  exec 3<>/tmp/pxh_in
  # 시뮬 켜기 (pxh> 입력은 /tmp/pxh_in 으로 보냄, 화면 출력은 sim.log)
  bash "$REPO/tools/run_mydrone_flow_test.sh" < /tmp/pxh_in > "/tmp/sim_${TAG}.log" 2>&1 &
  SIM_PID=$!
  echo "시뮬 켜는 중 (25초)..."
  sleep 25

  # 미션
  ( cd "$REPO" && python3 scripts/mission_gripper.py "$TAG" $EXTRA )
  sleep 3

  # shutdown
  echo "shutdown" >&3
  for k in $(seq 1 20); do kill -0 $SIM_PID 2>/dev/null || break; sleep 1; done
  exec 3>&-
  cleanup
  sleep 2

  # 방금 비행 로그로 분석
  ULG=$(ls -t "$LOGROOT"/*/*.ulg | head -1)
  ( cd "$REPO" && python3 tools/analysis/analyze_mission_log.py "${TAG}_route" "$ULG" > /dev/null )
  # H 착륙 오차 = 마지막 실제 위치 (Gazebo 정답, 원점 = H)
  python3 - "$ULG" "$TAG" "$SUM" "$REPO" <<'EOF'
import sys, math
from pyulog import ULog
ulg, tag, summ, repo = sys.argv[1:5]
sys.path.insert(0, repo + "/tools")
import auditorium_config as C
hn, he = 0.0, 0.0                            # Gazebo 정답 위치의 원점 = 월드 원점 = H
u = ULog(ulg, ["vehicle_local_position_groundtruth"])
g = u.data_list[0].data
n, e = (g["x"][-1] - hn) * 100, (g["y"][-1] - he) * 100
line = f"{tag}: H 착륙 오차 {math.hypot(n, e):.0f} cm (북 {n:+.0f}, 동 {e:+.0f})   로그 {ulg.split('/log/')[-1]}"
print("\n>>> " + line)
open(summ, "a", encoding="utf-8").write(line + "\n")
EOF
done

python3 - "$SUM" <<'EOF'
import sys, re
p = sys.argv[1]
s = open(p, encoding="utf-8").read()
v = [tuple(map(float, x)) for x in re.findall(r"오차 (\d+) cm \(북 ([+-]\d+), 동 ([+-]\d+)\)", s)]
if v:
    k = len(v)
    s += (f"\n평균: {sum(a for a, _, _ in v)/k:.0f} cm (북 {sum(b for _, b, _ in v)/k:+.0f}, "
          f"동 {sum(c for _, _, c in v)/k:+.0f}), 최대 {max(a for a, _, _ in v):.0f} cm\n")
open(p, "w", encoding="utf-8").write(s)
print("\n" + s)
EOF
echo "요약 파일: $SUM"
