#!/bin/bash
# 무게를 바꿔 가며 여유 시험(margin_test.py)을 자동으로 반복 → 최대 가능 무게 찾기
#
# 사용 (터미널 하나, 시뮬은 꺼진 상태에서. 무게 하나당 약 2분)
#   bash tools/weight_sweep.sh 결과폴더 모터 무게1 무게2 ...
#   예) bash tools/weight_sweep.sh 261121_sweep_air2216     air2216     2200 2400 2600 2800 3000 3200 3400
#       bash tools/weight_sweep.sh 261121_sweep_air2216_low air2216_low 2200 2400 2600 2800 3000
#   무게중심 바꾸려면 앞에 CG=-17.7 처럼 붙임. Gazebo 창 없이 빠르게: 앞에 HEADLESS=1
#
# 결과: results/결과폴더/무게g/ (summary.txt, hover.png) + results/결과폴더/sweep.txt, sweep.png (표·그래프)
# 끝나면 모델이 마지막 무게로 남아 있음 → 원래 설정은 set_mass_cg.py로 다시 지정

set -u
TAG=$1; MOTOR=$2; shift 2
CG=${CG:--14}
REPO=~/parcel-drone-sim
cd "$REPO" || exit 1
mkdir -p "results/$TAG"

for W in "$@"; do
  echo "================ ${W} g (모터 ${MOTOR}, 무게중심 ${CG} mm) ================"
  pkill -9 -f "px4_sitl_default/bin/px4" 2>/dev/null; pkill -9 -f "gz sim" 2>/dev/null; sleep 2
  python3 tools/vehicle/set_mass_cg.py "$W" "$CG" --motor "$MOTOR" > "results/$TAG/setting_${W}g.txt" || exit 1
  PX4_ARGS=-d bash tools/run_mydrone_flow_test.sh > "/tmp/sweep_px4_${W}.log" 2>&1 &
  SIM=$!
  sleep 20                                    # Gazebo·PX4 켜지는 시간
  timeout 400 python3 scripts/margin_test.py || echo "  !! ${W} g 비행 코드가 정상 종료되지 않음"
  sleep 2
  pkill -INT -f "px4_sitl_default/bin/px4"    # pxh> shutdown 과 같음 (로그 저장)
  wait "$SIM" 2>/dev/null; sleep 3
  python3 tools/analysis/analyze_hover.py "$TAG/${W}g" --margin | grep -E "호버 추력 [0-9]|모터 출력 최대|기울기 최대|최고 고도"
done

python3 tools/analysis/sweep_table.py "results/$TAG"
