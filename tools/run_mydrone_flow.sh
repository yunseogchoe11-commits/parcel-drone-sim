#!/bin/bash
# 우리 기체(x500_mydrone) + 광류·거리계, GPS 끔 (기체 번호 4021)
# 사용: bash tools/run_mydrone_flow.sh            (강당 월드)
#       bash tools/run_mydrone_flow.sh default    (다른 월드)
#
# PX4 센서 위치 설정 (FC가 쓰는 값)
#   기준 = 무게중심, 방향 = 앞 +, 오른쪽 +, 아래 + (m)
#   무게중심 (SolidWorks, 앞/왼쪽/위 mm): (-17.7, 0, -49.6)
#   광류·거리계: (80, 0, -20)  -> 무게중심 기준 앞 97.7 mm, 위 29.6 mm
#   IMU(FC):     (0, 0, 36)    -> 무게중심 기준 앞 17.7 mm, 위 85.6 mm (FC 높이 추정 포함)
#   무게중심이나 센서 위치가 바뀌면 아래 숫자를 다시 계산한다.

export PX4_PARAM_EKF2_OF_POS_X=0.098     # 광류: 앞 9.8 cm
export PX4_PARAM_EKF2_OF_POS_Y=0.0
export PX4_PARAM_EKF2_OF_POS_Z=-0.030    # 광류: 위 3.0 cm
export PX4_PARAM_EKF2_RNG_POS_X=0.098    # 거리계: 앞 9.8 cm
export PX4_PARAM_EKF2_RNG_POS_Y=0.0
export PX4_PARAM_EKF2_RNG_POS_Z=-0.030   # 거리계: 위 3.0 cm
export PX4_PARAM_EKF2_IMU_POS_X=0.018    # IMU: 앞 1.8 cm
export PX4_PARAM_EKF2_IMU_POS_Y=0.0
export PX4_PARAM_EKF2_IMU_POS_Z=-0.086   # IMU: 위 8.6 cm
export PX4_PARAM_EKF2_MIN_RNG=0.22       # 땅에 있을 때 거리계 값 (Gazebo 기본 다리 기준, m)
export PX4_PARAM_EKF2_MAG_TYPE=6        # 지자기: 처음 방향만 사용 (10/31 미션 비교 결과. 기본값 '자동'은 이륙 순간 기수가 튀어 경로가 휘고 추락)
export PX4_PARAM_SENS_FLOW_MINHGT=0.5     # 광류: 0.5 m 아래에서는 안 씀 (11/2 시험 3. 바닥·저고도의 흐릿한 광류가 기수를 8°씩 튀게 해서 이륙 때 왼쪽으로 돌던 원인)
export PX4_PARAM_EKF2_HEAD_NOISE=0.05    # 나침반 방향을 강하게 믿음 (11/2 시험 5. 기본 0.3이면 광류가 켜질 때 기수가 3~11° 끌려감)
export PX4_PARAM_EKF2_MAG_NOISE=0.01     # 나침반 3축 잡음 (기본 0.05)
export PX4_PARAM_EKF2_MAG_DECL=-2.5      # 자북 편차: 시뮬레이션에서 로그로 측정한 값 (실기체는 따로 확인)

# 무게중심 보정 (10/13 실험 E 값, results/261013_hovering)
#   모터 위치를 무게중심 기준으로 입력 (앞 +, 오른쪽 +, m). 무게중심이 뒤 17.7 mm -> 앞 모터 0.174+0.018, 뒤 모터 -0.174+0.018
#   THR_MDL_FAC 0.75 = Gazebo 모터의 출력-추력 곡선 (이 값에서 피치 토크 명령 ≈ 0). 실기체 값은 따로 측정 필요
export PX4_PARAM_CA_ROTOR0_PX=0.192      # 앞 오른쪽
export PX4_PARAM_CA_ROTOR0_PY=0.174
export PX4_PARAM_CA_ROTOR1_PX=-0.156     # 뒤 왼쪽
export PX4_PARAM_CA_ROTOR1_PY=-0.174
export PX4_PARAM_CA_ROTOR2_PX=0.192      # 앞 왼쪽
export PX4_PARAM_CA_ROTOR2_PY=-0.174
export PX4_PARAM_CA_ROTOR3_PX=-0.156     # 뒤 오른쪽
export PX4_PARAM_CA_ROTOR3_PY=0.174
export PX4_PARAM_THR_MDL_FAC=0.75

# 수평 속도 제한 (11/3: 시뮬이 실제보다 느리게 돌아 미션 코드 속도가 1.2~2.4 m/s로 빨라지고, 감속이 늦어 목표를 지나치던 문제)
#   목표점을 끝점으로 한 번에 주고 속도는 PX4가 제한 -> 시뮬 시간 기준 0.8 m/s, 가까워지면 스스로 감속
#   1 m/s 이하라 거리계 높이도 계속 사용 (EKF2_RNG_A_VMAX = 1.0)
export PX4_PARAM_MPC_XY_VEL_MAX=0.8
export PX4_PARAM_MPC_XY_CRUISE=0.8

# 남아 있는 Gazebo 정리
pkill -9 -f "gz sim"; sleep 1

# PX4가 끝나면 (shutdown 또는 Ctrl+C) 남은 Gazebo도 같이 끈다
trap 'pkill -9 -f "gz sim"; echo "Gazebo 정리 완료"' EXIT

# 드론 생성 높이 (그리퍼로 택배를 든 채 시작하면 set_gripper.py가 tools/vehicle/spawn_z.txt 에 씀)
REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SPAWN_Z=$(awk 'NR==1{print $1}' "$REPO_DIR/tools/vehicle/spawn_z.txt" 2>/dev/null)
# 드론 출발 자리 (tools/auditorium_config.py 의 S. 월드 원점 = H. 11/26: H 뒤 1 m)
SPAWN_XY=$(python3 -c "import sys; sys.path.insert(0, '$REPO_DIR/tools'); import auditorium_config as C; s = getattr(C, 'S', C.H); import math; print(f'{s[0]-C.H[0]},{s[1]-C.H[1]}', math.radians(getattr(C, 'S_YAW', 0.0)))" 2>/dev/null)
read SPAWN_XY SPAWN_YAW <<< "$SPAWN_XY"
export PX4_GZ_MODEL_POSE="${SPAWN_XY:-0,0},${SPAWN_Z:-0},0,0,${SPAWN_YAW:-0}"
echo "드론 생성 위치 (월드, H 기준 m): ${PX4_GZ_MODEL_POSE}"

cd ~/PX4-Autopilot
PX4_SYS_AUTOSTART=4021 PX4_GZ_WORLD=${1:-auditorium} PX4_SIM_MODEL=gz_x500_mydrone ./build/px4_sitl_default/bin/px4
