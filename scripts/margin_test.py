"""여유(마진) 시험: 호버 + 급상승 + 빠른 옆 이동. 무게를 늘려도 '정상 비행'이 되는지 판단용.

동작 (H 제자리, 약 1분 30초)
  1. 이륙 1.5 m → 15 s 호버                  → 호버 스로틀 (analyze_hover.py --margin 의 호버 구간)
  2. 급상승: 목표를 2.5 m로 한 번에 줌        → PX4 기본 제한(최대 상승 속도·가속)까지 밀어붙임 → 모터 여유
  3. 1.5 m로 복귀
  4. 빠른 옆 이동: 속도 제한 0.8 → 1.5 m/s, 북쪽 2 m 갔다가 복귀 → 기울기·자세 제어 여유
  5. 속도 제한 0.8 m/s로 되돌리고 착륙

판정 (tools/analysis/analyze_hover.py 결과폴더 --margin)
  - 호버 스로틀 ≤ 70 % (팀 기준), 모터 출력이 0.95 이상인 시간(포화) 거의 0
  - 급상승·옆 이동 중에도 기울기·고도가 크게 넘치지 않고 돌아옴

사용
  SIM: bash ~/parcel-drone-sim/tools/run_mydrone_flow_test.sh
  CMD: python3 scripts/margin_test.py
  착륙 후 SIM: pxh> shutdown
"""
import asyncio
import os
import sys
import warnings

warnings.filterwarnings("ignore", category=FutureWarning)
from mavsdk import System
from mavsdk.action import ActionError
from mavsdk.offboard import OffboardError

from mission_hdph import CONNECT, CRUISE_ALT, TAKEOFF_SPEED, Mission

HOVER_S = 15.0
CLIMB_TO = 2.5          # m
STEP_N = 2.0            # m
STEP_VEL = 1.5          # m/s (옆 이동 중 속도 제한)
HOLD_S = 6.0


async def main():
    drone = System()
    await drone.connect(system_address=CONNECT)
    print("PX4 연결 기다리는 중...")
    async for s in drone.core.connection_state():
        if s.is_connected:
            break
    async for h in drone.telemetry.health():
        if h.is_local_position_ok:
            break

    m = Mission(drone)
    tasks = [asyncio.create_task(m.track())]
    while m.pos is None:
        await asyncio.sleep(0.1)
    async for att in drone.telemetry.attitude_euler():
        yaw = att.yaw_deg % 360
        break
    n0, e0 = m.pos[0], m.pos[1]
    m.sp = [n0, e0, m.pos[2], yaw]
    tasks.append(asyncio.create_task(m.stream()))
    await asyncio.sleep(1.0)
    try:
        await drone.offboard.start()
    except OffboardError as err:
        m.log(f"Offboard 전환 실패: {err._result.result}")
        return
    for k in range(5):
        try:
            await drone.action.arm()
            break
        except ActionError as err:
            m.log(f"  시동 거부 ({err._result.result}), 다시 시도 {k + 1}/5")
            await asyncio.sleep(2)
    else:
        m.log("시동 실패")
        return

    async def jump(n, e, d, hold):
        m.sp = [n, e, d, yaw]
        await m.wait_arrive(n, e, d, timeout=20.0)
        await asyncio.sleep(hold)

    m.log(f"이륙 → {CRUISE_ALT} m")
    await m.move_to(n0, e0, -CRUISE_ALT, TAKEOFF_SPEED)
    m.log(f"1. 호버 {HOVER_S:.0f} s")
    await asyncio.sleep(HOVER_S)
    m.log(f"2. 급상승 → {CLIMB_TO} m (목표 한 번에)")
    await jump(n0, e0, -CLIMB_TO, HOLD_S)
    m.log(f"3. {CRUISE_ALT} m로 복귀")
    await jump(n0, e0, -CRUISE_ALT, HOLD_S)

    vel0 = await drone.param.get_param_float("MPC_XY_VEL_MAX")
    await drone.param.set_param_float("MPC_XY_VEL_MAX", STEP_VEL)
    m.log(f"4. 옆 이동: 속도 제한 {vel0:.1f} → {STEP_VEL} m/s, 북쪽 {STEP_N} m 갔다 오기")
    await jump(n0 + STEP_N, e0, -CRUISE_ALT, HOLD_S)
    await jump(n0, e0, -CRUISE_ALT, HOLD_S)
    await drone.param.set_param_float("MPC_XY_VEL_MAX", vel0)

    m.log("5. 착륙")
    await drone.action.land()
    async for in_air in drone.telemetry.in_air():
        if not in_air:
            break
    m.log("착륙 완료. SIM 터미널에서 pxh> shutdown 후 analyze_hover.py 폴더 --margin")
    for t in tasks:
        t.cancel()
    # MAVSDK 연결이 남아 프로그램이 안 끝나는 경우가 있어 여기서 바로 종료 (착륙 후라 안전)
    sys.stdout.flush()
    os._exit(0)


if __name__ == "__main__":
    asyncio.run(main())
    os._exit(0)
