"""호버링 시험: H에서 이륙 → 1.5 m에서 정지 → 착륙. (무게·무게중심 바꾼 뒤 확인용)

사용
  1. SIM:  bash ~/parcel-drone-sim/tools/run_mydrone_flow_test.sh   (무게 바꾼 경우)
           bash ~/parcel-drone-sim/tools/run_mydrone_flow.sh        (원래 모델)
  2. QGC:  ~/QGroundControl-x86_64.AppImage  (연결 확인, 시동 버튼은 누르지 않음)
  3. CMD:  python3 scripts/hover_test.py            (기본 30초 정지)
           python3 scripts/hover_test.py 60         (60초 정지)
  4. 착륙 후 SIM에서 pxh> shutdown
  5. 분석: python3 tools/analysis/analyze_hover.py 결과폴더이름

이륙·시동 방식은 mission_hdph.py와 같다 (GPS 없이 Offboard로 이륙).
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

HOVER_S = float(sys.argv[1]) if len(sys.argv) > 1 else 30.0


async def main():
    drone = System()
    await drone.connect(system_address=CONNECT)
    print("PX4 연결 기다리는 중...")
    async for s in drone.core.connection_state():
        if s.is_connected:
            break
    print("위치 추정(광류) 준비 기다리는 중...")
    async for h in drone.telemetry.health():
        if h.is_local_position_ok:
            break

    m = Mission(drone)
    tasks = [asyncio.create_task(m.track())]
    while m.pos is None:
        await asyncio.sleep(0.1)
    async for att in drone.telemetry.attitude_euler():
        yaw_now = att.yaw_deg % 360
        break
    n0, e0 = m.pos[0], m.pos[1]

    m.sp = [n0, e0, m.pos[2], yaw_now]
    tasks.append(asyncio.create_task(m.stream()))
    await asyncio.sleep(1.0)
    try:
        await drone.offboard.start()
    except OffboardError as err:
        m.log(f"Offboard 전환 실패: {err._result.result}")
        return
    m.log("Offboard 전환, 시동")
    for k in range(5):
        try:
            await drone.action.arm()
            break
        except ActionError as err:
            m.log(f"  시동 거부 ({err._result.result}), 2초 뒤 다시 시도 {k + 1}/5")
            await asyncio.sleep(2)
    else:
        m.log("시동 실패. QGC 왼쪽 위 'Not Ready'를 눌러 이유를 확인하세요.")
        return

    m.log(f"이륙 → {CRUISE_ALT} m ({TAKEOFF_SPEED} m/s)")
    await m.move_to(n0, e0, -CRUISE_ALT, TAKEOFF_SPEED)
    m.log(f"호버링 {HOVER_S:.0f} s (이 동안 pxh> listener actuator_motors 로 모터 출력 확인 가능)")
    await asyncio.sleep(HOVER_S)

    m.log("착륙")
    await drone.action.land()
    async for in_air in drone.telemetry.in_air():
        if not in_air:
            break
    m.log("착륙 완료. SIM 터미널에서 pxh> shutdown 후 분석")
    for t in tasks:
        t.cancel()
    # MAVSDK 연결이 남아 프로그램이 안 끝나는 경우가 있어 여기서 바로 종료 (착륙 후라 안전)
    sys.stdout.flush()
    os._exit(0)


if __name__ == "__main__":
    asyncio.run(main())
    os._exit(0)
