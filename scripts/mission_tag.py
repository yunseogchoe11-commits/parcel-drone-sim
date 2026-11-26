"""미션 비행 + 카메라 정밀 접근: H → D(배달) → P(수거, AprilTag 보고 정밀 접근) → H.

mission_hdph.py 와 같은 동선·고도·속도로 날고, P에서만 아래쪽 카메라로 택배 윗면 AprilTag를 보고
그리퍼(카메라)를 태그 바로 위로 맞춘 다음 내려간다. (tag_approach_test.py 는 H → P만 가는 단독 시험)

동작
  1. H 이륙 1.5 m → H → D (0.8 m/s, 거리는 tools/auditorium_config.py) → D 0.5 m 하강, 5 s (택배 내려놓기 흉내) → 1.5 m
  2. 오른쪽 90° → D → P (광류 위치만 믿고 감. 이때 오차 약 10~40 cm)
  3. P 위 1.5 m: 태그 인식 → 태그 바로 위로 이동 (최대 4번, 3 cm 안이면 끝)
  4. 1.0 m로 내려가서 한 번 더 맞춤 → 0.5 m(집기 고도)에서 한 번 더 맞추고 마지막 확인, 5 s (택배 집기 흉내) → 1.5 m
     (11/24 첫 시험: 1.0 m 2.0 cm였는데 0.5 m로 내려가면서 8.0 cm로 밀림 → 0.5 m 맞춤 추가)
  5. 오른쪽 135° → P → H, 착륙

카메라 → 이동 방향
  카메라는 바로 아래를 보고 화면 위쪽 = 기체 앞 (set_camera.py pitch 90°, 11/24 그리퍼 하우징 아래).
  태그가 영상에서 (x 오른쪽, y 아래)에 보이면 기체 기준 앞 -y, 오른쪽 +x → 지금 기수로 북·동으로 바꿔 이동.
  (tag_approach_test.py 처럼 25 cm씩 움직여 재지 않음. 11/22 시험에서 잰 행렬이 이 계산과 같았음: [[-1.02 -0.07] [+0.03 -0.96]])

사용
  0. 준비 (한 번): python3 tools/vehicle/set_camera.py   (또는 set_sensors.py --camera ...)
                   python3 tools/world/add_apriltag.py
  1. SIM:  bash ~/parcel-drone-sim/tools/run_mydrone_flow_test.sh
  2. CMD:  python3 scripts/mission_tag.py 결과폴더이름
  3. 착륙 후 SIM에서 pxh> shutdown
  4. 동선 오차 분석: python3 tools/analysis/analyze_mission_log.py 결과폴더이름_route
결과: results/결과폴더이름/ summary.txt, detections.csv, 사진 (P도착·맞춤·최종)
"""
import asyncio
import csv
import math
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "scripts"))
sys.path.insert(0, str(REPO / "tools"))
from mission_hdph import (CONNECT, CRUISE_ALT, CRUISE_SPEED, HOLD_S, TAKEOFF_SPEED, VERT_SPEED, WORK_ALT,  # noqa: E402
                          Mission, heading_deg, to_ned)
from tag_approach_test import CAM_CFG, TAG_CFG, Camera, TagEye, read_cfg, sample  # noqa: E402
import auditorium_config as C  # noqa: E402

LOW_ALT = 1.0       # 두 번째 맞춤 고도 m
DONE_CM = 3.0       # 이 안이면 맞춤 끝 cm
MAX_TRIES = 4       # 고도마다 최대 맞춤 횟수
SETTLE_S = 3.0      # 이동 후 멈춰서 기다리는 시간 s


def cam_to_ne(c, yaw_deg):
    """카메라에서 본 태그 (x 오른쪽, y 아래, m) → 드론이 갈 (북, 동) m."""
    fwd, right = -c[1], c[0]
    y = math.radians(yaw_deg)
    return fwd * math.cos(y) - right * math.sin(y), fwd * math.sin(y) + right * math.cos(y)


async def main(tag):
    from mavsdk import System
    from mavsdk.action import ActionError
    from mavsdk.offboard import OffboardError

    out = REPO / "results" / tag
    out.mkdir(parents=True, exist_ok=True)
    cam, eye = Camera(), TagEye()
    rows, log_lines = [], []

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
    if cam.latest()[0] is None:
        await asyncio.sleep(3)
    if cam.latest()[0] is None:
        sys.exit("카메라 영상이 안 옴. python3 scripts/tag_approach_test.py --view 로 먼저 확인하세요")

    H, D, P = to_ned(C.H), to_ned(C.D), to_ned(C.P)
    yaw_hd, yaw_dp, yaw_ph = heading_deg(H, D), heading_deg(D, P), heading_deg(P, H)
    m = Mission(drone)

    def log(msg):
        m.log(msg)
        log_lines.append(msg)

    tasks = [asyncio.create_task(m.track())]
    while m.pos is None:
        await asyncio.sleep(0.1)
    async for att in drone.telemetry.attitude_euler():
        yaw_now = att.yaw_deg % 360
        break
    m.sp = [m.pos[0], m.pos[1], m.pos[2], yaw_now]
    tasks.append(asyncio.create_task(m.stream()))
    await asyncio.sleep(1.0)
    try:
        await drone.offboard.start()
    except OffboardError as err:
        sys.exit(f"Offboard 전환 실패: {err._result.result}")
    for k in range(5):
        try:
            await drone.action.arm()
            break
        except ActionError as err:
            m.log(f"  시동 거부 ({err._result.result}), 2초 뒤 다시 {k + 1}/5")
            await asyncio.sleep(2)
    else:
        sys.exit("시동 실패. QGC 'Not Ready' 이유 확인")

    async def go(n, e, d, wait=SETTLE_S):
        m.sp = [n, e, d, m.sp[3]]
        await m.wait_arrive(n, e, d, timeout=20.0)
        await asyncio.sleep(wait)

    async def look(label, alt):
        c, last = await sample(cam, eye)
        if c is None:
            log(f"{label}: 태그 안 보임")
            rgb, _ = cam.latest()
            if last is None and rgb is not None:
                eye.draw(rgb, None, f"{label}: no tag", out / f"{label}.png")
            return None
        r = math.hypot(c[0], c[1]) * 100
        log(f"{label}: 카메라 중심 ~ 태그 수평 {r:.1f} cm (x {c[0]*100:+.1f}, y {c[1]*100:+.1f}), 거리 {c[2]:.2f} m")
        rows.append([label, f"{alt:.2f}", f"{m.pos[0]:.3f}", f"{m.pos[1]:.3f}",
                     f"{c[0]:.4f}", f"{c[1]:.4f}", f"{c[2]:.3f}", f"{r:.1f}"])
        eye.draw(last[0], last[1], f"{label}: {r:.1f} cm", out / f"{label}.png")
        return c

    async def align(alt, name, c_first=None):
        """지금 고도에서 태그 바로 위로. 마지막으로 본 태그 위치 (없으면 None).
        c_first: 방금 본 태그 위치가 있으면 다시 보지 않고 바로 옮김."""
        c_last = None
        for it in range(MAX_TRIES):
            c = c_first if (it == 0 and c_first is not None) else await look(f"{name}_{alt:.1f}m_{it + 1}", alt)
            if c is None:
                break
            c_last = c
            if math.hypot(c[0], c[1]) * 100 < DONE_CM:
                break
            dn, de = cam_to_ne(c, m.sp[3])
            await go(m.sp[0] + dn, m.sp[1] + de, -alt)
        return c_last

    # 1) 이륙, H → D, 배달
    log(f"이륙 → {CRUISE_ALT} m")
    await m.move_to(H[0], H[1], -CRUISE_ALT, TAKEOFF_SPEED)
    await asyncio.sleep(2.0)
    await m.turn_to(yaw_hd)
    log("H → D 이동")
    await m.move_to(D[0], D[1], -CRUISE_ALT, CRUISE_SPEED, ramp=False)
    log("D 도착, 하강 (택배 내려놓기)")
    await m.move_to(D[0], D[1], -WORK_ALT, VERT_SPEED)
    await asyncio.sleep(HOLD_S)
    await m.move_to(D[0], D[1], -CRUISE_ALT, VERT_SPEED)

    # 2) D → P (광류)
    log("오른쪽 회전 → P")
    await m.turn_to(yaw_dp)
    await m.move_to(P[0], P[1], -CRUISE_ALT, CRUISE_SPEED, ramp=False)
    await asyncio.sleep(SETTLE_S)
    p_est = m.pos

    # 3) P: 카메라 정밀 접근
    log("P 도착 (광류 위치), 카메라로 태그 찾기")
    c0 = await look("1_P도착_1.5m", CRUISE_ALT)
    c1 = c2 = c_work = None
    if c0 is not None:
        c1 = await align(CRUISE_ALT, "2_맞춤", c0)
        log(f"{LOW_ALT} m로 하강")
        await go(m.sp[0], m.sp[1], -LOW_ALT)
        c2 = await align(LOW_ALT, "3_맞춤")
        log(f"{WORK_ALT} m로 하강 (택배 집기)")
        await m.move_to(m.sp[0], m.sp[1], -WORK_ALT, VERT_SPEED)
        await asyncio.sleep(SETTLE_S)
        await align(WORK_ALT, "4_맞춤")          # 내려가는 동안 밀린 만큼 (11/24 시험: 0.5 m에서 8 cm) 다시 맞춤
        c_work = await look("5_최종_집기고도", WORK_ALT)
        await asyncio.sleep(max(0.0, HOLD_S - 2.5))
    else:
        log("태그 안 보임 → 광류 위치에서 그대로 하강")
        await m.move_to(P[0], P[1], -WORK_ALT, VERT_SPEED)
        await asyncio.sleep(HOLD_S)
    shift = math.dist(p_est[:2], m.sp[:2]) * 100
    await m.move_to(m.sp[0], m.sp[1], -CRUISE_ALT, VERT_SPEED)

    # 4) P → H, 착륙
    log("오른쪽 회전 → H")
    await m.turn_to(yaw_ph)
    await m.move_to(H[0], H[1], -CRUISE_ALT, CRUISE_SPEED, ramp=False)
    log("H 도착, 착륙")
    await asyncio.sleep(2.0)
    await drone.action.land()
    async for in_air in drone.telemetry.in_air():
        if not in_air:
            break
    for t in tasks:
        t.cancel()

    with open(out / "detections.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["단계", "고도_m", "추정위치_북_m", "추정위치_동_m", "태그_x_m", "태그_y_m", "태그거리_m", "수평오차_cm"])
        w.writerows(rows)
    cc, tc = read_cfg(CAM_CFG), read_cfg(TAG_CFG)
    hz = lambda c: f"{math.hypot(c[0], c[1]) * 100:.1f} cm" if c is not None else "태그 안 보임"
    S = [f"미션 비행 + 카메라 정밀 접근 ({tag})",
         f"동선: H → D → P → H ({math.dist(C.H, C.D) + math.dist(C.D, C.P) + math.dist(C.P, C.H):.1f} m, 1.5 m, 0.8 m/s), 정밀 접근은 P에서",
         f"카메라: 위치 ({float(cc['x'])*1000:.0f}, {float(cc['y'])*1000:.0f}, {float(cc['z'])*1000:.0f}) mm, pitch {cc['pitch']}°, "
         f"가로 화각 {cc['hfov']}°, {int(float(cc['width']))} x {int(float(cc['height']))}",
         f"태그: {tc['family']} id {tc['id']}, 검은 사각형 {float(tc['black_m']):.3f} m (P 택배 윗면)", "",
         "[P에서 카메라 중심 ~ 태그 중심 수평 거리]",
         f"  광류 위치로 도착 (1.5 m)   : {hz(c0)}",
         f"  1.5 m 맞춤 후              : {hz(c1)}",
         f"  1.0 m 맞춤 후              : {hz(c2)}",
         f"  집기 고도 {WORK_ALT} m 내려간 뒤  : {hz(c_work)}",
         f"  카메라 보고 옮긴 거리 (광류 목표 → 최종): {shift:.1f} cm", ""]
    if abs(float(cc["x"])) < 0.005 and abs(float(cc["y"])) < 0.005:
        S.append("참고: 카메라가 그리퍼 하우징 바로 아래라서 카메라 중심 = 그리퍼 중심")
    S += ["", "진행 기록:"] + [f"  {l}" for l in log_lines]
    S += ["", "사진: 1_P도착 / 2_맞춤 / 3_맞춤 / 4_맞춤 / 5_최종 .png (초록 = 인식한 태그, 빨간 점 = 태그 중심, 노란 + = 영상 중심)",
          f"동선 오차 분석: python3 tools/analysis/analyze_mission_log.py {tag}_route"]
    (out / "summary.txt").write_text("\n".join(S) + "\n", encoding="utf-8")
    print("\n" + "\n".join(S[:12]))
    print(f"\n저장: {out}  →  SIM에서 pxh> shutdown")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    asyncio.run(main(sys.argv[1]))
