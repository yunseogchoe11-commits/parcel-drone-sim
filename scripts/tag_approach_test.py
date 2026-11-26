"""4번 준비: 아래쪽 카메라로 택배(P)의 AprilTag를 보고 그 위로 정밀 접근하는 시험.

동작
  1. H에서 이륙 1.5 m → P로 이동 (광류 위치만 믿고 감. 이때 오차 약 10~40 cm)
  2. P 위에서 태그 인식 → 카메라 기준 태그 위치(cm) 기록, 사진 저장
  3. 방향 맞추기: 북쪽 0.25 m, 동쪽 0.25 m 씩 움직여 보고 '카메라에서 본 이동'과 '드론 이동'의 관계를 잼
     (카메라 설치 방향을 몰라도 되게. 결과 행렬은 summary.txt에 남김)
  4. 태그 바로 위로 이동 (최대 4번 반복, 3 cm 안이면 끝) → 1.0 m로 내려가서 한 번 더 맞춤
  5. 최종 오차 = 마지막에 카메라로 본 태그 중심의 수평 거리 → 사진 저장
  6. H로 돌아가서 착륙

사용
  0. 준비 (한 번):  python3 tools/vehicle/set_camera.py
                    python3 tools/world/add_apriltag.py
  1. SIM:  bash ~/parcel-drone-sim/tools/run_mydrone_flow_test.sh
  2. CMD:  python3 scripts/tag_approach_test.py 결과폴더이름
           python3 scripts/tag_approach_test.py --view     (비행 없이 카메라 연결·인식만 확인, 바닥에선 태그가 안 보임)
  3. 착륙 후 SIM에서 pxh> shutdown

필요
  pip3 install --user --break-system-packages pupil-apriltags opencv-python
  Gazebo 파이썬 연결: python3 -c "from gz.transport13 import Node"  가 안 되면
                      sudo apt install python3-gz-transport13 python3-gz-msgs10

카메라 기준 좌표 (OpenCV): x = 영상 오른쪽, y = 영상 아래, z = 카메라가 보는 방향. 단위 m.
"""
import asyncio
import csv
import math
import sys
import threading
import time
import warnings
from pathlib import Path

import numpy as np

warnings.filterwarnings("ignore", category=FutureWarning)
REPO = Path(__file__).resolve().parents[1]
CAM_CFG = REPO / "tools/vehicle/camera_config.txt"
TAG_CFG = REPO / "tools/world/apriltag_config.txt"
TOPIC = "/down_camera/image"

CAL_STEP = 0.25        # 방향 맞추기 이동 거리 m
SETTLE_S = 4.0         # 이동 후 멈춰서 기다리는 시간 s
SAMPLE_S = 1.5         # 태그 위치를 모으는 시간 s (중앙값 사용)
DONE_CM = 3.0          # 이 안이면 접근 끝 cm
LOW_ALT = 1.0          # 두 번째 맞춤 고도 m


def read_cfg(p):
    if not p.exists():
        sys.exit(f"{p.relative_to(REPO)} 가 없음. 먼저 set_camera.py / add_apriltag.py 를 실행하세요.")
    return {k: v for k, v in (line.split() for line in p.read_text().splitlines() if line.strip())}


class Camera:
    """Gazebo 카메라 영상을 받아 최신 한 장을 들고 있는다."""

    def __init__(self):
        try:
            from gz.transport13 import Node
            from gz.msgs10.image_pb2 import Image
        except ImportError:
            sys.exit("Gazebo 파이썬 연결이 없음 → sudo apt install python3-gz-transport13 python3-gz-msgs10")
        self.lock = threading.Lock()
        self.frame, self.stamp, self.count = None, 0.0, 0
        self.node = Node()
        if not self.node.subscribe(Image, TOPIC, self._cb):
            sys.exit(f"{TOPIC} 구독 실패")

    def _cb(self, msg):
        img = np.frombuffer(msg.data, np.uint8)
        if img.size != msg.width * msg.height * 3:
            return
        with self.lock:
            self.frame = img.reshape(msg.height, msg.width, 3).copy()
            self.stamp, self.count = time.time(), self.count + 1

    def latest(self):
        with self.lock:
            return (None, 0) if self.frame is None else (self.frame.copy(), self.stamp)


class TagEye:
    def __init__(self):
        from pupil_apriltags import Detector
        cam, tag = read_cfg(CAM_CFG), read_cfg(TAG_CFG)
        self.w, self.h = int(float(cam["width"])), int(float(cam["height"]))
        self.f = float(cam["fx"])
        self.size = float(tag["black_m"])
        self.tag_id = int(tag["id"])
        self.cam_x = float(cam["x"])
        self.det = Detector(families=tag["family"], quad_decimate=1.0)

    def detect(self, rgb):
        gray = rgb.mean(axis=2).astype(np.uint8)
        found = [d for d in self.det.detect(gray, estimate_tag_pose=True,
                                           camera_params=(self.f, self.f, self.w / 2, self.h / 2),
                                           tag_size=self.size) if d.tag_id == self.tag_id]
        return found[0] if found else None

    def draw(self, rgb, d, text, path):
        import cv2
        img = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        cv2.drawMarker(img, (self.w // 2, self.h // 2), (0, 255, 255), cv2.MARKER_CROSS, 30, 1)
        if d is not None:
            pts = d.corners.astype(int)
            cv2.polylines(img, [pts], True, (0, 255, 0), 2)
            cv2.circle(img, tuple(d.center.astype(int)), 4, (0, 0, 255), -1)
        cv2.putText(img, text, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        cv2.imwrite(str(path), img)


async def sample(cam, eye, secs=SAMPLE_S):
    """secs 동안 태그 위치(카메라 기준 x, y, z m)를 모아 중앙값. 못 보면 None."""
    xs, last = [], None
    t_end = time.time() + secs
    seen = 0
    while time.time() < t_end:
        rgb, st = cam.latest()
        if rgb is not None and st != seen:
            seen = st
            d = eye.detect(rgb)
            if d is not None:
                xs.append(d.pose_t.ravel())
                last = (rgb, d)
        await asyncio.sleep(0.03)
    if len(xs) < 3:
        return None, last
    return np.median(np.array(xs), axis=0), last


async def view_only():
    cam, eye = Camera(), TagEye()
    print(f"{TOPIC} 기다리는 중... (시뮬이 켜져 있어야 함)")
    t0 = time.time()
    while cam.latest()[0] is None:
        if time.time() - t0 > 15:
            sys.exit("15초 동안 영상이 안 옴. gz topic -l | grep down_camera 로 토픽 확인, set_camera.py 실행 후 시뮬 재시작")
        await asyncio.sleep(0.2)
    await asyncio.sleep(2)
    rgb, _ = cam.latest()
    n0 = cam.count
    await asyncio.sleep(2)
    print(f"영상 받음: {rgb.shape[1]} x {rgb.shape[0]}, 초당 약 {(cam.count - n0) / 2:.0f}장")
    d = eye.detect(rgb)
    out = REPO / "results/camera_view.png"
    out.parent.mkdir(exist_ok=True)
    eye.draw(rgb, d, "tag found" if d else "no tag", out)
    print("태그:", "보임" if d else "안 보임 (바닥에 있으면 정상. 비행 시험에서 확인)")
    print("저장:", out)


async def flight(tag):
    from mavsdk import System
    from mavsdk.action import ActionError
    from mavsdk.offboard import OffboardError
    sys.path.insert(0, str(REPO / "scripts"))
    from mission_hdph import CONNECT, CRUISE_ALT, TAKEOFF_SPEED, Mission, to_ned
    sys.path.insert(0, str(REPO / "tools"))
    import auditorium_config as C

    out = REPO / "results" / tag
    out.mkdir(parents=True, exist_ok=True)
    cam, eye = Camera(), TagEye()
    rows, L = [], []

    drone = System()
    await drone.connect(system_address=CONNECT)
    print("PX4 연결 기다리는 중...")
    async for s in drone.core.connection_state():
        if s.is_connected:
            break
    async for h in drone.telemetry.health():
        if h.is_local_position_ok:
            break
    if cam.latest()[0] is None:
        await asyncio.sleep(3)
    if cam.latest()[0] is None:
        sys.exit("카메라 영상이 안 옴. --view 로 먼저 확인하세요")

    m = Mission(drone)
    tasks = [asyncio.create_task(m.track())]
    while m.pos is None:
        await asyncio.sleep(0.1)
    async for att in drone.telemetry.attitude_euler():
        yaw = att.yaw_deg % 360
        break
    H, P = to_ned(C.H), to_ned(C.P)
    m.sp = [m.pos[0], m.pos[1], m.pos[2], yaw]
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
        except ActionError:
            await asyncio.sleep(2)
    else:
        sys.exit("시동 실패")

    async def go(n, e, d, wait=SETTLE_S):
        m.sp = [n, e, d, yaw]
        await m.wait_arrive(n, e, d, timeout=40.0)
        await asyncio.sleep(wait)

    async def look(label, alt):
        c, last = await sample(cam, eye)
        pos = m.pos
        if c is None:
            m.log(f"{label}: 태그 안 보임")
            if last is None:
                rgb, _ = cam.latest()
                eye.draw(rgb, None, f"{label}: no tag", out / f"{label}.png")
            return None
        r = math.hypot(c[0], c[1]) * 100
        m.log(f"{label}: 카메라 기준 태그 x {c[0]*100:+.1f}, y {c[1]*100:+.1f} cm (수평 {r:.1f} cm), 거리 {c[2]:.2f} m")
        rows.append([label, f"{alt:.2f}", f"{pos[0]:.3f}", f"{pos[1]:.3f}", f"{c[0]:.4f}", f"{c[1]:.4f}", f"{c[2]:.3f}", f"{r:.1f}"])
        eye.draw(last[0], last[1], f"{label}: {r:.1f} cm", out / f"{label}.png")
        return c

    m.log(f"이륙 → {CRUISE_ALT} m")
    await m.move_to(m.pos[0], m.pos[1], -CRUISE_ALT, TAKEOFF_SPEED)
    m.log("H → P (광류 위치로 이동)")
    await go(P[0], P[1], -CRUISE_ALT)
    c0 = await look("1_P도착_1.5m", CRUISE_ALT)
    A = None
    if c0 is not None:
        # 방향 맞추기: 북 +CAL_STEP, 동 +CAL_STEP 이동 후 카메라 변화
        m.log("방향 맞추기 (북·동으로 25 cm씩)")
        await go(P[0] + CAL_STEP, P[1], -CRUISE_ALT)
        cn = await look("2_보정_북", CRUISE_ALT)
        await go(P[0], P[1] + CAL_STEP, -CRUISE_ALT)
        ce = await look("3_보정_동", CRUISE_ALT)
        await go(P[0], P[1], -CRUISE_ALT)
        c0b = await look("4_보정_원위치", CRUISE_ALT)
        if cn is not None and ce is not None and c0b is not None:
            base = (c0 + c0b) / 2
            D = np.column_stack([(cn - base)[:2], (ce - base)[:2]])     # 카메라 변화 (열 = 북 이동, 동 이동)
            A = -CAL_STEP * np.linalg.inv(D)                             # 카메라 (x, y) → 드론이 갈 (북, 동)
            L.append("[방향 맞추기 결과] 카메라 (x, y) → 이동할 (북, 동) 행렬")
            L.append(f"  [[{A[0,0]:+.3f} {A[0,1]:+.3f}] [{A[1,0]:+.3f} {A[1,1]:+.3f}]]  (이상적이면 ±1, 0 조합)")
            m.log(f"  행렬 [[{A[0,0]:+.2f} {A[0,1]:+.2f}] [{A[1,0]:+.2f} {A[1,1]:+.2f}]]")

    final = None
    if A is not None:
        for alt in (CRUISE_ALT, LOW_ALT):
            if alt != CRUISE_ALT:
                m.log(f"{alt} m로 하강")
                await go(m.sp[0], m.sp[1], -alt)
            for it in range(4):
                c = await look(f"5_맞춤_{alt}m_{it + 1}", alt)
                if c is None:
                    break
                final = (alt, c)
                if math.hypot(c[0], c[1]) * 100 < DONE_CM:
                    break
                dn, de = A @ c[:2]
                await go(m.sp[0] + dn, m.sp[1] + de, -alt)
        if final is not None:
            c = await look("6_최종", final[0])
            if c is not None:
                final = (final[0], c)

    m.log("H로 복귀, 착륙")
    await go(H[0], H[1], -CRUISE_ALT, wait=1.0)
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
    cam_cfg, tag_cfg = read_cfg(CAM_CFG), read_cfg(TAG_CFG)
    S = [f"AprilTag 정밀 접근 시험 ({tag})",
         f"카메라: 앞 {cam_cfg['x']} m, 위아래 {cam_cfg['z']} m, pitch {cam_cfg['pitch']}°, 가로 화각 {cam_cfg['hfov']}°, "
         f"{int(float(cam_cfg['width']))} x {int(float(cam_cfg['height']))}, 초당 {cam_cfg['rate']}장",
         f"태그: {tag_cfg['family']} id {tag_cfg['id']}, 검은 사각형 {float(tag_cfg['black_m']):.3f} m (택배 상자 윗면, P)", ""]
    if c0 is None:
        S.append("P 도착 (1.5 m): 태그 안 보임 → 카메라 위치·화각 또는 광류 위치 오차 확인")
    else:
        S.append(f"P 도착 (1.5 m, 광류 위치만): 태그까지 수평 {math.hypot(c0[0], c0[1]) * 100:.1f} cm")
    S += L
    if final is not None:
        alt, c = final
        S.append(f"최종 ({alt} m, 태그 보고 맞춘 뒤): 카메라 중심 ~ 태그 중심 수평 {math.hypot(c[0], c[1]) * 100:.1f} cm "
                 f"(x {c[0]*100:+.1f}, y {c[1]*100:+.1f} cm)")
        if abs(float(cam_cfg['x'])) > 0.005:
            S.append(f"  참고: 카메라가 기체 중심보다 앞 {float(cam_cfg['x'])*100:.0f} cm에 있음. 그리퍼를 태그 위에 두려면 이만큼 더 옮겨야 함")
        else:
            S.append("  참고: 카메라가 그리퍼 하우징 바로 아래(기체 가운데)라서 카메라 중심 = 그리퍼 중심")
    S.append("")
    S.append("사진: 1_P도착 / 5_맞춤 / 6_최종 .png (초록 = 인식한 태그, 빨간 점 = 태그 중심, 노란 + = 영상 중심)")
    (out / "summary.txt").write_text("\n".join(S) + "\n", encoding="utf-8")
    print("\n" + "\n".join(S))
    print(f"\n저장: {out}  →  SIM에서 pxh> shutdown")


if __name__ == "__main__":
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    if sys.argv[1] == "--view":
        asyncio.run(view_only())
    else:
        asyncio.run(flight(sys.argv[1]))
