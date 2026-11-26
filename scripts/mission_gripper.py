"""그리퍼 배달·수거 미션 (11/26): S(빈손 출발, H 뒤 1 m) → H(카메라로 찾아 집기) → D(놓기) → P(카메라로 찾아 집기) → H(들고 착륙).

준비 (한 번, 순서대로)
  python3 tools/vehicle/set_mass_cg.py 2600 -12.1 --cg-y -1.0 --cg-z -34.8 --ground -270
  python3 tools/vehicle/set_sensors.py --flow 169 175 -5 --range 189 175 -5 --camera 0 0 -144 90
  python3 tools/vehicle/set_gripper.py
  python3 tools/world/make_auditorium_world.py && python3 tools/world/add_apriltag.py
실행
  SIM: bash tools/run_mydrone_flow_test.sh
  CMD: python3 scripts/mission_gripper.py 결과폴더        (카메라 창: python3 scripts/camera_view.py)
  착륙 후 SIM에서 pxh> shutdown,  동선 분석: python3 tools/analysis/analyze_mission_log.py 결과폴더_route

동작 (위치는 tools/auditorium_config.py: S = H 뒤 1 m, 첫 택배 B = H, H→D 5 m, D→P 5 m, P→H 7.07 m)
  0. 비행 전: 두 택배 모두 시뮬 시작 때 기체에 붙어 있으므로 떼고, 손가락을 벌림 (빈손 출발)
  1. S에서 이륙 1.5 m → H(B). 택배 방향으로 기수를 한 번 맞추고, 카메라로 태그(id 1)를 보며 1.5·1.0·0.5 m에서 맞춤 → 집기
  2. B → D. 택배 바닥이 바닥에서 약 4 cm 될 때까지 내려가서 손가락 열고 놓음
  3. D → P. 카메라로 태그(id 0)를 보며 맞춤 → 집기
  4. P → H, 택배를 든 채 착륙
  광류 누적 오차 보정: 카메라로 맞춘 순간 실제 위치 = 택배 자리이므로, 드론이 믿는 위치와의 차이를 재서
    B에서 잰 값은 D·P 목표에, P에서 잰 값은 H 목표에 반영
  기수(yaw)는 출발 방향(동쪽) 그대로. 택배를 90° 돌려 놓아서(config PARCEL_YAW 90) 손가락이 긴 쪽(0.25 m)을 걸쳐 잡음 → 회전 없음 (제자리 회전하면 광류 위치가 30~50 cm 튐.
    회전 직후 카메라 보정이 있어서 B에서의 회전 오차는 빠짐). 손가락이 택배 좁은 쪽(0.20 m)을 쥐어야 함
  택배 근처·착륙 때도 광류를 0.15 m까지 씀 (--imu-low: 옛 방식, 0.5 m 아래 IMU만)
"""
import asyncio
import csv
import math

import numpy as np
import sys
import time
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "scripts"))
sys.path.insert(0, str(REPO / "tools"))
sys.path.insert(0, str(REPO / "tools/vehicle"))
from mission_hdph import (CONNECT, CRUISE_ALT, CRUISE_SPEED, TAKEOFF_SPEED, VERT_SPEED, WORK_ALT,  # noqa: E402
                          Mission, to_ned)
from tag_approach_test import CAM_CFG, TAG_CFG, Camera, TagEye, read_cfg, sample  # noqa: E402
import auditorium_config as C  # noqa: E402
import set_gripper as G  # noqa: E402

LOW_ALT = 1.0
DONE_CM = 3.0
MAX_TRIES = 4
SETTLE_S = 3.0
LOW_SPEED = 0.15                  # 택배 근처에서 오르내리는 속도 m/s
GAP = 0.04                        # 놓기·잡기 때 택배 바닥과 바닥 사이 m
FLOW_LOW = "--imu-low" not in sys.argv  # 기본: 택배 근처·착륙 때도 광류 사용 (0.15 m까지). --imu-low = 옛 방식(0.5 m 아래 IMU만)
# 11/25 비교: IMU만 H 35 cm(제멋대로) → 광류 0.15 m까지 25 cm(동쪽 일정). 남은 동쪽 오차는 착륙 하강(0.5 m 아래 IMU만) 때 생김 → 착륙도 광류
FLOW_LOW_HGT = 0.15
CENTER_MAX = 0.10                 # 손가락 사이에서 이만큼(m)까지 어긋난 택배는 가운데로 끼워 맞춤, 더 크면 집지 않음
SERVO_END = 0.13                  # 카메라로 태그를 보며 내려갈 마지막 높이 m. 이 높이에서 카메라~태그 약 0.12 m (태그 0.18 m가 화면에 다 들어오는 최저 근처)
                                  # 11/26 영상용2~5: 그 아래를 IMU만으로 가면 8~13 cm 밀림 → 카메라 없이 가는 구간을 7 cm 남짓으로
FINGER_HOLD = 0.12                # 택배를 붙인 뒤 손가락 각도 rad (긴 쪽 0.25 m 옆면과 틈 약 1 cm)
RNG_BLOCKED = 0.45                # 0.5 m에서 거리계가 이보다 짧으면 택배가 가린 것 (정상은 약 0.75 m)
FINAL_ALT = 0.3                   # 집기 전 마지막 카메라 맞춤 높이 m
GRAB_SHIFT = {"B": (-0.08, 0.0), "P": (0.0, 0.0)}  # 마지막 몇 cm 내려가기 전에 드론을 미리 옮기는 양 (앞, 왼쪽) m
#   H: 11/26 영상용3·5·9에서 택배가 늘 앞 −10, −13, −10 cm(드론이 앞으로 밀림) → 8 cm 뒤로. P는 1~5 cm라 그대로
DROP_FLOW_OFF = 0.7                # 집으러 빠르게 내려갈 때 광류를 끄는 높이 (0.5 m보다 위로 둬서 바로 꺼지게)
SERVO_SAFE_CM = 8.0               # 카메라 보며 0.13 m까지 내려왔을 때 이보다 벗어나 있으면 집지 않음
SAFE_CM = 4.0                     # 집기 직전 태그가 그리퍼 중심에서 이보다 멀면 집지 않고 멈춤
NOAID_LOW, NOAID_NORMAL = 15_000_000, 5_000_000   # EKF2_NOAID_TOUT (us): 낮게 떠 있는 동안 광류 없이 버티는 시간


def cam_to_ne(c, yaw_deg):
    fwd, right = -c[1], c[0]
    y = math.radians(yaw_deg)
    return fwd * math.cos(y) - right * math.sin(y), fwd * math.sin(y) + right * math.cos(y)


class PickFail(Exception):
    pass


class Gripper:
    """Gazebo 토픽으로 손가락·택배 붙이기 (set_gripper.py가 모델에 넣은 플러그인)."""

    def __init__(self):
        from gz.transport13 import Node
        from gz.msgs10.double_pb2 import Double
        from gz.msgs10.empty_pb2 import Empty
        from gz.msgs10.stringmsg_pb2 import StringMsg
        self.Double, self.Empty = Double, Empty
        self.node = Node()
        self.pub = {t: self.node.advertise(t, Double) for t in ("/gripper/finger_l", "/gripper/finger_r")}
        for c in ("parcel_H", "parcel"):
            for a in ("attach", "detach"):
                self.pub[f"/{c}/{a}"] = self.node.advertise(f"/{c}/{a}", Empty)
        self.state = {}
        for c in ("parcel_H", "parcel"):
            self.node.subscribe(StringMsg, f"/{c}/state", lambda m, c=c: self.state.__setitem__(c, m.data))
        # Gazebo 실제 위치 (정답): 택배를 그리퍼 어디에 붙였는지, 어디에 놓았는지 재려고 (11/26)
        from gz.msgs10.pose_v_pb2 import Pose_V
        self.poses = {}
        self.node.subscribe(Pose_V, "/world/auditorium/pose/info", self._on_poses)

    def _on_poses(self, msg):
        got = set()
        for p in msg.pose:   # 모델 이름별 첫 번째 = 모델 위치
            if p.name in ("x500_mydrone_0", "parcel", "parcel_H") and p.name not in got:
                got.add(p.name)
                q = p.orientation
                yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
                self.poses[p.name] = (p.position.x, p.position.y, p.position.z, yaw)

    def offset(self, box):
        """택배 중심이 드론 기준 어디 있는지 (앞, 왼쪽, 위아래 오차) cm. 위아래 0 = 설계대로 쥔 높이."""
        if "x500_mydrone_0" not in self.poses or box not in self.poses:
            return None
        dx, dy, dz, yaw = self.poses["x500_mydrone_0"]
        bx, by, bz, _ = self.poses[box]
        ex, ey = bx - dx, by - dy
        fwd, left = ex * math.cos(yaw) + ey * math.sin(yaw), -ex * math.sin(yaw) + ey * math.cos(yaw)
        # 모델 원점 = base_link 0.24 m 아래, 택배 중심 설계 위치 = base_link 아래 0.21 m → 모델 원점보다 0.03 m 위
        return fwd * 100, left * 100, (bz - dz - (0.24 + G.HOLD_Z)) * 100

    def center_box(self, box, max_m):
        """손가락 사이(그리퍼 중심 바로 아래)로 택배를 옮겨 놓음 (수평만, 높이·방향은 그대로).
        실제 그리퍼는 손가락이 V자로 택배를 끼워 맞추는데, Gazebo 손가락은 좌우만 밀어 줘서 앞뒤로는 못 맞춤.
        앞뒤로 치우친 채 붙이면 택배 모서리가 광류 카메라를 가려서 드론이 위치를 잃음
        (11/26 서보1 로그: 앞 +15.8 cm로 붙이고 올라가는 동안 실제로 2.6 m 밀렸는데 드론은 제자리라고 믿음).
        max_m보다 많이 어긋나 있으면 옮기지 않고 None (그때는 집지 않음). 반환: 옮긴 거리 m."""
        from gz.msgs10.pose_pb2 import Pose
        from gz.msgs10.boolean_pb2 import Boolean
        if "x500_mydrone_0" not in self.poses or box not in self.poses:
            return None
        dx, dy, _, _ = self.poses["x500_mydrone_0"]
        bx, by, bz, byaw = self.poses[box]
        dist = math.hypot(bx - dx, by - dy)
        if dist > max_m:
            return None
        req = Pose()
        req.name = box
        req.position.x, req.position.y, req.position.z = dx, dy, bz
        req.orientation.w, req.orientation.z = math.cos(byaw / 2), math.sin(byaw / 2)
        try:
            ok, rep = self.node.request("/world/auditorium/set_pose", req, Pose, Boolean, 1000)
        except Exception:
            ok = False
        return dist if ok else -1.0          # -1: 옮기기 서비스 실패 (그대로 붙임)

    def ground_err(self, name, hall_xy):
        """name(드론/택배)의 실제 위치가 강당 지점에서 얼마나 떨어졌나 (동, 북) cm."""
        if name not in self.poses:
            return None
        x, y = hall_xy[0] - C.H[0], hall_xy[1] - C.H[1]
        return (self.poses[name][0] - x) * 100, (self.poses[name][1] - y) * 100
        time.sleep(1.0)            # 구독자와 연결될 시간

    def fingers(self, closed, angle=None):
        a = G.FINGER_CLOSE if angle is None else angle
        for n, sign in (("finger_l", -1), ("finger_r", 1)):
            msg = self.Double()
            msg.data = sign * a if closed else 0.0
            for _ in range(3):
                self.pub[f"/gripper/{n}"].publish(msg)

    def joint(self, child, attach):
        for _ in range(3):
            self.pub[f"/{child}/{'attach' if attach else 'detach'}"].publish(self.Empty())
            time.sleep(0.1)


async def main(tag):
    from mavsdk import System
    from mavsdk.action import ActionError
    from mavsdk.offboard import OffboardError

    out = REPO / "results" / tag
    out.mkdir(parents=True, exist_ok=True)
    cam, eye, grip = Camera(), TagEye(), Gripper()
    rows, log_lines = [], []

    drone = System()
    await drone.connect(system_address=CONNECT)
    print("PX4 연결 기다리는 중...")
    async for s in drone.core.connection_state():
        if s.is_connected:
            break

    # 0) 두 택배 모두 시뮬 시작 때 기체에 붙어 있음 → 떼고, 손가락 벌림 (빈손 출발)
    grip.joint("parcel_H", attach=False)
    grip.joint("parcel", attach=False)
    grip.fingers(closed=False)
    await asyncio.sleep(2.0)
    print(f"택배 상태: H {grip.state.get('parcel_H', '?')}, P {grip.state.get('parcel', '?')} "
          f"(둘 다 detached 여야 함. '?'면 상태 토픽이 없는 Gazebo 버전 — 영상으로 확인)")

    print("위치 추정(광류) 준비 기다리는 중...")
    async for h in drone.telemetry.health():
        if h.is_local_position_ok:
            break
    if cam.latest()[0] is None:
        await asyncio.sleep(3)
    if cam.latest()[0] is None:
        sys.exit("카메라 영상이 안 옴. python3 scripts/tag_approach_test.py --view 로 먼저 확인하세요")
    await drone.param.set_param_float("SENS_FLOW_MINHGT", 0.5)
    await drone.param.set_param_int("EKF2_NOAID_TOUT", NOAID_NORMAL)

    o = to_ned(getattr(C, "S", C.H))   # 로컬 위치 원점 = 드론 출발 자리 S
    loc = lambda X: (to_ned(X)[0] - o[0], to_ned(X)[1] - o[1])
    Sl, H, B, D, P = loc(getattr(C, "S", C.H)), loc(C.H), loc(C.B), loc(C.D), loc(C.P)
    # 택배를 쥐는 기수 = 택배 긴 쪽(0.25 m)이 드론 좌우(손가락 사이)가 되는 방향 (PARCEL_YAW 90 → 동쪽 90°).
    # 앞뒤 대칭이라 180° 차이는 같음
    yaw_box = (-getattr(C, "PARCEL_YAW", 0.0)) % 360
    m = Mission(drone)

    def log(msg):
        m.log(msg)
        log_lines.append(msg)

    tasks = [asyncio.create_task(m.track())]
    rng = {"d": None}

    async def track_rng():   # 거리계 값 (택배가 거리계를 가리는지 확인용)
        async for ds in drone.telemetry.distance_sensor():
            rng["d"] = ds.current_distance_m
    tasks.append(asyncio.create_task(track_rng()))
    m.att_yaw = None

    async def track_yaw():   # 드론이 믿는 기수 (도, 북=0 시계방향)
        async for eu in drone.telemetry.attitude_euler():
            m.att_yaw = eu.yaw_deg % 360
    tasks.append(asyncio.create_task(track_yaw()))
    while m.pos is None:
        await asyncio.sleep(0.1)
    z0 = m.pos[2]                      # 시작 높이 (다리로 선 높이 ≈ 쥔 택배 바닥이 바닥에 닿는 높이, 차이 4 mm)
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

    async def look(label, alt, eye):
        c, last = await sample(cam, eye)
        if c is None:
            rgb, _ = cam.latest()
            other = []
            if rgb is not None:
                other = sorted({d.tag_id for d in eye.det.detect(rgb.mean(axis=2).astype("uint8"))})
            log(f"{label}: 태그(id {eye.tag_id}) 안 보임" + (f" — 다른 태그 id {other} 는 보임" if other else ""))
            if rgb is not None:
                import cv2
                cv2.imwrite(str(out / f"{label}_못찾음.png"), cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
            return None
        r = math.hypot(c[0], c[1]) * 100
        log(f"{label}: 카메라(그리퍼) 중심 ~ 태그 수평 {r:.1f} cm, 거리 {c[2]:.2f} m")
        rows.append([label, f"{alt:.2f}", f"{m.pos[0]:.3f}", f"{m.pos[1]:.3f}",
                     f"{c[0]:.4f}", f"{c[1]:.4f}", f"{c[2]:.3f}", f"{r:.1f}"])
        eye.draw(last[0], last[1], f"{r:.1f} cm", out / f"{label}.png")
        return c

    async def align(alt, name, eye, c_first=None):
        c_last = None
        for it in range(MAX_TRIES):
            c = c_first if (it == 0 and c_first is not None) else await look(f"{name}_{alt:.1f}m_{it + 1}", alt, eye)
            if c is None:
                break
            c_last = c
            if math.hypot(c[0], c[1]) * 100 < DONE_CM:
                break
            dn, de = cam_to_ne(c, m.sp[3])
            await go(m.sp[0] + dn, m.sp[1] + de, -alt)
        return c_last

    async def low(enable):
        if FLOW_LOW:   # 광류를 낮은 데서도 씀 → IMU만으로 버티는 시간이 없어짐
            await drone.param.set_param_float("SENS_FLOW_MINHGT", FLOW_LOW_HGT if enable else 0.5)
        await drone.param.set_param_int("EKF2_NOAID_TOUT", NOAID_LOW if enable else NOAID_NORMAL)

    async def tag_yaw_err(eye, secs=1.5):
        """태그 방향으로 드론이 믿는 기수의 오차(믿는 기수 - 실제 기수, 도)를 잰다. 못 보면 None.
        11/26 측정K1 로그: 기수를 4.8° 잘못 알고 있어서 5 m 갈 때마다 옆으로 40 cm씩 벗어남 (H 착륙 1.1 m의 원인)."""
        vals = []
        t_end = time.time() + secs
        seen_st = 0
        while time.time() < t_end:
            rgb, st = cam.latest()
            if rgb is not None and st != seen_st:
                seen_st = st
                d = eye.detect(rgb)
                if d is not None and d.pose_R is not None:
                    Rm = np.asarray(d.pose_R)
                    right, fwd = Rm[0, 0], -Rm[1, 0]          # 태그 x축을 드론 기준(오른쪽, 앞)으로
                    vals.append(math.degrees(math.atan2(right, fwd)))
            await asyncio.sleep(0.03)
        if len(vals) < 3:
            return None
        ang = float(np.median(vals))
        h_tag = 90.0 - getattr(C, "PARCEL_YAW", 0.0)          # 태그 x축 = 택배 x축의 실제 방향 (NED, 도)
        est = m.att_yaw if m.att_yaw is not None else m.sp[3]
        best = None
        for kq in range(4):                                     # 90° 단위로 겹치는 것 중 믿는 기수에 가장 가까운 것
            true = (h_tag - ang + 90 * kq) % 360
            err = (est - true + 180) % 360 - 180
            if best is None or abs(err) < abs(best):
                best = err
        return best

    def to_est(X):
        """실제 위치 X(로컬 NED, 출발 S 기준)로 가려면 드론이 믿는 좌표로 어디를 목표로 줘야 하는지.
        마지막 카메라 맞춤 지점(anchor)에서 드론이 믿는 위치와 실제 위치, 기수 오차로 변환."""
        ea, ta, ye = anchor["est"], anchor["true"], math.radians(anchor["yaw_err"])
        dn, de = X[0] - ta[0], X[1] - ta[1]
        return (ea[0] + dn * math.cos(ye) - de * math.sin(ye), ea[1] + dn * math.sin(ye) + de * math.cos(ye))

    async def servo_down(eye, z_end, label, timeout=12.0):
        """카메라로 태그를 계속 보면서 그 위에 머물도록 위치를 고치며 z_end(m, 위가 +)까지 천천히 내려감.
        11/26 기수1 로그: 0.5 m 아래에서 광류만 믿고 내려가는 동안 실제로 30~50 cm 밀림 → 카메라로 바로잡으며 내려감.
        반환: 마지막으로 본 태그 위치(카메라 기준) 또는 None."""
        t_end = time.time() + timeout
        seen_st, last_c, ok_since, last_seen = 0, None, None, None
        zt = m.sp[2]
        while time.time() < t_end:
            zt = min(zt + LOW_SPEED * 0.05, -z_end)          # NED: 위가 -, 천천히 z_end까지
            rgb, st = cam.latest()
            if rgb is not None and st != seen_st:
                seen_st = st
                d = eye.detect(rgb)
                if d is not None:
                    c = d.pose_t.ravel()
                    last_c = c
                    dn, de = cam_to_ne(c, m.sp[3])
                    tn, te = m.pos[0] + dn, m.pos[1] + de        # 태그 바로 위 (드론이 믿는 좌표)
                    m.sp = [0.6 * m.sp[0] + 0.4 * tn, 0.6 * m.sp[1] + 0.4 * te, zt, m.sp[3]]
                    r = math.hypot(c[0], c[1])
                    if abs(m.pos[2] - (-z_end)) < 0.03 and r < 0.015:
                        ok_since = ok_since or time.time()
                        if time.time() - ok_since > 1.0:
                            break
                    else:
                        ok_since = None
                else:
                    m.sp = [m.sp[0], m.sp[1], zt, m.sp[3]]
                    if abs(m.pos[2] - (-z_end)) < 0.03 and last_seen and time.time() - last_seen > 1.0:
                        break                                    # 다 내려왔는데 1초 넘게 못 봄 → 마지막 값으로
                if d is not None:
                    last_seen = time.time()
            await asyncio.sleep(0.05)
        if last_c is not None:
            r = math.hypot(last_c[0], last_c[1]) * 100
            log(f"{label}: 카메라 보며 {z_end:.2f} m까지 내려옴, 그리퍼 중심 ~ 태그 {r:.1f} cm")
        else:
            log(f"{label}: 카메라 보며 내려오는 중 태그 못 봄")
        return last_c

    async def pick(k, name, X, start, eye, child):
        """X(택배 자리) 위에서 카메라로 태그 맞추고 집기. 반환: 광류 누적 오차(북, 동), 단계별 태그 거리."""
        log(f"{name} 도착 (광류 위치), 카메라로 태그(id {eye.tag_id}) 찾기")
        await asyncio.sleep(SETTLE_S)
        p_est = m.pos
        c0 = await look(f"{k}1_{name}도착_1.5m", CRUISE_ALT, eye)
        if c0 is None:   # 못 보면 주변 ±0.4 m 네 곳을 돌며 다시 찾음
            for dn0, de0 in ((0.4, 0), (0, 0.4), (-0.4, 0), (0, -0.4)):
                await go(start[0] + dn0, start[1] + de0, -CRUISE_ALT)
                c0 = await look(f"{k}1_{name}찾기_{dn0:+.1f}_{de0:+.1f}", CRUISE_ALT, eye)
                if c0 is not None:
                    break
        c1 = c2 = c3 = None
        if c0 is not None:
            c1 = await align(CRUISE_ALT, f"{k}2_맞춤", eye, c0)
            await go(m.sp[0], m.sp[1], -LOW_ALT)
            c2 = await align(LOW_ALT, f"{k}3_맞춤", eye)
            # 0.5 m 맞춤 동안 광류를 확실히 켜 둠. 광류 최저 높이가 0.5 m 그대로면 0.5 m에서 광류가 켜졌다 꺼졌다 해서
            # 위치가 흔들리고 맞출수록 더 벌어짐 (11/26 측정1~3: 9.1 → 5.8 → 6.8 → 7.7 → 10.7 cm, 중단)
            await low(True)
            await m.move_to(m.sp[0], m.sp[1], -WORK_ALT, VERT_SPEED)
            await asyncio.sleep(SETTLE_S)
            c3 = await align(WORK_ALT, f"{k}4_맞춤", eye)
            c5 = await look(f"{k}5_맞춤확인_0.5m", WORK_ALT, eye)
            c3 = c5 if c5 is not None else c3
        seen[k] = (c0, c1, c2, c3)
        if c3 is None:   # 0.5 m에서 태그를 못 보면 집지 않음. 몇 cm 어긋난 건 아래에서 카메라 보며 내려가면서 맞춤
            # (11/26 영상용4: 0.5 m 4.1 cm로 멈췄는데, 이제는 0.2 m까지 카메라로 계속 맞추므로 여기서 멈출 필요 없음)
            raise PickFail(f"{name}에서 태그를 못 맞춤 (안 보임) → 집지 않음")
        drift = None
        if c3 is not None:   # 카메라로 맞춘 순간: 실제 위치 = 택배 자리 - (카메라에서 본 태그 방향)
            dn, de = cam_to_ne(c3, m.sp[3])
            drift = (m.pos[0] - (X[0] - dn), m.pos[1] - (X[1] - de))
            log(f"  광류 누적 오차 (믿는 위치 - 실제): 북 {drift[0]*100:+.0f}, 동 {drift[1]*100:+.0f} cm")
            ye = await tag_yaw_err(eye)
            if ye is not None and abs(ye) < 20:
                yaw_errs[k] = ye
                log(f"  기수 오차 (믿는 기수 - 실제, 태그로 잼): {ye:+.1f}°")
            anchor.update(est=m.pos[:2], true=(X[0] - dn, X[1] - de),
                          yaw_err=yaw_errs.get(k, anchor["yaw_err"]))
        # 0.5 m → 택배 높이도 광류(0.15 m까지)를 켠 채로 내려감.
        # 11/26 측정D~F: IMU만으로 내려가면 집는 몇 초 동안 드론이 실제로 0.6~1 m 밀려도 모름 → D·H 오차 1 m.
        #   광류를 켜 두면 택배 근처에서 몇 cm 틀어질 수는 있어도 크게 밀리지는 않음 (11/25 광류낮게: 집는 동안 0~3 cm)
        await low(True)
        c7 = await servo_down(eye, SERVO_END, f"{k}6")
        # 아주 가까운 높이(0.13 m)라 몇 cm는 남을 수 있음 → 손가락 사이 끼워 맞춤(10 cm까지)이 마저 맞춤.
        # 여기서는 크게 벗어난 것만 멈춤 (11/26 영상용7: 5.3 cm로 멈췄음)
        if c7 is None or math.hypot(c7[0], c7[1]) * 100 > SERVO_SAFE_CM:
            raise PickFail(f"{name}에서 카메라 보며 내려오다 태그를 놓침/벗어남 → 집지 않음")
        dn7, de7 = cam_to_ne(c7, m.sp[3])
        anchor.update(est=m.pos[:2], true=(X[0] - dn7, X[1] - de7))    # 집기 직전 위치로 다시 맞춤
        # 0.3 m 아래는 광류 카메라에 택배가 너무 가까이 보여서 속도를 크게 잘못 잼 (11/26 서보1: 0.3 m에서 1.3 cm였는데
        # 광류 켠 채 내려가자 택배가 앞 +15.8 cm에 잡히고, 그 뒤 위치가 2 m 틀어짐) → 짧은 구간만 IMU로 빠르게
        await drone.param.set_param_int("EKF2_NOAID_TOUT", NOAID_LOW)
        await drone.param.set_param_float("SENS_FLOW_MINHGT", DROP_FLOW_OFF)
        log("  택배 쥘 높이까지 빠르게 하강 (마지막 몇 cm만 IMU)")
        # 쥔 위치 보정: IMU만으로 내려가는 동안 늘 뒤로 밀려서 택배가 앞 +9~10 cm에 잡힘 (측정D1 H·P) → 미리 앞으로 옮겨서 내려감.
        #   앞으로 치우치면 택배 모서리가 광류 카메라에 보여서 들고 나는 동안 위치가 틀어짐 (측정D1: D 34 cm, H 107 cm)
        cf, cl = GRAB_SHIFT[k]
        y = math.radians(m.sp[3])
        tn = m.sp[0] + cf * math.cos(y) + cl * math.sin(y)
        te = m.sp[1] + cf * math.sin(y) - cl * math.cos(y)
        m.sp = [tn, te, z0 - GAP, m.sp[3]]
        # 끝까지 내려갈 때까지 기다림 (도착 판정 범위가 넓어서 11/26 측정B: 아직 11 cm 위에 있을 때 붙여 버림
        #   → 택배가 손가락 아래에 매달림)
        t_end = time.time() + 1.5          # IMU만 쓰는 시간을 짧게 (11/26 영상용2: 3.6초 동안 8 cm 넘게 밀림)
        while time.time() < t_end and abs(m.pos[2] - (z0 - GAP)) > 0.02:
            await asyncio.sleep(0.05)
        # 손가락을 먼저 닫아서 택배를 좌우 가운데로 밀어 넣고(손가락 충돌), 그다음 그 자리에서 고정 (11/26 측정C).
        # 11/26 측정C1: 택배가 왼쪽 13 cm에 붙자 왼쪽에 있는 거리계(왼쪽 17.5 cm)를 가려서 높이를 못 잼 → 날아감
        grip.fingers(closed=True)
        await asyncio.sleep(0.7)
        moved = grip.center_box(child, CENTER_MAX)
        if moved is None:
            grip.fingers(closed=False)
            o = grip.offset(child)
            how = f"앞 {o[0]:+.0f}, 왼쪽 {o[1]:+.0f} cm" if o else "Gazebo 위치를 못 받음"
            raise PickFail(f"{name} 택배가 손가락 사이에서 {CENTER_MAX*100:.0f} cm 넘게 벗어남 ({how}) → 집지 않음")
        log(f"  택배를 손가락 사이 가운데로 끼워 맞춤 ({moved*100:.1f} cm)" if moved >= 0
            else "  (가운데로 옮기기 실패: Gazebo set_pose 서비스 없음 → 그대로 붙임)")
        await asyncio.sleep(0.2)
        grip.joint(child, attach=True)
        await asyncio.sleep(0.3)
        o = grip.offset(child)
        if o:
            grab_off[k] = o
            log(f"  [정답] 쥔 위치: 택배 중심이 그리퍼 중심에서 앞 {o[0]:+.1f}, 왼쪽 {o[1]:+.1f} cm, 높이 {o[2]:+.1f} cm")
        # 붙인 뒤에는 손가락을 살짝 풀어 둠 (0.30 → 0.22 rad, 택배 옆면과 틈 약 1.5 cm).
        # 붙인 택배에 손가락이 계속 닿아 있으면 Gazebo가 둘을 계속 밀어내서 드론이 떨림 (11/26 측정J)
        grip.fingers(closed=True, angle=FINGER_HOLD)
        log("  손가락 닫고(가운데 맞춤) 택배 붙임")
        await m.move_to(m.sp[0], m.sp[1], -WORK_ALT, VERT_SPEED)
        await asyncio.sleep(0.5)
        if rng["d"] is not None and rng["d"] < RNG_BLOCKED:
            # 0.5 m 떠 있는데 거리계가 이보다 짧으면 택배가 거리계를 가린 것 (치우쳐 잡힘) → 놓고 내려앉음
            # (11/26 측정I3: 왼쪽 23 cm로 잡혀 거리계가 가려지자 높이를 못 재고 날아감)
            grip.fingers(closed=False)
            grip.joint(child, attach=False)
            raise PickFail(f"{name} 택배가 거리계를 가림 (거리계 {rng['d']:.2f} m) → 택배 놓음")
        await low(False)
        shift = math.dist(p_est[:2], m.sp[:2]) * 100
        await m.move_to(m.sp[0], m.sp[1], -CRUISE_ALT, VERT_SPEED)
        return drift, (c0, c1, c2, c3), shift

    eye_b = TagEye()
    eye_b.tag_id = int(read_cfg(TAG_CFG).get("id_b", 1))
    eye_p = eye                      # id 0 (apriltag_config.txt 의 id)

    drift_b = drift_p = None
    seen, grab_off, drop_err = {}, {}, {}
    yaw_errs = {}
    anchor = {"est": (0.0, 0.0), "true": (0.0, 0.0), "yaw_err": 0.0}   # 아직 못 쟀으면 그대로
    cb = cp = (None, None, None, None)
    shift_b = shift_p = 0.0
    try:
        # 1) 이륙 (빈손), H → B, 기수를 택배 방향으로 한 번 맞추고 카메라로 찾아 집기
        log(f"S에서 이륙 (빈손) → {CRUISE_ALT} m")
        await m.move_to(Sl[0], Sl[1], -CRUISE_ALT, TAKEOFF_SPEED)
        await asyncio.sleep(2.0)
        log(f"S → H 이동 ({math.dist(getattr(C, 'S', C.H), C.B):.1f} m, 첫 택배)")
        await m.move_to(B[0], B[1], -CRUISE_ALT, CRUISE_SPEED, ramp=False)
        if abs((yaw_now - yaw_box + 90) % 180 - 90) > 5:     # 출발 기수가 이미 택배 방향이면 회전 없음
            log("H 위에서 기수를 택배 방향에 맞춤 (집기용, 이후 고정)")
            await m.turn_to(yaw_box)
        drift_b, cb, shift_b = await pick("B", "H", B, B, eye_b, "parcel_H")

        # 2) B → D, 놓기 (B에서 잰 광류 오차 반영)
        log(f"H → D 이동 ({math.dist(C.B, C.D):.1f} m)")
        Dt = to_est(D) if drift_b is not None else D
        await m.move_to(Dt[0], Dt[1], -CRUISE_ALT, CRUISE_SPEED, ramp=False)
        log("D 도착, 내려가서 택배 놓기")
        await m.move_to(m.sp[0], m.sp[1], -WORK_ALT, VERT_SPEED)
        await low(True)
        await m.move_to(m.sp[0], m.sp[1], z0 - GAP, LOW_SPEED)
        await asyncio.sleep(1.5)
        grip.fingers(closed=False)
        await asyncio.sleep(0.8)
        grip.joint("parcel_H", attach=False)
        log(f"  손가락 열고 택배 놓음 (택배 바닥 ~ 바닥 {GAP * 100:.0f} cm)")
        await asyncio.sleep(1.5)
        de_ = grip.ground_err("parcel_H", C.D)
        if de_:
            drop_err["D"] = de_
            log(f"  [정답] 놓은 택배가 D 중심에서 동 {de_[0]:+.0f}, 북 {de_[1]:+.0f} cm ({math.hypot(*de_):.0f} cm)")
        await m.move_to(m.sp[0], m.sp[1], -WORK_ALT, VERT_SPEED)
        await low(False)
        await m.move_to(m.sp[0], m.sp[1], -CRUISE_ALT, VERT_SPEED)

        # 3) D → P, 카메라로 찾아 집기
        log(f"D → P 이동 ({math.dist(C.D, C.P):.1f} m)")
        Pt = to_est(P) if drift_b is not None else P
        await m.move_to(Pt[0], Pt[1], -CRUISE_ALT, CRUISE_SPEED, ramp=False)
        drift_p, cp, shift_p = await pick("P", "P", P, Pt, eye_p, "parcel")

        # 4) P → H, 택배 들고 착륙 (P에서 잰 광류 오차 반영)
        log(f"P → H 이동 ({math.dist(C.P, C.H):.1f} m)")
        Ht = to_est(H) if (drift_p is not None or drift_b is not None) else H
        await m.move_to(Ht[0], Ht[1], -CRUISE_ALT, CRUISE_SPEED, ramp=False)
        log("H 도착, 택배 들고 착륙")
        await asyncio.sleep(2.0)
        if FLOW_LOW:   # 착륙하며 내려오는 마지막 0.5 m에서도 광류로 위치 유지
            await low(True)
    except PickFail as err:
        log(f"중단: {err}. 그 자리에서 빈손으로 착륙")
        await low(True)

    await drone.action.land()
    async for in_air in drone.telemetry.in_air():
        if not in_air:
            break
    log("착륙 완료")
    await asyncio.sleep(1.0)
    he_ = grip.ground_err("x500_mydrone_0", C.H)
    if he_:
        drop_err["H"] = he_
        log(f"  [정답] 착륙한 드론이 H 중심에서 동 {he_[0]:+.0f}, 북 {he_[1]:+.0f} cm ({math.hypot(*he_):.0f} cm)")
    for t in tasks:
        t.cancel()

    with open(out / "detections.csv", "w", newline="", encoding="utf-8") as fcsv:
        w = csv.writer(fcsv)
        w.writerow(["단계", "고도_m", "추정위치_북_m", "추정위치_동_m", "태그_x_m", "태그_y_m", "태그거리_m", "수평오차_cm"])
        w.writerows(rows)
    cc, tc = read_cfg(CAM_CFG), read_cfg(TAG_CFG)
    cb, cp = seen.get("B", cb), seen.get("P", cp)
    hz = lambda c: f"{math.hypot(c[0], c[1]) * 100:.1f} cm" if c is not None else "태그 안 보임"
    dz = lambda d: f"{math.hypot(*d)*100:.0f} cm" if d is not None else "못 잼"
    legs = [math.dist(getattr(C, "S", C.H), C.B), math.dist(C.B, C.D), math.dist(C.D, C.P), math.dist(C.P, C.H)]
    S = [f"그리퍼 배달·수거 미션 ({tag}, 낮은 곳 {'광류 0.15 m까지' if FLOW_LOW else 'IMU만'})",
         "동선: S(빈손 출발) → H(카메라로 찾아 집기) → D(놓기) → P(카메라로 찾아 집기) → H(들고 착륙), "
         + " + ".join(f"{x:.1f}" for x in legs) + f" = {sum(legs):.1f} m, 1.5 m, 0.8 m/s",
         f"카메라: ({float(cc['x'])*1000:.0f}, {float(cc['y'])*1000:.0f}, {float(cc['z'])*1000:.0f}) mm, 가로 화각 {cc['hfov']}° "
         f"(그리퍼 하우징 아래 = 그리퍼 중심)",
         f"태그: {tc['family']} H 택배 id {eye_b.tag_id}, P 택배 id {eye_p.tag_id}, 검은 사각형 {float(tc['black_m']):.3f} m", ""]
    for nm, cs, sh, dr in (("H", cb, shift_b, drift_b), ("P", cp, shift_p, drift_p)):
        S += [f"[{nm}에서 그리퍼(카메라) 중심 ~ 택배 태그 중심 수평 거리]",
              f"  광류 위치로 도착 (1.5 m) : {hz(cs[0])}",
              f"  1.5 m 맞춤 후            : {hz(cs[1])}",
              f"  1.0 m 맞춤 후            : {hz(cs[2])}",
              f"  집기 직전 0.5 m          : {hz(cs[3])}",
              f"  카메라 보고 옮긴 거리    : {sh:.1f} cm",
              f"  잰 광류 누적 오차        : {dz(dr)} ({'D·P' if nm == 'H' else 'H'} 목표에 반영)", ""]
    S += ["[Gazebo 정답]"]
    for k2, nm in (("B", "H 택배"), ("P", "P 택배")):
        if k2 in grab_off:
            o = grab_off[k2]
            S.append(f"  {nm} 쥔 위치 (그리퍼 중심 기준): 앞 {o[0]:+.1f}, 왼쪽 {o[1]:+.1f} cm, 높이 {o[2]:+.1f} cm")
    for k2, nm in (("D", "D에 놓은 택배"), ("H", "H에 착륙한 드론")):
        if k2 in drop_err:
            e2 = drop_err[k2]
            S.append(f"  {nm}: 중심에서 {math.hypot(*e2):.0f} cm (동 {e2[0]:+.0f}, 북 {e2[1]:+.0f})")
    S.append("")
    S += [f"택배 상태 (Gazebo): 첫 택배 {grip.state.get('parcel_H', '?')}, P 택배 {grip.state.get('parcel', '?')}  "
          f"(끝났을 때 첫 택배 detached(D에 놓임), P attached 이면 정상)", "", "진행 기록:"] + [f"  {l}" for l in log_lines]
    (out / "summary.txt").write_text("\n".join(S) + "\n", encoding="utf-8")
    print("\n" + "\n".join(S[:30]))
    print(f"\n저장: {out}  →  SIM에서 pxh> shutdown")


if __name__ == "__main__":
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    if not args:
        sys.exit(__doc__)
    import os
    asyncio.run(main(args[0]))
    os._exit(0)
