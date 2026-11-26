"""센서 3개(광류, 거리계, 아래쪽 카메라) 위치를 바꾸고, PX4 센서 위치 파라미터까지 다시 계산한다.

사용 (단위 mm, 기준 base_link = 모터 4개 가운데·하판 아랫면, 앞 +, 왼쪽 +, 위 +)
  python3 tools/vehicle/set_sensors.py --flow 169 175 -5 --range 189 175 -5 --camera 0 0 -144 90   # 11/24 확정 위치
  python3 tools/vehicle/set_sensors.py --camera 0 0 -144 90         # 바꿀 센서만 줘도 됨
  python3 tools/vehicle/set_sensors.py show                         # 지금 위치 보기

  11/24 선배 확정: 광류·거리계는 M3(앞-왼쪽) 모터 마운트 아랫덮개 (그리퍼가 하판 아래 시야를 가려서),
                   카메라는 그리퍼 하우징 아래 가운데 (IMX219 120°, 가로 화각 90°)
  --flow   x y z        광류 카메라 (PMW3901 역할) 위치. 바닥을 바라봄
  --range  x y z        거리계 (VL53L1X 역할) 위치. 바닥을 바라봄
  --camera x y z pitch  아래쪽 카메라 (IMX219) 위치 + pitch(°, 90 = 바로 아래). 화각 등은 set_camera.py 옵션 그대로

하는 일
  1. models/x500_mydrone/model.sdf 의 광류·거리계 pose와 [위치] 주석을 바꿈
  2. --camera 가 있으면 set_camera.py 를 그 위치로 다시 실행 (카메라 설정 파일 camera_config.txt 도 갱신 → AprilTag 접근 코드가 읽음)
  3. 지금 무게·무게중심·모터·관성·모터 위치 그대로 set_mass_cg.py 를 다시 실행
     → run_mydrone_flow_test.sh 의 EKF2_OF_POS_X/Y/Z, EKF2_RNG_POS_X/Y/Z, EKF2_MIN_RNG 다시 계산 (무게중심 기준)
  끝나면 시뮬을 다시 켜야 적용: bash tools/run_mydrone_flow_test.sh
"""
import re
import subprocess
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
MOTOR_SDF = REPO / "models/x500_mydrone/model.sdf"
BASE_SDF = REPO / "models/x500_mydrone_base/model.sdf"
CAM_CFG = REPO / "tools/vehicle/camera_config.txt"
sys.path.insert(0, str(Path(__file__).resolve().parent))
import set_mass_cg as M  # noqa: E402


def fmt(v_mm):
    return f"{v_mm / 1000:.4f}".rstrip("0").rstrip(".") if v_mm else "0"


def desc(x, y, z):
    side = "" if abs(y) < 0.5 else (f", 왼쪽 {y / 10:g} cm" if y > 0 else f", 오른쪽 {-y / 10:g} cm")
    vert = f"하판 아래 {-z / 10:g} cm" if z <= 0 else f"하판 위 {z / 10:g} cm"
    fb = f"앞 {x / 10:g} cm" if x >= 0 else f"뒤 {-x / 10:g} cm"
    return f"{fb}{side}, {vert}"


def show():
    flow, rng = M.sensor_poses()
    print(f"광류   (앞/왼/위 mm): {tuple(round(v, 1) for v in flow)}")
    print(f"거리계 (앞/왼/위 mm): {tuple(round(v, 1) for v in rng)}")
    if CAM_CFG.exists():
        c = dict(l.split() for l in CAM_CFG.read_text().splitlines() if l.strip())
        print(f"카메라 (앞/왼/위 mm): ({float(c['x']) * 1000:g}, {float(c['y']) * 1000:g}, {float(c['z']) * 1000:g}), pitch {c['pitch']}°")
    else:
        print("카메라: 안 달림 (set_camera.py)")


def pop(args, name, n):
    if name not in args:
        return None
    i = args.index(name)
    v = args[i + 1:i + 1 + n]
    if len(v) != n:
        sys.exit(__doc__)
    del args[i:i + 1 + n]
    return [float(x) for x in v]


def main():
    args = sys.argv[1:]
    if args == ["show"]:
        show()
        return
    flow, rng, cam = pop(args, "--flow", 3), pop(args, "--range", 3), pop(args, "--camera", 4)
    if args or not (flow or rng or cam):
        sys.exit(__doc__)

    s = MOTOR_SDF.read_text()
    if flow:
        s, n1 = re.subn(r"(<uri>model://optical_flow</uri>\s*<pose relative_to=\"base_link\">)[^<]+(</pose>)",
                        lambda m: f"{m.group(1)}{fmt(flow[0])} {fmt(flow[1])} {fmt(flow[2])} 0 0 0{m.group(2)}", s)
        s, n2 = re.subn(r"<!-- \[위치\] 광류 카메라: .*?-->", f"<!-- [위치] 광류 카메라: {desc(*flow)} -->", s)
        if n1 != 1:
            sys.exit("model.sdf에서 광류 pose를 못 찾음")
    if rng:
        s, n1 = re.subn(r'(<link name="lidar_sensor_link">\s*<pose relative_to="base_link">)[^<]+(</pose>)',
                        lambda m: f"{m.group(1)}{fmt(rng[0])} {fmt(rng[1])} {fmt(rng[2])} 0 1.5707 0{m.group(2)}", s)
        s, n2 = re.subn(r"<!-- \[위치\] 거리계: .*?-->", f"<!-- [위치] 거리계: {desc(*rng)}, 아래를 바라봄 -->", s)
        if n1 != 1:
            sys.exit("model.sdf에서 거리계 pose를 못 찾음")
    if flow or rng:
        f_now, r_now = (flow or M.sensor_poses()[0]), (rng or M.sensor_poses()[1])
        same = "두 센서 같은 자리. " if f_now == r_now else ""
        s = re.sub(r"(위치: ).*?(\(10/31 사용자\)|\(set_sensors\.py\))",
                   lambda m: f"{m.group(1)}{same}광류 {desc(*f_now)} / 거리계 {desc(*r_now)}, 바닥을 바라봄 (set_sensors.py)", s, count=1)
        MOTOR_SDF.write_text(s)

    if cam:
        r = subprocess.run([sys.executable, str(REPO / "tools/vehicle/set_camera.py"),
                            "--x", f"{cam[0] / 1000}", "--y", f"{cam[1] / 1000}", "--z", f"{cam[2] / 1000}",
                            "--pitch", f"{cam[3]}"], capture_output=True, text=True)
        print(r.stdout.strip() or r.stderr.strip())
        if r.returncode:
            sys.exit(1)

    # 지금 설정 그대로 set_mass_cg.py 다시 실행 → 센서 위치 파라미터 재계산
    _, _, _, mass, pose, inert = M.read_sdf()
    _, k, w, _ = M.read_motor()
    motor = min(M.MOTORS, key=lambda key: abs(M.MOTORS[key]["k"] - k) + abs(M.MOTORS[key]["wmax"] - w) * 1e-6)
    arm = M.current_arm(BASE_SDF.read_text())
    total = round(mass * 1000 + M.ROTOR_G)
    cmd = [sys.executable, str(REPO / "tools/vehicle/set_mass_cg.py"), f"{total}", f"{pose[0] * 1000:.1f}",
           "--motor", motor, "--arm", f"{arm:.4f}"]
    if abs(inert[0] - 0.0216667 * mass / 2) > 1e-4 or abs(inert[2] - 0.04 * mass / 2) > 1e-4:
        cmd += ["--inertia", *(f"{v:.5f}" for v in inert)]      # 직접 넣은 관성이면 유지
    print("\n지금 설정 그대로 다시 계산:", " ".join(cmd[1:]).replace(str(REPO) + "/", ""))
    r = subprocess.run(cmd, capture_output=True, text=True)
    out = r.stdout.splitlines()
    print("\n".join(l for l in out if "센서 위치" in l or "EKF2_" in l))
    if r.returncode:
        sys.exit(r.stderr)
    print("\n시뮬을 다시 켜야 적용: bash tools/run_mydrone_flow_test.sh")


if __name__ == "__main__":
    main()
