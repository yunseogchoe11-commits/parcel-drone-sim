"""시뮬 기체의 총무게·무게중심(x)·모터(·관성·모터 위치)를 바꾸고, 그에 맞춰 PX4 파라미터를 다시 계산한 실행 스크립트를 만든다.

사용
  python3 tools/vehicle/set_mass_cg.py 2600 -11.9                      # 총무게 2,600 g, 무게중심 x -11.9 mm (앞 +), 실제 모터
  python3 tools/vehicle/set_mass_cg.py 2600 -11.9 --motor air2216_low  # 저전압·손실 최악 조건 모터로
  python3 tools/vehicle/set_mass_cg.py 2600 -11.9 --inertia 0.0450 0.0453 0.0424   # 관성(ixx iyy izz, kg·m²)을 직접 넣음
  python3 tools/vehicle/set_mass_cg.py 2600 -11.9 --arm 0.177          # 모터 위치(중심~모터 x, y 거리 m)를 바꿈 (기본 0.174)
  python3 tools/vehicle/set_mass_cg.py 2600 -12.1 --cg-y -1.0 --cg-z -34.8   # 무게중심 y(왼쪽 +)·z(위 +)도 바꿈 (mm)
  python3 tools/vehicle/set_mass_cg.py 2600 -12.1 --ground -270        # 착지했을 때 바닥 높이 (mm, 하판 아랫면 기준, 아래 -)
                                                                       #  그리퍼+택배: -270 (택배 바닥), 그리퍼만: -254, x500 다리: -240
  ※ 그리퍼를 붙였으면(set_gripper.py) 이 도구를 실행한 뒤 set_gripper.py 를 다시 실행
  python3 tools/vehicle/set_mass_cg.py reset                           # 원래 모델로 되돌림 (git 기준)
  옵션은 같이 써도 됨. 생략하면: --motor air2216, 관성은 무게 비율 근사, --arm·--cg-y·--cg-z는 지금 모델 값 그대로,
  --ground는 마지막으로 준 값 (없으면 -240 = Gazebo 기본 다리). 기준점(base_link) = 모터 4개 가운데 · 하판 아랫면

모터 (--motor, 생략하면 air2216 = 실제 모터)
  air2216     : 실제 모터 T-Motor AIR2216 II KV920 + T1045 II (11/22 확인). 제조사 공식 추력표(16 V, 30~100 % 15점)에
                T = k·ω² 맞춤 → k 1.2299e-5. 14.8 V 최대 회전 = 9,857 rpm × 14.8/16 = 9,118 rpm = 955 rad/s
                → 모터당 1.14 kgf, 합계 4.57 kgf. 반토크 계수 0.0140 (추력표 토크/추력)
                SIM_GZ_EC_MIN/MAX를 150/1000 → 143/955 로 같은 비율로 줄여 출력-추력 곡선 모양 유지 → THR_MDL_FAC 0.75 그대로
  air2216_low : 최악 조건. 배터리 14.4 V(셀당 3.6 V) + 손실 10 % → 모터당 0.97 kgf, 합계 3.90 kgf (비행 막바지)
  px4         : PX4 기본 x500 (옛 모델 비교용). k 8.54858e-6, 최대 1000 rad/s → 합계 3.49 kgf

하는 일
  1. models/x500_mydrone_base/model.sdf : 몸통 무게·무게중심·관성 (+ --arm이면 로터 위치)
     - 몸통 = 총무게 - 로터 4개(64 g). 무게중심 y·z는 --cg-y/--cg-z (없으면 지금 모델 값 유지)
       (11/24 선배 계산: 택배 2호 포함 2,596 g → (-12.1, -1.0, -34.8) mm, 택배 없음 → (-13.1, -1.1, -19.3) mm)
     - 관성: --inertia 값, 없으면 PX4 기본 x500 관성을 무게 비율로 키운 근사
  2. models/x500_mydrone/model.sdf : 모터 4개의 motorConstant, maxRotVelocity, momentConstant
  3. tools/run_mydrone_flow_test.sh 생성 (run_mydrone_flow.sh 복사 + 아래 값·주석만 다시 계산·추가)
     - CA_ROTOR*_PX, _PY : 모터 위치를 무게중심 기준으로 (PX: 앞 arm - cg_x, 뒤 -arm - cg_x / PY: 오른쪽 arm + cg_y, 왼쪽 -arm + cg_y)
     - EKF2_OF_POS_X/Y/Z, EKF2_RNG_POS_X/Y/Z : 광류·거리계 위치 - 무게중심 (FRD: 앞, 오른쪽, 아래)
     - EKF2_IMU_POS_X/Y/Z : IMU(FC, 기준점 위 36 mm) - 무게중심
     - EKF2_MIN_RNG : 거리계 높이 - 바닥 (--ground)
     - SIM_GZ_EC_MIN1~4, SIM_GZ_EC_MAX1~4 : 모터 최대 회전에 맞춤
     - BAT1_CAPACITY 5200 (VEGA 4S 5200 mAh)
     원본 run_mydrone_flow.sh 는 건드리지 않는다.
  4. 바뀐 값 표를 화면에 출력 + tools/vehicle/last_setting.txt 에 저장 (보고용)

비행: bash tools/run_mydrone_flow_test.sh  →  python3 scripts/hover_test.py  →  pxh> shutdown
분석: python3 tools/analysis/analyze_hover.py 결과폴더이름
"""
import csv
import re
import subprocess
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
SDF = REPO / "models/x500_mydrone_base/model.sdf"
MOTOR_SDF = REPO / "models/x500_mydrone/model.sdf"
RUN = REPO / "tools/run_mydrone_flow.sh"
RUN_TEST = REPO / "tools/run_mydrone_flow_test.sh"
RECORD = REPO / "tools/vehicle/last_setting.txt"

ROTOR_G = 64.0          # 로터 4개 (model.sdf rotor 링크 0.016 kg x 4)
ARM_DEFAULT = 0.174     # 모터 x, y 위치 (m, 기체 중심 기준). PX4 기본. CAD 실측은 0.1771
IMU_MM = (0.0, 0.0, 36.0)   # IMU(FC) 위치 (앞/왼/위 mm, base_link 기준. 상판 위 FC 높이 추정)
GROUND_DEFAULT_MM = -240.0  # 착륙했을 때 바닥 높이 (base_link 기준, Gazebo 기본 x500 다리) → EKF2_MIN_RNG
GROUND_FILE = REPO / "tools/vehicle/ground_mm.txt"   # --ground 로 준 값을 기억 (set_sensors.py 재실행 때도 유지)
BAT_MAH = 5200
G = 9.81

MOTORS = {
    "air2216": dict(name="실제 모터 AIR2216 II KV920 + T1045 II @14.8 V (T-Motor 공식 추력표)",
                    k=1.2299e-05, wmax=954.8, km=0.0140, ec_min=143, ec_max=955),
    "air2216_low": dict(name="실제 모터 @14.4 V + 손실 10 % (최악 조건)",
                        k=1.2299e-05 * 0.9, wmax=954.8 * 14.4 / 14.8, km=0.0140, ec_min=139, ec_max=929),
    "px4": dict(name="PX4 기본 x500 (옛 모델)", k=8.54858e-06, wmax=1000.0, km=0.016, ec_min=150, ec_max=1000),
}


def parts_cg():
    rows = list(csv.reader(open(REPO / "tools/vehicle/parts.csv", encoding="utf-8")))[1:]
    m = sum(float(r[1]) for r in rows)
    return m, [sum(float(r[1]) * float(r[i]) for r in rows) / m for i in (2, 3, 4)]


def read_sdf():
    s = SDF.read_text()
    i = re.search(r"<link name=[\"']base_link[\"']>", s).end()
    a = s.index("<inertial>", i)
    b = s.index("</inertial>", a) + len("</inertial>")
    block = s[a:b]
    pose = [float(v) for v in re.search(r"<pose>([^<]+)</pose>", block).group(1).split()[:3]]
    mass = float(re.search(r"<mass>([^<]+)</mass>", block).group(1))
    inert = [float(re.search(rf"<{t}>([^<]+)</{t}>", block).group(1)) for t in ("ixx", "iyy", "izz")]
    return s, a, b, mass, pose, inert


def current_arm(s):
    return float(re.search(r'<link name="rotor_0">.*?<pose>([\d.]+)', s, re.S).group(1))


def sensor_poses():
    """x500_mydrone/model.sdf에서 광류·거리계 위치 (mm, base_link 기준 앞/왼/위). set_sensors.py가 바꿈."""
    s = MOTOR_SDF.read_text()
    f = re.search(r"<uri>model://optical_flow</uri>\s*<pose relative_to=\"base_link\">([^<]+)</pose>", s)
    r = re.search(r'<link name="lidar_sensor_link">\s*<pose relative_to="base_link">([^<]+)</pose>', s)
    to_mm = lambda m: [float(v) * 1000 for v in m.group(1).split()[:3]]
    return to_mm(f), to_mm(r)


def read_motor():
    s = MOTOR_SDF.read_text()
    get = lambda tag: float(re.search(rf"<{tag}>([^<]+)</{tag}>", s).group(1))
    return s, get("motorConstant"), get("maxRotVelocity"), get("momentConstant")


def ground_mm():
    return float(GROUND_FILE.read_text().split()[0]) if GROUND_FILE.exists() else GROUND_DEFAULT_MM


def params_for(cg, arm, flow, rng, ground=None):
    """cg, flow, rng: (앞, 왼, 위) mm. PX4 위치는 무게중심 기준 FRD (앞 +, 오른쪽 +, 아래 +, m)."""
    ground = ground_mm() if ground is None else ground
    cx, cy = cg[0] / 1000, cg[1] / 1000
    frd = lambda p: ((p[0] - cg[0]) / 1000, -(p[1] - cg[1]) / 1000, -(p[2] - cg[2]) / 1000)
    fo, ro, io = frd(flow), frd(rng), frd(IMU_MM)
    return {
        "CA_ROTOR0_PX": arm - cx, "CA_ROTOR2_PX": arm - cx,      # 앞
        "CA_ROTOR1_PX": -arm - cx, "CA_ROTOR3_PX": -arm - cx,    # 뒤
        "CA_ROTOR0_PY": arm + cy, "CA_ROTOR1_PY": -arm + cy,     # 0 앞-오른쪽, 1 뒤-왼쪽
        "CA_ROTOR2_PY": -arm + cy, "CA_ROTOR3_PY": arm + cy,     # 2 앞-왼쪽, 3 뒤-오른쪽
        "EKF2_OF_POS_X": fo[0], "EKF2_OF_POS_Y": fo[1], "EKF2_OF_POS_Z": fo[2],
        "EKF2_RNG_POS_X": ro[0], "EKF2_RNG_POS_Y": ro[1], "EKF2_RNG_POS_Z": ro[2],
        "EKF2_IMU_POS_X": io[0], "EKF2_IMU_POS_Y": io[1], "EKF2_IMU_POS_Z": io[2],
        "EKF2_MIN_RNG": (rng[2] - ground) / 1000,
    }


# (설명, 부호): 주석에 쓸 말. 부호 -1이면 값의 반대 (아래 + → 위)
COMMENT = {"EKF2_OF_POS_X": ("광류: 앞", 1), "EKF2_OF_POS_Y": ("광류: 오른쪽", 1), "EKF2_OF_POS_Z": ("광류: 위", -1),
           "EKF2_RNG_POS_X": ("거리계: 앞", 1), "EKF2_RNG_POS_Y": ("거리계: 오른쪽", 1), "EKF2_RNG_POS_Z": ("거리계: 위", -1),
           "EKF2_IMU_POS_X": ("IMU: 앞", 1), "EKF2_IMU_POS_Y": ("IMU: 오른쪽", 1), "EKF2_IMU_POS_Z": ("IMU: 위", -1)}


def run_value(text, name):
    m = re.search(rf"^export PX4_PARAM_{name}=([-\d.]+)", text, re.M)
    return float(m.group(1)) if m else None


def pop_opt(args, name, n):
    if name not in args:
        return None
    i = args.index(name)
    vals = args[i + 1:i + 1 + n]
    if len(vals) != n:
        sys.exit(__doc__)
    del args[i:i + 1 + n]
    return vals


def main():
    args = sys.argv[1:]
    if args == ["reset"]:
        for f in (SDF, MOTOR_SDF):
            subprocess.run(["git", "-C", str(REPO), "checkout", "--", str(f.relative_to(REPO))], check=True)
        RUN_TEST.unlink(missing_ok=True)
        GROUND_FILE.unlink(missing_ok=True)
        (REPO / "tools/vehicle/spawn_z.txt").unlink(missing_ok=True)
        print("원래 모델로 되돌림 (git 기준 model.sdf 2개 복구, run_mydrone_flow_test.sh·ground_mm.txt 삭제)")
        print("주의: 카메라(set_camera.py)를 달았었다면 다시 실행")
        return
    motor_key = (pop_opt(args, "--motor", 1) or ["air2216"])[0]
    inertia = pop_opt(args, "--inertia", 3)
    arm_opt = pop_opt(args, "--arm", 1)
    cgy_opt, cgz_opt = pop_opt(args, "--cg-y", 1), pop_opt(args, "--cg-z", 1)
    ground_opt = pop_opt(args, "--ground", 1)
    if len(args) != 2 or motor_key not in MOTORS:
        sys.exit(__doc__)
    total_g, cg_x = float(args[0]), float(args[1])
    mo = MOTORS[motor_key]

    # 1. 몸통 무게·무게중심·관성 (+ 로터 위치)
    s, a, b, old_mass, old_pose, old_in = read_sdf()
    cg_y = float(cgy_opt[0]) if cgy_opt else old_pose[1] * 1000
    cg_z = float(cgz_opt[0]) if cgz_opt else old_pose[2] * 1000
    old_ground = ground_mm()
    ground = float(ground_opt[0]) if ground_opt else old_ground
    if ground_opt:
        GROUND_FILE.write_text(f"{ground:g}  # 착지 때 바닥 높이 mm (base_link 기준). set_mass_cg.py --ground\n")
    m = (total_g - ROTOR_G) / 1000
    if inertia:
        ixx, iyy, izz = (float(v) for v in inertia)
        in_src = "직접 입력 (--inertia)"
    else:
        k = m / 2.0
        ixx, iyy, izz = 0.0216667 * k, 0.0216667 * k, 0.04 * k
        in_src = "PX4 기본 x500 관성 × 무게 비율 (근사)"
    new = (f"<inertial><pose>{cg_x/1000:.4f} {cg_y/1000:.4f} {cg_z/1000:.4f} 0 0 0</pose><mass>{m:.3f}</mass><inertia>"
           f"<ixx>{ixx:.5f}</ixx><ixy>0</ixy><ixz>0</ixz><iyy>{iyy:.5f}</iyy><iyz>0</iyz>"
           f"<izz>{izz:.5f}</izz></inertia></inertial>")
    s = s[:a] + new + s[b:]
    old_arm = current_arm(s)
    arm = float(arm_opt[0]) if arm_opt else old_arm
    if abs(arm - old_arm) > 1e-6:
        o = f"{old_arm:g}".lstrip("0") if old_arm < 1 else f"{old_arm:g}"
        pat = re.compile(rf"(-?)0?{re.escape(o)} (-?)0?{re.escape(o)} ")
        s, cnt = pat.subn(lambda mm: f"{mm.group(1)}{arm:.4f} {mm.group(2)}{arm:.4f} ", s)
        if cnt < 4:
            sys.exit(f"로터 위치 {old_arm}를 model.sdf에서 4곳 이상 못 찾음 ({cnt}곳)")
    SDF.write_text(s)

    # 2. 모터
    ms, old_k, old_w, old_km = read_motor()
    for tag, v in (("motorConstant", f"{mo['k']:.5e}"), ("maxRotVelocity", f"{mo['wmax']:.1f}"),
                   ("momentConstant", f"{mo['km']:.4f}")):
        ms, cnt = re.subn(rf"<{tag}>[^<]+</{tag}>", f"<{tag}>{v}</{tag}>", ms)
        if cnt != 4:
            sys.exit(f"x500_mydrone/model.sdf 에서 {tag}가 4개가 아님 ({cnt}개). 모델이 바뀌었는지 확인")
    MOTOR_SDF.write_text(ms)

    # 3. 실행 스크립트 (값 + 주석)
    run = RUN.read_text()
    flow, rng = sensor_poses()
    new_p = params_for((cg_x, cg_y, cg_z), arm, flow, rng, ground)
    old_p = {n: run_value(run, n) for n in new_p}
    for n, v in new_p.items():
        if n in COMMENT:
            label, val = COMMENT[n][0], COMMENT[n][1] * v * 100
            if val < -0.05 and label.endswith(("오른쪽", "위")):
                label, val = label.replace("오른쪽", "왼쪽").replace("위", "아래"), -val
            note = f"  # {label} {val:.1f} cm"
        elif n == "EKF2_MIN_RNG":
            note = "  # 땅에 있을 때 거리계 값 (거리계 높이 - 바닥, m)"
        else:
            note = None

        def rep(mm, v=v, note=note):
            tail = note if note else mm.group(2)
            return f"{mm.group(1)}{v:.3f}{tail}"
        run, cnt = re.subn(rf"^(export PX4_PARAM_{n}=)[-\d.]+(.*)$", rep, run, flags=re.M)
        if cnt != 1:
            sys.exit(f"run_mydrone_flow.sh 에서 {n} 줄을 못 찾음. 스크립트가 바뀌었는지 확인")
    run = re.sub(r"^#   무게중심 \(.*$", f"#   무게중심 (앞/왼쪽/위 mm): ({cg_x:.1f}, {cg_y:.1f}, {cg_z:.1f})  [set_mass_cg.py로 생성]",
                 run, count=1, flags=re.M)
    run = re.sub(r"^#   광류·거리계: .*$", f"#   광류 (앞/왼/위 mm): ({flow[0]:.0f}, {flow[1]:.0f}, {flow[2]:.0f}), 거리계: ({rng[0]:.0f}, {rng[1]:.0f}, {rng[2]:.0f})"
                 f"  -> 무게중심 기준 광류 앞 {flow[0] - cg_x:.1f} mm, 위 {flow[2] - cg_z:.1f} mm", run, count=1, flags=re.M)
    run = re.sub(r"^#   IMU\(FC\): .*$", f"#   IMU(FC):     ({IMU_MM[0]:.0f}, {IMU_MM[1]:.0f}, {IMU_MM[2]:.0f})    -> 무게중심 기준 앞 {IMU_MM[0] - cg_x:.1f} mm, "
                 f"위 {IMU_MM[2] - cg_z:.1f} mm (FC 높이 추정 포함)", run, count=1, flags=re.M)
    run = re.sub(r"^# 무게중심 보정 \(.*$", "# 무게중심 보정 (모터 위치를 무게중심 기준으로. set_mass_cg.py가 계산)", run, count=1, flags=re.M)
    run = re.sub(r"^#   모터 위치를 무게중심 기준으로 입력.*$",
                 f"#   모터 위치를 무게중심 기준으로 입력 (앞 +, 오른쪽 +, m). 무게중심 x {cg_x:+.1f} mm, 모터 ±{arm:.3f} m"
                 f" -> 앞 {arm:.3f}{-cg_x / 1000:+.4f}, 뒤 {-arm:.3f}{-cg_x / 1000:+.4f}, 좌우 ±{arm:.3f}{cg_y / 1000:+.4f}", run, count=1, flags=re.M)
    extra = (f"\n# 모터 회전 범위 (set_mass_cg.py --motor {motor_key})\n" +
             "".join(f"export PX4_PARAM_SIM_GZ_EC_MIN{i}={mo['ec_min']}\nexport PX4_PARAM_SIM_GZ_EC_MAX{i}={mo['ec_max']}\n"
                     for i in range(1, 5)) +
             f"export PX4_PARAM_BAT1_N_CELLS=4\nexport PX4_PARAM_BAT1_CAPACITY={BAT_MAH}   # 배터리 VEGA 4S {BAT_MAH} mAh\n")
    run, cnt = re.subn(r"^(export PX4_PARAM_THR_MDL_FAC=.*\n)", lambda mm: mm.group(1) + extra, run, flags=re.M)
    if cnt != 1:
        sys.exit("run_mydrone_flow.sh 에서 THR_MDL_FAC 줄을 못 찾음")
    run = run.replace("./build/px4_sitl_default/bin/px4\n", "./build/px4_sitl_default/bin/px4 ${PX4_ARGS:-}\n")
    run = run.replace("# 우리 기체(x500_mydrone)",
                      f"# [set_mass_cg.py로 생성: 총 {total_g:.0f} g, 무게중심 x {cg_x:+.1f} mm, 모터 {motor_key}, 모터 위치 ±{arm:.3f} m] 우리 기체(x500_mydrone)", 1)
    RUN_TEST.write_text(run)
    RUN_TEST.chmod(0o755)

    tmax_old = 4 * old_k * old_w ** 2 / G
    tmax = 4 * mo["k"] * mo["wmax"] ** 2 / G
    cmd = " ".join(sys.argv[1:])
    lines = [
        f"시뮬 기체 설정 변경 (set_mass_cg.py {cmd})",
        "",
        "[Gazebo 모델 x500_mydrone_base/model.sdf]",
        f"  총무게      {old_mass*1000 + ROTOR_G:7.0f} g  ->  {total_g:7.0f} g   (몸통 {old_mass:.3f} -> {m:.3f} kg)",
        f"  무게중심 x  {old_pose[0]*1000:+7.1f} mm ->  {cg_x:+7.1f} mm",
        f"  무게중심 y  {old_pose[1]*1000:+7.1f} mm ->  {cg_y:+7.1f} mm  (왼쪽 +)",
        f"  무게중심 z  {old_pose[2]*1000:+7.1f} mm ->  {cg_z:+7.1f} mm  (위 +, 하판 아랫면 기준)",
        f"  관성 ixx {old_in[0]:.5f} -> {ixx:.5f}, iyy {old_in[1]:.5f} -> {iyy:.5f}, izz {old_in[2]:.5f} -> {izz:.5f}  [{in_src}]",
        f"  모터 위치   ±{old_arm:.4f} -> ±{arm:.4f} m",
        f"  센서 위치 (앞/왼/위 mm, set_sensors.py가 바꿈): 광류 {tuple(round(v) for v in flow)}, 거리계 {tuple(round(v) for v in rng)}",
        f"  착지 바닥 높이 {old_ground:+.0f} -> {ground:+.0f} mm (EKF2_MIN_RNG 계산용, --ground)",
        "",
        f"[Gazebo 모터 x500_mydrone/model.sdf] {mo['name']}",
        f"  motorConstant   {old_k:.5e} -> {mo['k']:.5e}",
        f"  maxRotVelocity  {old_w:7.1f} -> {mo['wmax']:7.1f} rad/s",
        f"  momentConstant  {old_km:.4f} -> {mo['km']:.4f}",
        f"  최대 추력 합계  {tmax_old:.2f} -> {tmax:.2f} kgf  (모터당 {tmax/4*1000:.0f} g)",
        f"  예상 호버 추력  {total_g/1000/tmax*100:.1f} %  (= 총무게 / 최대 추력 합계)",
        "",
        "[PX4 파라미터 (tools/run_mydrone_flow_test.sh), 원본 run_mydrone_flow.sh 대비]",
    ]
    for n, v in new_p.items():
        o = old_p[n]
        mark = "" if o is not None and abs(o - v) < 0.0005 else "   <- 변경"
        lines.append(f"  {n:15s} {o:+.3f} -> {v:+.3f}{mark}")
    lines += [
        f"  SIM_GZ_EC_MIN1~4  150 -> {mo['ec_min']}" + ("" if motor_key == "px4" else "   <- 변경"),
        f"  SIM_GZ_EC_MAX1~4  1000 -> {mo['ec_max']}" + ("" if motor_key == "px4" else "   <- 변경"),
        f"  BAT1_CAPACITY   -1 -> {BAT_MAH}   <- 변경 (BAT1_N_CELLS 4)",
        "  THR_MDL_FAC     0.75 그대로 (EC_MIN/MAX를 같은 비율로 줄여 곡선 모양 유지)",
        "  MPC_THR_HOVER   그대로 (PX4가 비행 중 자동 추정). MC_·MPC_ 게인 그대로",
        "",
        "실행: bash tools/run_mydrone_flow_test.sh",
        "되돌리기: python3 tools/vehicle/set_mass_cg.py reset",
    ]
    text = "\n".join(lines)
    print(text)
    RECORD.write_text(text + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
