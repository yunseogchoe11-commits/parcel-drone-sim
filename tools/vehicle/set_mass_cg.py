"""시뮬 기체의 총무게·무게중심(x)·모터를 바꾸고, 그에 맞춰 PX4 파라미터를 다시 계산한 실행 스크립트를 만든다.

사용
  python3 tools/vehicle/set_mass_cg.py 2400 -17.7                  # 총무게 2,400 g, 무게중심 x -17.7 mm (앞 +), 모터 PX4 기본
  python3 tools/vehicle/set_mass_cg.py 2600 -14 --motor air2216    # 모터를 AIR2216 II KV920 + T1045 II (14.8 V) 실측값으로
  python3 tools/vehicle/set_mass_cg.py reset                       # 원래 모델(2,283 g, -17.7 mm, PX4 기본 모터)로 되돌림

모터 (--motor, 생략하면 px4)
  px4      : PX4 기본 x500. k 8.54858e-6, 최대 1000 rad/s → 모터당 0.87 kgf, 합계 3.49 kgf
  air2216_low : air2216을 배터리 14.4 V + 손실 10 %로 낮춘 최악 조건 → 모터당 0.97 kgf, 합계 3.90 kgf
  air2216  : T-Motor AIR2216 II KV920 + T1045 II 공식 추력표(16 V, 30~100 %)에 T = k·ω² 맞춤 → k 1.2299e-5
             14.8 V 최대 회전 = 9,857 rpm × 14.8/16 = 9,118 rpm = 955 rad/s → 모터당 1.14 kgf, 합계 4.57 kgf
             반토크 계수 0.0140 (추력표 토크/추력)
             SIM_GZ_EC_MIN/MAX를 150/1000 → 143/955 로 같은 비율로 줄여서 출력-추력 곡선 모양을 그대로 유지
             → THR_MDL_FAC 0.75 (10/13 실험 E) 그대로 쓸 수 있음

하는 일
  1. models/x500_mydrone_base/model.sdf : 몸통 무게·무게중심·관성
     - 몸통 = 총무게 - 로터 4개(64 g), 무게중심 y·z는 parts.csv 계산값 (y +0.1, z -49.6 mm)
     - 관성은 cg_calc.py와 같은 방식 (PX4 기본 x500 관성을 무게 비율로 키움, 근사)
  2. models/x500_mydrone/model.sdf : 모터 4개의 motorConstant, maxRotVelocity, momentConstant
  3. tools/run_mydrone_flow_test.sh 생성 (run_mydrone_flow.sh 복사 + 아래 값만 다시 계산·추가)
     - CA_ROTOR*_PX : 모터 위치를 무게중심 기준으로 (앞 0.174 - cg, 뒤 -0.174 - cg)
     - EKF2_OF_POS_X, EKF2_RNG_POS_X : 광류·거리계 위치 (센서 x 80 mm - cg)
     - EKF2_IMU_POS_X : IMU 위치 (FC x 0 mm - cg)
     - SIM_GZ_EC_MIN1~4, SIM_GZ_EC_MAX1~4 : 모터 최대 회전에 맞춤
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
ROTOR_X = 0.174         # 모터 x 위치 (m, 기체 중심 기준)
FLOW_X_MM = 80.0        # 광류·거리계 x 위치 (mm)
IMU_X_MM = 0.0          # IMU(FC) x 위치 (mm)
G = 9.81

MOTORS = {
    "px4": dict(name="PX4 기본 x500", k=8.54858e-06, wmax=1000.0, km=0.016, ec_min=150, ec_max=1000),
    "air2216": dict(name="AIR2216 II KV920 + T1045 II @14.8 V (T-Motor 공식 추력표)",
                    k=1.2299e-05, wmax=954.8, km=0.0140, ec_min=143, ec_max=955),
    # 최악 조건: 배터리가 14.4 V(셀당 3.6 V)까지 떨어지고 배선·프롭 손실 10 % (발표 설계 기준과 같은 방식)
    "air2216_low": dict(name="AIR2216 II @14.4 V, 손실 10 % (최악 조건)",
                        k=1.2299e-05 * 0.9, wmax=954.8 * 14.4 / 14.8, km=0.0140, ec_min=139, ec_max=929),
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
    return s, a, b, mass, pose


def read_motor():
    s = MOTOR_SDF.read_text()
    get = lambda tag: float(re.search(rf"<{tag}>([^<]+)</{tag}>", s).group(1))
    return s, get("motorConstant"), get("maxRotVelocity"), get("momentConstant")


def params_for(cg_x_mm):
    c = cg_x_mm / 1000
    f = (FLOW_X_MM - cg_x_mm) / 1000
    return {
        "CA_ROTOR0_PX": ROTOR_X - c, "CA_ROTOR2_PX": ROTOR_X - c,      # 앞
        "CA_ROTOR1_PX": -ROTOR_X - c, "CA_ROTOR3_PX": -ROTOR_X - c,    # 뒤
        "EKF2_OF_POS_X": f, "EKF2_RNG_POS_X": f,
        "EKF2_IMU_POS_X": (IMU_X_MM - cg_x_mm) / 1000,
    }


def run_value(text, name):
    m = re.search(rf"^export PX4_PARAM_{name}=([-\d.]+)", text, re.M)
    return float(m.group(1)) if m else None


def main():
    args = sys.argv[1:]
    if args == ["reset"]:
        for f in (SDF, MOTOR_SDF):
            subprocess.run(["git", "-C", str(REPO), "checkout", "--", str(f.relative_to(REPO))], check=True)
        RUN_TEST.unlink(missing_ok=True)
        print("원래 모델로 되돌림 (model.sdf 2개 복구, run_mydrone_flow_test.sh 삭제)")
        print("원래 실행: bash tools/run_mydrone_flow.sh")
        return
    motor_key = "px4"
    if "--motor" in args:
        i = args.index("--motor")
        motor_key = args[i + 1] if i + 1 < len(args) else ""
        del args[i:i + 2]
    if len(args) != 2 or motor_key not in MOTORS:
        sys.exit(__doc__)
    total_g, cg_x = float(args[0]), float(args[1])
    mo = MOTORS[motor_key]

    # 1. 몸통 무게·무게중심
    s, a, b, old_mass, old_pose = read_sdf()
    _, (_, cg_y, cg_z) = parts_cg()
    m = (total_g - ROTOR_G) / 1000
    k = m / 2.0
    new = (f"<inertial><pose>{cg_x/1000:.4f} {cg_y/1000:.4f} {cg_z/1000:.4f} 0 0 0</pose><mass>{m:.3f}</mass><inertia>"
           f"<ixx>{0.0216667*k:.5f}</ixx><ixy>0</ixy><ixz>0</ixz><iyy>{0.0216667*k:.5f}</iyy><iyz>0</iyz>"
           f"<izz>{0.04*k:.5f}</izz></inertia></inertial>")
    SDF.write_text(s[:a] + new + s[b:])

    # 2. 모터
    ms, old_k, old_w, old_km = read_motor()
    for tag, v in (("motorConstant", f"{mo['k']:.5e}"), ("maxRotVelocity", f"{mo['wmax']:.1f}"),
                   ("momentConstant", f"{mo['km']:.4f}")):
        ms, cnt = re.subn(rf"<{tag}>[^<]+</{tag}>", f"<{tag}>{v}</{tag}>", ms)
        if cnt != 4:
            sys.exit(f"x500_mydrone/model.sdf 에서 {tag}가 4개가 아님 ({cnt}개). 모델이 바뀌었는지 확인")
    MOTOR_SDF.write_text(ms)

    # 3. 실행 스크립트
    run = RUN.read_text()
    new_p = params_for(cg_x)
    old_p = {n: run_value(run, n) for n in new_p}
    for n, v in new_p.items():
        run, cnt = re.subn(rf"^(export PX4_PARAM_{n}=)[-\d.]+", rf"\g<1>{v:.3f}", run, flags=re.M)
        if cnt != 1:
            sys.exit(f"run_mydrone_flow.sh 에서 {n} 줄을 못 찾음. 스크립트가 바뀌었는지 확인")
    ec = "\n# 모터 회전 범위 (set_mass_cg.py --motor " + motor_key + ")\n" + "".join(
        f"export PX4_PARAM_SIM_GZ_EC_MIN{i}={mo['ec_min']}\nexport PX4_PARAM_SIM_GZ_EC_MAX{i}={mo['ec_max']}\n"
        for i in range(1, 5))
    run, cnt = re.subn(r"^(export PX4_PARAM_THR_MDL_FAC=.*\n)", lambda mm: mm.group(1) + ec, run, flags=re.M)
    if cnt != 1:
        sys.exit("run_mydrone_flow.sh 에서 THR_MDL_FAC 줄을 못 찾음")
    run = run.replace("# 우리 기체(x500_mydrone)",
                      f"# [set_mass_cg.py로 생성: 총 {total_g:.0f} g, 무게중심 x {cg_x:+.1f} mm, 모터 {motor_key}] 우리 기체(x500_mydrone)", 1)
    run = run.replace("./build/px4_sitl_default/bin/px4\n", "./build/px4_sitl_default/bin/px4 ${PX4_ARGS:-}\n")
    RUN_TEST.write_text(run)
    RUN_TEST.chmod(0o755)

    tmax_old = 4 * old_k * old_w ** 2 / G
    tmax = 4 * mo["k"] * mo["wmax"] ** 2 / G
    lines = [
        f"시뮬 기체 설정 변경 (set_mass_cg.py {total_g:.0f} {cg_x:+.1f} --motor {motor_key})",
        "",
        "[Gazebo 모델 x500_mydrone_base/model.sdf]",
        f"  총무게      {old_mass*1000 + ROTOR_G:7.0f} g  ->  {total_g:7.0f} g   (몸통 {old_mass:.3f} -> {m:.3f} kg)",
        f"  무게중심 x  {old_pose[0]*1000:+7.1f} mm ->  {cg_x:+7.1f} mm  (y {cg_y:+.1f}, z {cg_z:+.1f} mm 유지)",
        f"  관성 Ixx=Iyy {0.0216667*old_mass/2:.5f} -> {0.0216667*k:.5f}, Izz {0.04*old_mass/2:.5f} -> {0.04*k:.5f} (무게 비율 근사)",
        "",
        f"[Gazebo 모터 x500_mydrone/model.sdf] {mo['name']}",
        f"  motorConstant   {old_k:.5e} -> {mo['k']:.5e}",
        f"  maxRotVelocity  {old_w:7.1f} -> {mo['wmax']:7.1f} rad/s",
        f"  momentConstant  {old_km:.4f} -> {mo['km']:.4f}",
        f"  최대 추력 합계  {tmax_old:.2f} -> {tmax:.2f} kgf  (모터당 {tmax/4*1000:.0f} g)",
        f"  예상 호버 추력  {total_g/1000/tmax*100:.1f} %  (= 총무게 / 최대 추력 합계)",
        "",
        "[PX4 파라미터 (tools/run_mydrone_flow_test.sh)]",
    ]
    for n, v in new_p.items():
        o = old_p[n]
        mark = "" if o is not None and abs(o - v) < 0.0005 else "   <- 변경"
        lines.append(f"  {n:15s} {o:+.3f} -> {v:+.3f}{mark}")
    mark = "" if motor_key == "px4" else "   <- 변경"
    lines += [
        f"  SIM_GZ_EC_MIN1~4  150 -> {mo['ec_min']}{mark}",
        f"  SIM_GZ_EC_MAX1~4  1000 -> {mo['ec_max']}{mark}",
        "  THR_MDL_FAC     0.75 그대로 (EC_MIN/MAX를 같은 비율로 줄여 곡선 모양 유지)",
        "  MPC_THR_HOVER   그대로 (PX4가 비행 중 자동 추정)",
        "",
        "실행: bash tools/run_mydrone_flow_test.sh",
        "되돌리기: python3 tools/vehicle/set_mass_cg.py reset",
    ]
    text = "\n".join(lines)
    print(text)
    RECORD.write_text(text + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
