"""그리퍼(선배 SolidWorks 조립품)를 Gazebo 기체 모델에 붙이기 / 떼기.

  python3 tools/vehicle/set_gripper.py            # 붙이기 (다시 실행해도 됨)
  python3 tools/vehicle/set_gripper.py remove     # 떼기 (원래 x500 다리·모양으로)
  python3 tools/vehicle/set_gripper.py --keep-x500-look   # 기체 모양은 x500 그대로 두고 그리퍼만

순서 (중요): set_mass_cg.py → set_sensors.py → set_gripper.py (마지막).
  set_mass_cg.py / set_sensors.py 를 다시 실행하면 몸통 무게가 다시 '그리퍼·택배 포함'으로 돌아가므로
  그 다음에 set_gripper.py 를 다시 실행한다.

모양 (models/x500_mydrone/meshes/, 11/24 선배 STEP에서 변환)
  gripper_mount      장착판 (Arm_plate, 잠금핀 서보)        — 기체에 고정
  gripper_body       하우징·어깨 팔·기어·연결부              — 고정 (어깨 0°. ±30° 관절은 나중에)
  gripper_leg_*      착륙 다리 4개 (양쪽 손가락)             — 고정 (편 상태 = 착륙 다리)
  gripper_finger_l/r 가운데 손가락 2개                       — 관절 (앞뒤 방향 축), 열기·닫기
  frame_cad          기체 (X500 V2 프레임, 젯슨, 픽스호크, 배터리, 센서) — 보이기만 (x500 다리 없는 실제 모양)
  기준: base_link = 모터 4개 가운데 · 하판 아랫면 (SolidWorks 그리퍼 Arm_plate 윗면 = 하판 아랫면, 드론 total.STEP으로 확인)

무게 (그리퍼 721.5 g, 선배 11/24 집계. 부품별은 CAD 부피 비율로 나눔 = 추정)
  몸통(base_link) = 지금 몸통 - 택배 0.2 kg - 그리퍼 0.7215 kg
  몸통 무게중심은 '드론 전체(택배 없이)' 무게중심이 선배 계산 (-13.1, -1.1, -19.3) mm가 되도록 다시 계산 (--cg-np 로 바꿈)
  택배는 이제 진짜 물체 (월드의 parcel_H, parcel). 잡으면 기체에 붙고(DetachableJoint), 놓으면 떨어짐
  PX4 파라미터(CA_ROTOR, EKF2 센서 위치)는 그대로 = 택배 들었을 때(2.6 kg) 기준

충돌 (바닥에 닿는 것)
  x500 다리 충돌 4개를 끄고, 그리퍼 다리 끝 4곳에 작은 공(반지름 12 mm)을 둠 → 다리 끝 = 하판 아래 254 mm
  택배를 들고 있으면 택배 바닥(하판 아래 270 mm)이 먼저 닿음 → 시작할 때 드론은 택배 위에 얹혀 있음

택배 잡는 위치: 택배 가운데가 하판 아래 0.21 m (윗면 = 카메라 바로 아래)  → --hold-z 로 바꿈
드론 생성 높이: tools/vehicle/spawn_z.txt (= 0.21 + 택배 높이/2 - 0.24). run_mydrone_flow*.sh가 읽음
  (x500 모델 원점은 바닥, base_link는 그 위 0.24 m. 11/25 첫 시험에서 이걸 빠뜨려 그리퍼가 0.24 m 아래 바닥에 붙어 있었음)

Gazebo 토픽 (scripts/mission_gripper.py가 씀)
  /gripper/finger_l, /gripper/finger_r   (gz.msgs.Double, rad)  손가락 각도. 0 = 열림, 닫힘 = ±0.40
  /parcel_H/detach, /parcel_H/attach      (gz.msgs.Empty)        H에서 들고 가는 택배 놓기/잡기
  /parcel/detach, /parcel/attach          (gz.msgs.Empty)        P 택배 (시작할 때 붙어 있으므로 비행 전에 detach)
"""
import json
import re
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "tools"))
import auditorium_config as C  # noqa: E402

BASE = REPO / "models/x500_mydrone_base/model.sdf"
MODEL = REPO / "models/x500_mydrone/model.sdf"
SPAWN = REPO / "tools/vehicle/spawn_z.txt"
MESH = "model://x500_mydrone/meshes"

GRIPPER_KG = 0.7215
PARCEL_KG = C.PARCEL_MASS
CG_NP_MM = (-13.1, -1.1, -19.3)      # 드론 전체 (택배 없이) 무게중심, 선배 11/24
HOLD_Z = -0.21                        # 잡은 택배 가운데 높이 (base_link 기준 m)
FINGER_CLOSE = 0.17                   # 닫힘 각도 rad (오른쪽 +, 왼쪽 -). 11/26: 손가락이 택배 긴 쪽(0.25 m)을 걸쳐 잡음 → 옆면과 틈 약 2 mm (메시로 계산)

# CAD 부피 비율로 나눈 무게, 무게중심(base_link 기준 m), 관성(각 무게중심 기준, 외곽 상자 근사)
LINKS = {
    "mount":    dict(mass=0.1523, com=(-0.0044, -0.0001, -0.0244), I=(0.0003268, 0.0001379, 0.0003331)),
    "body":     dict(mass=0.2988, com=(-0.0005, 0.0068, -0.0970), I=(0.0039181, 0.0018384, 0.0038100)),
    "leg_fl":   dict(mass=0.0296, com=(0.1571, 0.0907, -0.1508), I=(9.16e-05, 0.0001286, 8.62e-05)),
    "leg_fr":   dict(mass=0.0296, com=(0.1571, -0.0907, -0.1508), I=(9.16e-05, 0.0001286, 8.62e-05)),
    "leg_bl":   dict(mass=0.0296, com=(-0.1571, 0.0907, -0.1508), I=(9.16e-05, 0.0001286, 8.62e-05)),
    "leg_br":   dict(mass=0.0296, com=(-0.1571, -0.0907, -0.1508), I=(9.16e-05, 0.0001286, 8.62e-05)),
    "finger_l": dict(mass=0.0761, com=(0.0001, 0.1329, -0.1576), I=(0.0002715, 0.0002086, 0.0001927)),
    "finger_r": dict(mass=0.0761, com=(-0.0001, -0.1324, -0.1588), I=(0.0002722, 0.0002113, 0.0001907)),
}
LEG_TIPS = [(0.213, 0.123, -0.254), (0.213, -0.123, -0.254), (-0.213, 0.123, -0.254), (-0.213, -0.123, -0.254)]
# 가운데 손가락 회전축 (가운데_손가락_회전축 부품): 앞뒤(x) 방향, 좌우 ±50 mm, 하판 아래 120 mm
FINGER_PIVOT = {"finger_l": (0.0, 0.050, -0.1203), "finger_r": (0.0, -0.050, -0.1203)}

S, E = "<!-- gripper start (tools/vehicle/set_gripper.py) -->", "<!-- gripper end -->"
ORIG = re.compile(r"<!-- gripper:orig_base_inertial (.*?) -->\n?")
OFF_S, OFF_E = "<!-- gripper-off ", " gripper-off -->"
X500_LEGS = [f"base_link_collision_{i}" for i in (1, 2, 3, 4)]
X500_LOOK = ["base_link_visual", "5010_motor_base_0", "5010_motor_base_1", "5010_motor_base_2",
             "5010_motor_base_3", "NXP_FMUK66_FRONT", "NXP_FMUK66_TOP", "RDDRONE_FMUK66_TOP"]


def f(v):
    return f"{v:.6g}"


def base_inertial(s):
    i = re.search(r"<link name=[\"']base_link[\"']>", s).end()
    a = s.index("<inertial>", i)
    b = s.index("</inertial>", a) + len("</inertial>")
    blk = s[a:b]
    pose = [float(v) for v in re.search(r"<pose>([^<]+)</pose>", blk).group(1).split()[:3]]
    mass = float(re.search(r"<mass>([^<]+)</mass>", blk).group(1))
    inert = [float(re.search(rf"<{t}>([^<]+)</{t}>", blk).group(1)) for t in ("ixx", "iyy", "izz")]
    return a, b, mass, pose, inert


def inertial_xml(mass, pose, I):
    return (f"<inertial><pose>{f(pose[0])} {f(pose[1])} {f(pose[2])} 0 0 0</pose><mass>{f(mass)}</mass>"
            f"<inertia><ixx>{f(I[0])}</ixx><ixy>0</ixy><ixz>0</ixz><iyy>{f(I[1])}</iyy><iyz>0</iyz>"
            f"<izz>{f(I[2])}</izz></inertia></inertial>")


def toggle(s, names, off):
    """base_link 의 visual/collision 을 이름으로 주석 처리(off) 하거나 되살림."""
    for n in names:
        if off:
            m = re.search(rf'\n(\s*)(<(visual|collision) name="{n}">.*?</\3>)', s, re.S)
            if m and OFF_S not in s[max(0, m.start() - 30):m.start() + 1]:
                body = m.group(2).replace("--", "- -")
                s = s[:m.start(2)] + OFF_S + body + OFF_E + s[m.end(2):]
        else:
            m = re.search(rf'{re.escape(OFF_S)}(<(visual|collision) name="{n}">.*?</\2>){re.escape(OFF_E)}', s, re.S)
            if m:
                s = s[:m.start()] + m.group(1).replace("- -", "--") + s[m.end():]
    return s


def link_xml(name, L, visuals, collisions=""):
    m, c, I = L["mass"], L["com"], L["I"]
    vis = "".join(f'<visual name="{v}"><geometry><mesh><uri>{MESH}/gripper_{v}.stl</uri></mesh></geometry>'
                  f'<material><ambient>{rgba}</ambient><diffuse>{rgba}</diffuse></material></visual>'
                  for v, rgba in visuals)
    return (f'    <link name="gripper_{name}"><pose relative_to="base_link">0 0 0 0 0 0</pose>'
            f'{inertial_xml(m, c, I)}{vis}{collisions}</link>\n')


def combine(names):
    m = sum(LINKS[n]["mass"] for n in names)
    c = [sum(LINKS[n]["mass"] * LINKS[n]["com"][k] for n in names) / m for k in range(3)]
    I = [0.0, 0.0, 0.0]
    for n in names:
        L = LINKS[n]
        d = [L["com"][k] - c[k] for k in range(3)]
        I[0] += L["I"][0] + L["mass"] * (d[1] ** 2 + d[2] ** 2)
        I[1] += L["I"][1] + L["mass"] * (d[0] ** 2 + d[2] ** 2)
        I[2] += L["I"][2] + L["mass"] * (d[0] ** 2 + d[1] ** 2)
    return dict(mass=m, com=c, I=I)


def gripper_block(keep_look):
    PETG, GREY, DARK = "0.85 0.85 0.82 1", "0.55 0.57 0.60 1", "0.20 0.22 0.25 1"
    fixed = ["mount", "body", "leg_fl", "leg_fr", "leg_bl", "leg_br"]
    tips = "".join(f'<collision name="leg_tip_{k}"><pose>{x} {y} {z + 0.012:.3f} 0 0 0</pose>'
                   f'<geometry><sphere><radius>0.012</radius></sphere></geometry>'
                   f'<surface><friction><ode><mu>0.8</mu><mu2>0.8</mu2></ode></friction></surface></collision>'
                   for k, (x, y, z) in enumerate(LEG_TIPS))
    out = [f"    {S}\n"]
    if not keep_look:
        out.append(f'    <link name="frame_cad"><pose relative_to="base_link">0 0 0 0 0 0</pose>'
                   f'{inertial_xml(1e-4, (0, 0, 0), (1e-7, 1e-7, 1e-7))}'
                   f'<visual name="frame_cad"><geometry><mesh><uri>{MESH}/frame_cad.stl</uri></mesh></geometry>'
                   f'<material><ambient>{DARK}</ambient><diffuse>{DARK}</diffuse></material></visual></link>\n'
                   f'    <joint name="frame_cad_joint" type="fixed"><parent>base_link</parent><child>frame_cad</child></joint>\n')
    out.append(link_xml("body", combine(fixed),
                        [("mount", GREY), ("body", PETG)] + [(n, PETG) for n in fixed[2:]], tips))
    out.append('    <joint name="gripper_body_joint" type="fixed"><parent>base_link</parent><child>gripper_body</child></joint>\n')
    for n in ("finger_l", "finger_r"):
        px, py, pz = FINGER_PIVOT[n]
        # 손가락에도 충돌을 넣음 (11/26): 닫히면서 옆으로 치우친 택배를 가운데로 밀어 줌 (실제 그리퍼처럼).
        # 가운데 있으면 닫힌 손가락과 택배 옆면 사이 약 7 mm → 힘이 안 걸림
        fcol = (f'<collision name="{n}_col"><geometry><mesh><uri>{MESH}/gripper_{n}.stl</uri></mesh></geometry>'
                f'<surface><friction><ode><mu>0.3</mu><mu2>0.3</mu2></ode></friction></surface></collision>')
        out.append(link_xml(n, LINKS[n], [(n, "0.86 0.45 0.20 1")], fcol))
        out.append(f'    <joint name="{n}_joint" type="revolute"><pose>{px} {py} {pz} 0 0 0</pose>'
                   f'<parent>base_link</parent><child>gripper_{n}</child>'
                   f'<axis><xyz>1 0 0</xyz><limit><lower>-0.6</lower><upper>0.6</upper><effort>3</effort>'
                   f'<velocity>3</velocity></limit><dynamics><damping>0.02</damping></dynamics></axis></joint>\n')
        sign = -1 if n == "finger_l" else 1
        out.append(f'    <plugin filename="gz-sim-joint-position-controller-system" '
                   f'name="gz::sim::systems::JointPositionController"><joint_name>{n}_joint</joint_name>'
                   f'<topic>/gripper/{n}</topic><p_gain>5</p_gain><i_gain>0.5</i_gain><d_gain>0.1</d_gain>'
                   f'<i_max>0.5</i_max><i_min>-0.5</i_min><cmd_max>2</cmd_max><cmd_min>-2</cmd_min>'
                   f'<initial_position>{sign * FINGER_CLOSE}</initial_position></plugin>\n')
    for child in ("parcel_H", "parcel"):
        out.append(f'    <plugin filename="gz-sim-detachable-joint-system" name="gz::sim::systems::DetachableJoint">'
                   f'<parent_link>base_link</parent_link><child_model>{child}</child_model><child_link>link</child_link>'
                   f'<detach_topic>/{child}/detach</detach_topic><attach_topic>/{child}/attach</attach_topic>'
                   f'<output_topic>/{child}/state</output_topic></plugin>\n')
    out.append(f"    {E}\n")
    return "".join(out)


def remove(base, model):
    m = ORIG.search(base)
    if m:
        mass, x, y, z, ixx, iyy, izz = map(float, m.group(1).split())
        a, b, *_ = base_inertial(base)
        base = base[:a] + inertial_xml(mass, (x, y, z), (ixx, iyy, izz)) + base[b:]
        base = ORIG.sub("", base)
    base = toggle(base, X500_LEGS + X500_LOOK, off=False)
    model = re.sub(rf"\s*{re.escape(S)}.*?{re.escape(E)}\n?", "\n", model, flags=re.S)
    return base, model


def main():
    args = sys.argv[1:]
    base, model = BASE.read_text(), MODEL.read_text()
    if args[:1] == ["remove"]:
        base, model = remove(base, model)
        BASE.write_text(base)
        MODEL.write_text(model)
        SPAWN.unlink(missing_ok=True)
        print("그리퍼 뗌: 몸통 무게 원래대로, x500 다리·모양 되살림, spawn_z.txt 삭제")
        return
    keep_look = "--keep-x500-look" in args
    cg_np = CG_NP_MM
    if "--cg-np" in args:
        i = args.index("--cg-np")
        cg_np = tuple(float(v) for v in args[i + 1:i + 4])
    hold_z = float(args[args.index("--hold-z") + 1]) if "--hold-z" in args else HOLD_Z

    # 몸통 원래 값 (그리퍼 붙이기 전). 이미 붙어 있고 set_mass_cg.py를 다시 안 돌렸으면 기록해 둔 값을 씀
    a, b, mass, pose, inert = base_inertial(base)
    m = ORIG.search(base)
    if m:
        o = list(map(float, m.group(1).split()))
        reduced = abs(mass - (o[0] - PARCEL_KG - sum(L["mass"] for L in LINKS.values()))) < 1e-3
        if reduced:
            mass, pose, inert = o[0], o[1:4], o[4:7]
        base, model = remove(base, model)
        a, b, *_ = base_inertial(base)
    orig = (mass, *pose, *inert)

    grip_total = sum(L["mass"] for L in LINKS.values())
    m_base = mass - PARCEL_KG - grip_total
    if m_base <= 0.5:
        sys.exit(f"몸통 무게가 이상함 ({mass:.3f} kg). set_mass_cg.py 2600 ... 를 먼저 실행")
    M = m_base + grip_total
    c_np = [v / 1000 for v in cg_np]
    c_base = [(M * c_np[k] - sum(L["mass"] * L["com"][k] for L in LINKS.values())) / m_base for k in range(3)]
    I_base = [v * m_base / mass for v in inert]

    base = base[:a] + inertial_xml(m_base, c_base, I_base) + base[b:]
    base = base[:a] + (f"<!-- gripper:orig_base_inertial {' '.join(f(v) for v in orig)} -->\n      ") + base[a:]
    base = toggle(base, X500_LEGS + ([] if keep_look else X500_LOOK), off=True)
    model = model.replace("  </model>\n</sdf>", gripper_block(keep_look) + "  </model>\n</sdf>")
    BASE.write_text(base)
    MODEL.write_text(model)
    # x500_mydrone_base 는 모델 원점이 바닥(x500 다리 끝)이고 base_link가 그 위 0.24 m (<pose>0 0 .24). 생성 높이는 모델 원점 기준
    base_off = float(re.search(r"<model name='x500_mydrone_base'>\s*<pose>[^ ]+ [^ ]+ ([^ ]+)", base).group(1))
    spawn = round(-hold_z + C.PARCEL_SIZE[2] / 2 - base_off, 3)
    SPAWN.write_text(f"{spawn}  # 드론 생성 높이 m (택배를 든 채 택배 위에 얹힌 높이). set_gripper.py가 씀\n")

    print("그리퍼 붙임 (models/x500_mydrone/model.sdf, x500_mydrone_base/model.sdf)")
    print(f"  몸통   {mass:.4f} kg -> {m_base:.4f} kg  (택배 {PARCEL_KG} kg, 그리퍼 {grip_total:.4f} kg 뺌)")
    print(f"  몸통 무게중심 ({pose[0]*1000:+.1f}, {pose[1]*1000:+.1f}, {pose[2]*1000:+.1f}) -> "
          f"({c_base[0]*1000:+.1f}, {c_base[1]*1000:+.1f}, {c_base[2]*1000:+.1f}) mm")
    print(f"  드론 전체 (택배 없이) {M + 0.064:.3f} kg, 무게중심 {cg_np} mm  /  택배 들면 {M + 0.064 + PARCEL_KG:.3f} kg")
    print(f"  손가락 관절 2개 (처음 닫힘 ±{FINGER_CLOSE} rad), 택배 붙이기 2개 (parcel_H 들고 시작, parcel은 비행 전에 떼기)")
    print(f"  착륙: 그리퍼 다리 끝 (하판 아래 254 mm), x500 다리 충돌 끔{'' if keep_look else ', 기체 모양 = 실제 CAD'}")
    print(f"  드론 생성 높이 {spawn} m (모델 원점 기준. base_link는 그 위 {base_off} m → 바닥에서 {spawn + base_off:.2f} m)")
    print("다음: python3 tools/world/make_auditorium_world.py && python3 tools/world/add_apriltag.py  (월드에 택배 2개)")
    print("      bash tools/run_mydrone_flow_test.sh  →  python3 scripts/mission_gripper.py 결과폴더")


if __name__ == "__main__":
    main()
