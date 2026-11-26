"""기체 아래쪽 카메라를 Gazebo 모델에 단다. (4번: 카메라 + AprilTag 정밀 접근 준비)

사용
  python3 tools/vehicle/set_camera.py                          # 기본값으로 달기 (아래 표)
  python3 tools/vehicle/set_camera.py --x 0.12 --pitch 90      # 위치·각도 바꾸기 (선배가 정해 주면)
  python3 tools/vehicle/set_camera.py --hfov 62.2 --width 640 --height 480 --rate 15
  python3 tools/vehicle/set_camera.py remove                   # 카메라 떼기

기본값 (11/24 선배 확정 센서 위치 문서 기준)
  모듈   IMX219 120° 렌즈 (DB-NVA-009): 대각 120° = 가로 약 90°, 세로 약 75°, 4:3
         시뮬은 가볍게 640 x 480, 초당 15장
  위치   그리퍼 하우징 아래 가운데 (어깨 0°): base_link(모터 4개 가운데·하판 아랫면) 기준 (0, 0, -0.144) m
         어깨 축 (0, 0, -0.060)에서 아래로 0.083 m. 어깨가 ±30° 기울면 같이 기울어짐 (그리퍼를 넣으면 그리퍼 링크에 붙임)
  각도   pitch 90° = 바로 아래를 봄, 화면 위쪽 = 기체 앞
  가까운 면 0.003 m: 그리퍼로 쥔 택배 윗면이 카메라 6 mm 아래라서, 0.02 m면 택배가 잘려 안 보이고 바닥이 보였음 (11/25)
  (11/23까지는 IMX219 v2 렌즈 62.2°, 앞 0.12 m, 아래 0.03 m 가정)

결과
  models/x500_mydrone/model.sdf 에 down_camera_link + camera 센서 추가
  영상 토픽: /down_camera/image  (확인: gz topic -l | grep down_camera)
  설정은 tools/vehicle/camera_config.txt 에 저장 (인식 코드가 읽음)
  주의: set_mass_cg.py reset 을 하면 모델 파일이 원래대로 돌아가서 카메라도 빠짐 → 이 스크립트를 다시 실행
"""
import math
import re
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
SDF = REPO / "models/x500_mydrone/model.sdf"
CONFIG = REPO / "tools/vehicle/camera_config.txt"
START, END = "<!-- down_camera start -->", "<!-- down_camera end -->"
DEFAULT = dict(x=0.0, y=0.0, z=-0.144, pitch=90.0, yaw=0.0, hfov=90.0, width=640, height=480, rate=15)


def strip(s):
    return re.sub(r"\s*" + re.escape(START) + r".*?" + re.escape(END), "", s, flags=re.S)


def main():
    args = sys.argv[1:]
    s = SDF.read_text(encoding="utf-8")
    if args == ["remove"]:
        SDF.write_text(strip(s), encoding="utf-8")
        CONFIG.unlink(missing_ok=True)
        print("카메라 뗌")
        return
    c = dict(DEFAULT)
    i = 0
    while i < len(args):
        key = args[i].lstrip("-")
        if key not in c or i + 1 >= len(args):
            sys.exit(__doc__)
        c[key] = type(DEFAULT[key])(float(args[i + 1]))
        i += 2

    hfov = math.radians(c["hfov"])
    block = f"""
    {START}
    <!-- [위치] 아래쪽 카메라 (IMX219 120°, 그리퍼 하우징 아래): 앞 {c['x']} m, 좌우 {c['y']} m, 위아래 {c['z']} m, pitch {c['pitch']}° -->
    <joint name="down_camera_joint" type="fixed">
      <parent>base_link</parent>
      <child>down_camera_link</child>
    </joint>
    <link name="down_camera_link">
      <pose relative_to="base_link">{c['x']} {c['y']} {c['z']} 0 {math.radians(c['pitch']):.4f} {math.radians(c['yaw']):.4f}</pose>
      <inertial>
        <mass>0.003</mass>
        <inertia><ixx>1e-6</ixx><iyy>1e-6</iyy><izz>1e-6</izz><ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia>
      </inertial>
      <visual name="down_camera_visual">
        <geometry><box><size>0.01 0.025 0.024</size></box></geometry>
        <material><ambient>0.1 0.1 0.1 1</ambient><diffuse>0.1 0.1 0.1 1</diffuse></material>
      </visual>
      <sensor name="down_camera" type="camera">
        <gz_frame_id>down_camera_link</gz_frame_id>
        <topic>/down_camera/image</topic>
        <update_rate>{c['rate']}</update_rate>
        <always_on>1</always_on>
        <visualize>false</visualize>
        <camera>
          <horizontal_fov>{hfov:.4f}</horizontal_fov>
          <image><width>{c['width']}</width><height>{c['height']}</height><format>R8G8B8</format></image>
          <clip><near>0.003</near><far>30</far></clip>
          <noise><type>gaussian</type><mean>0</mean><stddev>0.005</stddev></noise>
        </camera>
      </sensor>
    </link>
    {END}"""
    s = strip(s)
    k = s.rindex("</model>")
    s = s[:k].rstrip() + "\n" + block + "\n  " + s[k:]
    SDF.write_text(s, encoding="utf-8")

    fx = c["width"] / 2 / math.tan(hfov / 2)
    CONFIG.write_text("".join(f"{k} {v}\n" for k, v in c.items()) + f"fx {fx:.2f}\n", encoding="utf-8")
    vfov = 2 * math.degrees(math.atan(c["height"] / 2 / fx))
    print(f"카메라 달기: 앞 {c['x']} m, 좌우 {c['y']} m, 위아래 {c['z']} m, pitch {c['pitch']}°")
    print(f"  화각 가로 {c['hfov']}° / 세로 {vfov:.1f}°, {c['width']} x {c['height']}, 초당 {c['rate']:.0f}장, 초점거리 {fx:.0f} px")
    for h in (1.0, 1.5):
        print(f"  고도 {h} m에서 바닥이 보이는 범위: {2 * h * math.tan(hfov / 2):.2f} x {2 * h * c['height'] / 2 / fx:.2f} m")
    print("  영상 토픽: /down_camera/image  (시뮬을 다시 켜야 적용)")


if __name__ == "__main__":
    main()
