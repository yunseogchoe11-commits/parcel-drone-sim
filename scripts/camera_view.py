"""아래쪽 카메라 화면을 따로 창으로 띄우기 (태그 인식 표시 포함). 녹화·발표 영상용.

Gazebo의 Image Display 플러그인이 안 뜰 때 대신 씀. 시뮬이 켜져 있는 동안 아무 때나 실행.
  초록 테두리 = 인식한 태그, 빨간 점 = 태그 중심, 노란 + = 영상 가운데(= 그리퍼 중심)
  위쪽 글자 = 영상 가운데 ~ 태그 수평 거리, 태그까지 거리

사용 (CMD 터미널을 하나 더 열어서):
  python3 scripts/camera_view.py          # 끄기: 창에서 q 또는 Esc, 또는 터미널에서 Ctrl+C
  python3 scripts/camera_view.py --scale 1.5   # 창 크기 1.5배
"""
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from tag_approach_test import Camera, TagEye  # noqa: E402


def main():
    import cv2
    scale = float(sys.argv[sys.argv.index("--scale") + 1]) if "--scale" in sys.argv else 1.0
    cam, eye = Camera(), TagEye()
    win = "down camera (q: quit)"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(win, int(eye.w * scale), int(eye.h * scale))
    print("카메라 영상 기다리는 중... (시뮬이 켜져 있어야 함)")
    last_stamp, t_wait = 0.0, time.time()
    while True:
        rgb, stamp = cam.latest()
        if rgb is None:
            if time.time() - t_wait > 10:
                sys.exit("10초 동안 영상이 안 옴 → 시뮬이 켜져 있는지, 카메라가 달렸는지(set_camera.py) 확인")
            time.sleep(0.1)
            continue
        if stamp != last_stamp:
            last_stamp = stamp
            d = eye.detect(rgb)
            img = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            cv2.drawMarker(img, (eye.w // 2, eye.h // 2), (0, 255, 255), cv2.MARKER_CROSS, 40, 2)
            if d is not None:
                cv2.polylines(img, [d.corners.astype(int)], True, (0, 255, 0), 3)
                cv2.circle(img, tuple(d.center.astype(int)), 5, (0, 0, 255), -1)
                t = d.pose_t.ravel()
                text = f"TAG  offset {math.hypot(t[0], t[1]) * 100:.1f} cm   dist {t[2]:.2f} m"
                color = (0, 200, 0)
            else:
                text, color = "no tag", (0, 0, 255)
            cv2.rectangle(img, (0, 0), (eye.w, 34), (30, 30, 30), -1)
            cv2.putText(img, text, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
            cv2.imshow(win, img)
        if cv2.waitKey(15) & 0xFF in (ord("q"), 27):
            break
    cv2.destroyAllWindows()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
