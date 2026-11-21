"""여유 시험(margin_test.py)의 빠른 옆 이동(북쪽 2 m)에서 넘친 원인 가리기.

사용: python3 tools/analysis/check_overshoot.py results/261121_2600g/2600g
      (폴더 안 flight.ulg, 없으면 summary.txt 첫 줄의 로그 경로)

비교하는 세 가지 (모두 이륙 지점 기준 북쪽 m)
  목표   : 드론에 보낸 목표 위치 (trajectory_setpoint, 추정 좌표)
  추정   : 드론이 믿는 위치 (vehicle_local_position)
  실제   : Gazebo 실제 위치 (vehicle_local_position_groundtruth)

판정
  추정도 목표를 크게 넘음     → 제어(위치·속도 제어 게인, 가속 제한) 문제 → 튜닝 대상
  추정은 목표 근처, 실제만 넘음 → 위치 추정(광류) 오차 → 센서·EKF 설정 대상
"""
import sys
from pathlib import Path

import numpy as np
from pyulog import ULog


def topic(u, name):
    for d in u.data_list:
        if d.name == name:
            return d.data
    return None


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    folder = Path(sys.argv[1])
    log = folder / "flight.ulg"
    if not log.exists():
        first = (folder / "summary.txt").read_text(encoding="utf-8").splitlines()[0]
        log = Path(first.split(":", 1)[1].strip())
    u = ULog(str(log), ["trajectory_setpoint", "vehicle_local_position", "vehicle_local_position_groundtruth"])
    sp, est, gt = (topic(u, n) for n in ("trajectory_setpoint", "vehicle_local_position", "vehicle_local_position_groundtruth"))
    t0 = est["timestamp"][0]
    te = (est["timestamp"] - t0) / 1e6
    ts = (sp["timestamp"] - t0) / 1e6
    tg = (gt["timestamp"] - t0) / 1e6
    n_sp = sp["position[0]"]
    n_est = est["x"] - est["x"][0]
    n_gt = gt["x"] - gt["x"][0]
    n0 = est["x"][0]

    # 옆 이동 구간: 목표가 북쪽으로 1 m 넘게 나가 있는 동안 (+ 끝나고 3초)
    far = ts[(n_sp - n0) > 1.0]
    if len(far) == 0:
        sys.exit("북쪽 2 m 이동 구간을 못 찾음 (margin_test.py 로그가 맞는지 확인)")
    a, b = far[0], far[-1]
    target = float(np.median(n_sp[(ts >= a) & (ts <= b)] - n0))
    me = (te >= a) & (te <= b)
    mg = (tg >= a) & (tg <= b)
    est_max, gt_max = n_est[me].max(), n_gt[mg].max()
    t_end = b - 0.5
    est_end = float(np.interp(t_end, te, n_est))
    gt_end = float(np.interp(t_end, tg, n_gt))

    print(f"로그: {log}")
    print(f"옆 이동 구간: {a:.1f} ~ {b:.1f} s, 목표 북쪽 {target:.2f} m")
    print(f"  최대 위치   추정 {est_max:.2f} m (넘침 {max(0, est_max - target) * 100:.0f} cm) / "
          f"실제 {gt_max:.2f} m (넘침 {max(0, gt_max - target) * 100:.0f} cm)")
    print(f"  멈춘 뒤 위치 추정 {est_end:.2f} m / 실제 {gt_end:.2f} m (추정 오차 {(est_end - gt_end) * 100:+.0f} cm)")
    over_est = est_max - target
    over_gt = gt_max - target
    if over_est > 0.6 * over_gt and over_est > 0.10:
        print("판정: 드론이 믿는 위치도 목표를 넘음 → 제어 반응 문제 (튜닝 대상: MPC_XY_P, MPC_XY_VEL_P_ACC, MPC_ACC_HOR 등)")
    elif over_gt > 0.10:
        print("판정: 드론은 목표에 맞게 멈췄다고 믿는데 실제로 더 감 → 위치 추정(광류) 오차 (튜닝보다 센서·EKF 쪽)")
    else:
        print("판정: 넘침 10 cm 이하 → 문제 없음")


if __name__ == "__main__":
    main()
