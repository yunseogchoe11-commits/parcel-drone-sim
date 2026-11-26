"""미션 비행 분석: 계획 경로 vs 드론이 믿는 경로(추정) vs 실제 경로(Gazebo 정답).

사용
  python3 tools/analysis/analyze_mission_log.py 결과폴더이름 [ulog 파일 경로]
  예) python3 tools/analysis/analyze_mission_log.py 261031_mission_hdph
  ulog 경로를 안 주면 가장 최근 로그를 쓴다.

결과 (results/결과폴더이름/)
  1_route_top.png    위에서 본 경로: 계획(회색) / 추정(주황 점선) / 실제(파랑) + H·D·P
  2_error_time.png   시간에 따른 수평 위치 오차, 고도
  3_yaw.png          기수(yaw): 추정 vs 실제, 그 차이 (기수 오차가 있으면 경로가 옆으로 휨)
  summary.txt        각 지점 도착 오차, 비행 시간, 최대 오차

용어
  - 추정 위치: FC가 광류·거리계·IMU로 계산한 위치 (제어에 쓰는 값)
  - 실제 위치: Gazebo 정답 (시뮬레이션에만 있음)
  - 도착 오차: 드론이 "도착했다"고 믿는 순간(추정 위치가 목표에 가장 가까운 때) 실제 위치와 목표 사이 거리
좌표: 그림은 x = 동, y = 북 (Gazebo 화면과 같은 방향). 원점 = H.
"""
import math
import sys
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from pyulog import ULog

REPO = Path(__file__).resolve().parents[2]   # 저장소 맨 위 (tools/analysis/ 에서 두 단계 위)
sys.path.insert(0, str(REPO / "tools"))
import auditorium_config as C  # noqa: E402

LOG_ROOT = Path.home() / "PX4-Autopilot/build/px4_sitl_default/rootfs/log"

# 차분한 색
C_PLAN = "#9a9a9a"
C_TRUTH = "#3d5a80"
C_EST = "#c08457"
C_ERR = "#6b4e71"
C_ALT = "#5c7a5c"
C_GRID = "#e0e0e0"
PT_COLOR = {"H": "#4d6a8a", "D": "#9a5b52", "P": "#5f7f62"}


def latest_ulog():
    logs = sorted(LOG_ROOT.glob("*/*.ulg"), key=lambda p: p.stat().st_mtime)
    if not logs:
        sys.exit(f"로그가 없습니다: {LOG_ROOT}")
    return logs[-1]


def topic(ulog, name):
    for d in ulog.data_list:
        if d.name == name and d.multi_id == 0:
            data = dict(d.data)
            data["timestamp"] = data["timestamp"].astype(np.float64)   # 빼기 넘침 방지
            return data
    return None


def yaw_of(q):
    """쿼터니언 (w, x, y, z) → 기수 (도, 0 = 북, 시계방향 +)."""
    w, x, y, z = q
    return np.degrees(np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))) % 360


def wrap180(a):
    return (a + 180) % 360 - 180


def to_en(hall_xy):
    """강당 좌표 → (동, 북), 원점 = H (= 월드 원점 = Gazebo 정답 위치의 원점)."""
    return hall_xy[0] - C.H[0], hall_xy[1] - C.H[1]


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    out = REPO / "results" / sys.argv[1]
    path = Path(sys.argv[2]) if len(sys.argv) > 2 else latest_ulog()
    out.mkdir(parents=True, exist_ok=True)

    ulog = ULog(str(path), ["vehicle_local_position", "vehicle_local_position_groundtruth",
                            "vehicle_attitude", "vehicle_attitude_groundtruth"])
    est = topic(ulog, "vehicle_local_position")
    tru = topic(ulog, "vehicle_local_position_groundtruth")
    if est is None or tru is None:
        sys.exit("필요한 기록(vehicle_local_position / _groundtruth)이 로그에 없습니다.")

    t0 = tru["timestamp"][0]
    tt = (tru["timestamp"] - t0) / 1e6
    te = (est["timestamp"] - t0) / 1e6
    ex = np.interp(tt, te, est["x"]); ey = np.interp(tt, te, est["y"]); ez = np.interp(tt, te, est["z"])
    s = getattr(C, "S", C.H)          # 추정 위치의 원점 = 드론 출발 자리 S → H 기준으로 옮김 (11/26: S = H 뒤 1 m)
    ex, ey = ex + (s[1] - C.H[1]), ey + (s[0] - C.H[0])
    tx, ty, tz = tru["x"].astype(float), tru["y"].astype(float), tru["z"].astype(float)

    # 이륙 시점과 기준점(이륙 직전 바닥)
    lift = np.where(-(tz - np.median(tz[tt < 3])) > 0.2)[0]
    if len(lift) == 0:
        sys.exit("이 로그에는 이륙 기록이 없습니다.")
    t_lift = tt[lift[0]]
    ground = (tt > t_lift - 3.0) & (tt < t_lift - 0.5)
    if ground.sum() < 5:
        ground = tt < t_lift
    ex, ey, ez = ex - np.median(ex[ground]), ey - np.median(ey[ground]), ez - np.median(ez[ground])
    tx, ty, tz = tx - np.median(tx[ground]), ty - np.median(ty[ground]), tz - np.median(tz[ground])
    alt, alt_est = -tz, -ez

    # 비행 구간: 이륙 ~ 착륙 (실제 고도 0.2 m 이상)
    air = np.where(alt > 0.2)[0]
    t_land = tt[air[-1]]
    fly = (tt >= t_lift) & (tt <= t_land)

    # NED(북 x, 동 y) → 그림 (동, 북)
    e_est, n_est = ey, ex
    e_tru, n_tru = ty, tx
    err = np.hypot(e_est - e_tru, n_est - n_tru)

    pts = {"H": to_en(C.H), "D": to_en(C.D), "P": to_en(C.P)}

    # 각 지점 도착 오차
    lines_arr = []
    order = [("D", None), ("P", None), ("H", "end")]
    t_from = t_lift + 5.0
    for name, mode in order:
        pe, pn = pts[name]
        if mode == "end":
            k = np.where(fly)[0][-1]                       # 착륙 직전
        else:
            m = fly & (tt > t_from)
            d_est = np.hypot(e_est - pe, n_est - pn)
            d_est[~m] = np.inf
            k = int(np.argmin(d_est))
            t_from = tt[k] + 3.0
        d_real = math.hypot(e_tru[k] - pe, n_tru[k] - pn)
        lines_arr.append(f"  {name}: 실제 위치가 목표에서 {d_real * 100:.0f} cm "
                         f"(동 {(e_tru[k] - pe) * 100:+.0f}, 북 {(n_tru[k] - pn) * 100:+.0f} cm), t = {tt[k]:.0f} s")

    # ── 그림 1: 경로 ──
    fig, ax = plt.subplots(figsize=(7.5, 7))
    route = [pts["H"], pts["D"], pts["P"], pts["H"]]
    ax.plot([p[0] for p in route], [p[1] for p in route], color=C_PLAN, lw=2.5, alpha=0.6, label="planned route")
    ax.plot(e_tru[fly], n_tru[fly], color=C_TRUTH, lw=1.8, label="actual (groundtruth)")
    ax.plot(e_est[fly], n_est[fly], color=C_EST, lw=1.3, ls="--", label="estimated (EKF)")
    for name, (pe, pn) in pts.items():
        ax.add_patch(plt.Rectangle((pe - 0.6, pn - 0.6), 1.2, 1.2, fill=False, ec=PT_COLOR[name], lw=1.5))
        ax.text(pe + 0.8, pn + 0.4, name, color=PT_COLOR[name], fontsize=13, weight="bold")
    ax.set_xlabel("East [m]"); ax.set_ylabel("North [m]")
    ax.set_aspect("equal", "datalim"); ax.grid(color=C_GRID); ax.legend(frameon=False, loc="lower left")
    ax.set_title("Mission H-D-P-H top view")
    fig.tight_layout(); fig.savefig(out / "1_route_top.png", dpi=150); plt.close(fig)

    # ── 그림 2: 오차와 고도 ──
    fig, (a1, a2) = plt.subplots(2, 1, figsize=(9, 5.5), sharex=True)
    a1.plot(tt[fly], err[fly] * 100, color=C_ERR, lw=1.2)
    a1.set_ylabel("horizontal error [cm]"); a1.grid(color=C_GRID)
    a2.plot(tt[fly], alt[fly], color=C_ALT, lw=1.4, label="actual")
    a2.plot(tt[fly], alt_est[fly], color=C_EST, lw=1.0, ls="--", label="estimated")
    a2.set_ylabel("altitude [m]"); a2.set_xlabel("time [s]"); a2.grid(color=C_GRID); a2.legend(frameon=False)
    fig.suptitle("Estimated vs actual position during mission")
    fig.tight_layout(); fig.savefig(out / "2_error_time.png", dpi=150); plt.close(fig)

    # ── 그림 3: 기수 ──
    att, att_t = topic(ulog, "vehicle_attitude"), topic(ulog, "vehicle_attitude_groundtruth")
    yaw_line = "기수 비교: 기록 없음"
    if att is not None and att_t is not None:
        ta = (att["timestamp"] - t0) / 1e6
        tg = (att_t["timestamp"] - t0) / 1e6
        ya = yaw_of([att[f"q[{i}]"] for i in range(4)])
        yg = yaw_of([att_t[f"q[{i}]"] for i in range(4)])
        yg_on_a = np.degrees(np.interp(ta, tg, np.unwrap(np.radians(yg)))) % 360
        dy = wrap180(ya - yg_on_a)
        m = (ta >= t_lift) & (ta <= t_land)
        fig, (b1, b2) = plt.subplots(2, 1, figsize=(9, 5.5), sharex=True)
        b1.plot(tg[(tg >= t_lift) & (tg <= t_land)], yg[(tg >= t_lift) & (tg <= t_land)], color=C_TRUTH, lw=1.6, label="actual")
        b1.plot(ta[m], ya[m], color=C_EST, lw=1.0, ls="--", label="estimated")
        b1.set_ylabel("yaw [deg] (0 = North)"); b1.grid(color=C_GRID); b1.legend(frameon=False)
        b2.plot(ta[m], dy[m], color=C_ERR, lw=1.2)
        b2.set_ylabel("yaw error [deg]"); b2.set_xlabel("time [s]"); b2.grid(color=C_GRID)
        fig.tight_layout(); fig.savefig(out / "3_yaw.png", dpi=150); plt.close(fig)
        yaw_line = (f"기수 오차 (추정 - 실제): 평균 {dy[m].mean():+.1f}°, 최대 {np.abs(dy[m]).max():.1f}° "
                    f"(10 m 이동 시 옆으로 약 {10 * math.sin(math.radians(abs(dy[m].mean()))) * 100:.0f} cm에 해당)")

    lines = [
        f"로그: {path}",
        f"비행 시간: {t_land - t_lift:.0f} s (이륙 {t_lift:.0f} s ~ 착륙 {t_land:.0f} s)",
        f"고도: 실제 최고 {alt[fly].max():.2f} m",
        "",
        "[각 지점 도착 오차 = 드론이 도착했다고 믿을 때 실제 위치와 목표의 거리]",
        *lines_arr,
        "",
        "[수평 위치 추정 오차 (추정 - 실제)]",
        f"  평균 {err[fly].mean() * 100:.0f} cm, 최대 {err[fly].max() * 100:.0f} cm, 착륙 직전 {err[fly][-1] * 100:.0f} cm",
        "",
        yaw_line,
    ]
    (out / "summary.txt").write_text("\n".join(lines), encoding="utf-8")
    print("\n".join(lines))
    print("\n저장:", out)


if __name__ == "__main__":
    main()
