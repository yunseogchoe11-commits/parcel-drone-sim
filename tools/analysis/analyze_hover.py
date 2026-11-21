"""호버링 시험 분석: 호버 추력(스로틀), 모터 출력, 자세·위치 흔들림.

사용
  python3 tools/analysis/analyze_hover.py 결과폴더이름 [ulog 파일 경로]
  예) python3 tools/analysis/analyze_hover.py 261120_hover_2400g_cg-17.7
  ulog 경로를 안 주면 가장 최근 로그를 쓴다.

결과 (results/결과폴더이름/)
  hover.png     고도 / 모터 출력 / 롤·피치 (호버 구간 음영)
  summary.txt   기체 설정, 로그에 기록된 PX4 파라미터, 호버 추력, 흔들림

판정 기준
  - 호버 추력 = PX4가 추정한 '떠 있는 데 필요한 추력 비율' (hover_thrust_estimate, 0~1).
    THR_MDL_FAC 0.75에서는 Gazebo 모터의 실제 추력 비율과 거의 같다.
    70 % 이상이면 튜닝보다 경량화가 필요하다는 근거로 쓴다 (공학설계 발표 계획).
  - 모터 출력이 0.95 이상 찍히면 포화 (자세 제어 여유 없음).
"""
import math
import re
import sys
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from pyulog import ULog

REPO = Path(__file__).resolve().parents[2]
LOG_ROOT = Path.home() / "PX4-Autopilot/build/px4_sitl_default/rootfs/log"
SDF = REPO / "models/x500_mydrone_base/model.sdf"
PARAMS = ["SYS_AUTOSTART", "THR_MDL_FAC", "SIM_GZ_EC_MIN1", "SIM_GZ_EC_MAX1", "MPC_THR_HOVER", "MPC_USE_HTE",
          "CA_ROTOR0_PX", "CA_ROTOR1_PX", "CA_ROTOR2_PX", "CA_ROTOR3_PX",
          "EKF2_OF_POS_X", "EKF2_RNG_POS_X", "EKF2_IMU_POS_X",
          "MC_ROLLRATE_P", "MC_PITCHRATE_P", "MC_ROLL_P", "MC_PITCH_P", "BAT1_N_CELLS", "BAT1_CAPACITY"]
MOTOR_SDF = REPO / "models/x500_mydrone/model.sdf"


def max_thrust_kgf():
    """모터 4개 최대 추력 합 = 4 x motorConstant x maxRotVelocity^2 (x500_mydrone/model.sdf)."""
    s = MOTOR_SDF.read_text()
    k = float(re.search(r"<motorConstant>([^<]+)</motorConstant>", s).group(1))
    w = float(re.search(r"<maxRotVelocity>([^<]+)</maxRotVelocity>", s).group(1))
    return 4 * k * w * w / 9.81, k, w


def latest_ulog():
    logs = sorted(LOG_ROOT.glob("*/*.ulg"), key=lambda p: p.stat().st_mtime)
    if not logs:
        sys.exit(f"로그가 없습니다: {LOG_ROOT}")
    return logs[-1]


def topic(u, name):
    for d in u.data_list:
        if d.name == name:
            return d.data
    return None


def rp_deg(q0, q1, q2, q3):
    roll = np.degrees(np.arctan2(2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1 ** 2 + q2 ** 2)))
    pitch = np.degrees(np.arcsin(np.clip(2 * (q0 * q2 - q3 * q1), -1, 1)))
    return roll, pitch


def model_setting():
    s = SDF.read_text()
    i = re.search(r"<link name=[\"']base_link[\"']>", s).end()
    blk = s[s.index("<inertial>", i):s.index("</inertial>", i)]
    pose = [float(v) for v in re.search(r"<pose>([^<]+)</pose>", blk).group(1).split()[:3]]
    mass = float(re.search(r"<mass>([^<]+)</mass>", blk).group(1)) + 0.064
    return mass, pose


def margin_stats(u, t0, tp, alt, pos, mot):
    """margin_test.py 전체 비행(공중 구간)의 여유: 모터 포화, 최대 기울기, 상승 속도, 고도·위치 넘침."""
    air = alt > 0.3
    ta, tb = tp[air][0], tp[air][-1]
    L = ["", "[여유 시험 전체 비행] (급상승 2.5 m, 옆 이동 2 m @1.5 m/s 포함)"]
    if mot is not None:
        tm = (mot["timestamp"] - t0) / 1e6
        m = (tm > ta) & (tm < tb)
        c = np.vstack([mot[f"control[{i}]"][m] for i in range(4)])
        sat = (c.max(axis=0) >= 0.95).mean() * 100
        L.append(f"  모터 출력 최대 {c.max():.3f}, 최소 {c.min():.3f}, 0.95 이상(포화) 시간 {sat:.1f} %"
                 + ("  ← 포화 있음: 이 무게는 급기동 여유 부족" if sat > 1 else ""))
    att = topic(u, "vehicle_attitude_groundtruth") or topic(u, "vehicle_attitude")
    tt = (att["timestamp"] - t0) / 1e6
    m = (tt > ta) & (tt < tb)
    r, pch = rp_deg(att["q[0]"][m], att["q[1]"][m], att["q[2]"][m], att["q[3]"][m])
    L.append(f"  기울기 최대 {np.hypot(r, pch).max():.1f}°")
    vz = -pos["vz"] if "vz" in pos else np.gradient(alt, tp)
    L.append(f"  상승 속도 최대 {vz[air].max():.2f} m/s, 하강 최대 {(-vz[air]).max():.2f} m/s")
    L.append(f"  최고 고도 {alt.max():.2f} m (목표 2.5 m, 넘친 양 {max(0.0, alt.max() - 2.5) * 100:.0f} cm)")
    n = pos["x"] - pos["x"][0]
    L.append(f"  북쪽 최대 이동 {n[air].max():.2f} m (목표 2.0 m, 넘친 양 {max(0.0, n[air].max() - 2.0) * 100:.0f} cm)")
    return L


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    args = [x for x in sys.argv[1:] if x != "--margin"]
    margin = "--margin" in sys.argv
    tag = args[0]
    path = Path(args[1]) if len(args) > 1 else latest_ulog()
    out = REPO / "results" / tag
    out.mkdir(parents=True, exist_ok=True)
    u = ULog(str(path))

    pos = topic(u, "vehicle_local_position_groundtruth")
    src = "Gazebo 실제값"
    if pos is None:
        pos, src = topic(u, "vehicle_local_position"), "FC 추정값"
    t0 = pos["timestamp"][0]
    tp = (pos["timestamp"] - t0) / 1e6
    alt = -pos["z"] + pos["z"][0]
    high = tp[alt > 1.35]
    if len(high) < 2:
        sys.exit("호버 구간을 못 찾음 (1.35 m 이상 비행 기록 없음)")
    a, b = high[0] + 5.0, high[-1] - 2.0          # 올라간 직후 5초, 내려가기 전 2초 제외
    if margin:                                     # margin_test.py: 처음 1.5 m 호버 15초 중 가운데만
        a, b = high[0] + 4.0, high[0] + 14.0
    if b - a < 5:
        sys.exit(f"호버 구간이 너무 짧음 ({b - a:.1f} s). hover_test.py 시간을 늘려 주세요")
    w = (tp > a) & (tp < b)

    def win(d):
        t = (d["timestamp"] - t0) / 1e6
        return t, (t > a) & (t < b)

    mass, pose = model_setting()
    tmax, _, _ = max_thrust_kgf()
    p = u.initial_parameters
    L = [f"로그: {path}", f"분석 시각 기준 모델: 총 {mass * 1000:.0f} g, 무게중심 x {pose[0] * 1000:+.1f} mm, z {pose[2] * 1000:+.1f} mm, 최대 추력 {tmax:.2f} kgf",
         "  (모델을 비행 뒤에 바꿨다면 이 줄은 비행 때와 다를 수 있음. 아래 파라미터는 로그에 기록된 비행 당시 값)",
         f"호버 구간: {a:.1f} ~ {b:.1f} s ({b - a:.1f} s), 위치 기준: {src}", "",
         "[비행 당시 PX4 파라미터]"]
    for n in PARAMS:
        if n in p:
            L.append(f"  {n:15s} {p[n]:g}")

    # 호버 추력
    L += ["", "[호버 추력]"]
    hte = topic(u, "hover_thrust_estimate")
    hover = None
    if hte is not None:
        t, m = win(hte)
        ok = m & (hte["valid"] > 0) if "valid" in hte else m
        if ok.any():
            hover = float(hte["hover_thrust"][ok].mean())
            L.append(f"  PX4 추정 호버 추력 {hover * 100:.1f} %  (시작값 MPC_THR_HOVER {p.get('MPC_THR_HOVER', float('nan')):.2f})")
    ts = topic(u, "vehicle_thrust_setpoint")
    if ts is not None:
        t, m = win(ts)
        L.append(f"  추력 명령 평균     {-ts['xyz[2]'][m].mean() * 100:.1f} %")
    tmax, mk, mw = max_thrust_kgf()
    L.append(f"  참고: 무게/최대추력 = {mass:.3f} / {tmax:.2f} kgf = {mass / tmax * 100:.1f} % "
             f"(분석 시각 기준 모터: k {mk:.4e}, 최대 {mw:.0f} rad/s)")
    if hover is not None:
        L.append("  판정: " + ("70 % 이상 → 튜닝보다 경량화 필요 근거" if hover >= 0.70 else
                              "70 % 미만" + (" (65 % 이상, 여유 적음)" if hover >= 0.65 else "")))

    # 모터
    mot = topic(u, "actuator_motors")
    mv = None
    if mot is not None:
        t, m = win(mot)
        mv = [float(mot[f"control[{i}]"][m].mean()) for i in range(4)]
        mx = max(float(mot[f"control[{i}]"][m].max()) for i in range(4))
        L += ["", "[모터 출력 평균] (0 앞오른, 1 뒤왼, 2 앞왼, 3 뒤오른)",
              "  " + ", ".join(f"{v:.3f}" for v in mv),
              f"  앞(0,2) {np.mean([mv[0], mv[2]]):.3f} / 뒤(1,3) {np.mean([mv[1], mv[3]]):.3f}, 호버 중 최대 {mx:.3f}"
              + ("  ← 0.95 이상: 포화" if mx >= 0.95 else "")]
    tq = topic(u, "vehicle_torque_setpoint")
    if tq is not None:
        t, m = win(tq)
        L.append(f"  토크 명령 평균 롤 {tq['xyz[0]'][m].mean():+.4f}, 피치 {tq['xyz[1]'][m].mean():+.4f} "
                 "(0에 가까울수록 무게중심 보정이 맞음)")

    # 자세 흔들림
    att = topic(u, "vehicle_attitude_groundtruth")
    if att is None:
        att = topic(u, "vehicle_attitude")
    ta, ma = win(att)
    roll, pitch = rp_deg(att["q[0]"], att["q[1]"], att["q[2]"], att["q[3]"])
    L += ["", "[자세 흔들림] (호버 구간)",
          f"  롤   평균 {roll[ma].mean():+.2f}°, 표준편차 {roll[ma].std():.2f}°, 최대 {np.abs(roll[ma]).max():.2f}°",
          f"  피치 평균 {pitch[ma].mean():+.2f}°, 표준편차 {pitch[ma].std():.2f}°, 최대 {np.abs(pitch[ma]).max():.2f}°"]

    # 위치 흔들림
    n, e = pos["x"][w], pos["y"][w]
    r = np.hypot(n - n.mean(), e - e.mean())
    L += ["", "[위치 흔들림] (호버 구간, " + src + ")",
          f"  수평: 평균에서 최대 {r.max() * 100:.1f} cm, RMS {math.sqrt((r ** 2).mean()) * 100:.1f} cm",
          f"  고도: 평균 {alt[w].mean():.2f} m, 표준편차 {alt[w].std() * 100:.1f} cm"]

    if margin:
        L += margin_stats(u, t0, tp, alt, pos, mot)

    text = "\n".join(L)
    print(text)
    (out / "summary.txt").write_text(text + "\n", encoding="utf-8")

    fig, axs = plt.subplots(3, 1, figsize=(9, 9), sharex=True)
    axs[0].plot(tp, alt); axs[0].set_ylabel("altitude [m]")
    if mot is not None:
        tm = (mot["timestamp"] - t0) / 1e6
        for i, lb in enumerate(["0 front-R", "1 rear-L", "2 front-L", "3 rear-R"]):
            axs[1].plot(tm, mot[f"control[{i}]"], label=lb, lw=0.8)
        axs[1].legend(fontsize=7, ncol=4); axs[1].set_ylabel("motor output")
    axs[2].plot(ta, roll, label="roll", lw=0.8); axs[2].plot(ta, pitch, label="pitch", lw=0.8)
    axs[2].set_ylabel("angle [deg]"); axs[2].legend(fontsize=7); axs[2].set_xlabel("time [s]")
    for ax in axs:
        ax.axvspan(a, b, alpha=0.12); ax.grid(alpha=0.3)
    title = f"hover {mass * 1000:.0f} g, CG x {pose[0] * 1000:+.1f} mm"
    if hover is not None:
        title += f", hover thrust {hover * 100:.0f} %"
    fig.suptitle(title); fig.tight_layout(); fig.savefig(out / "hover.png", dpi=150); plt.close(fig)
    print(f"\n저장: {out}/summary.txt, hover.png")


if __name__ == "__main__":
    main()
