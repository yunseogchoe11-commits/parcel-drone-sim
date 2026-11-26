"""비행 로그에서 '드론이 믿는 위치 - 실제 위치'를 시간별로 보여 줌 (어느 구간에서 오차가 커지는지 찾기).
사용: python3 tools/analysis/est_vs_true.py [로그.ulg] [시작초] [간격초]   (로그를 안 주면 가장 최근 로그)
"""
import glob, os, sys
import numpy as np
from pyulog import ULog
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
import auditorium_config as C

f = sys.argv[1] if len(sys.argv) > 1 else max(glob.glob(os.path.expanduser(
    "~/PX4-Autopilot/build/px4_sitl_default/rootfs/log/*/*.ulg")), key=os.path.getmtime)
t_start = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0
step = float(sys.argv[3]) if len(sys.argv) > 3 else 3.0
s = getattr(C, "S", C.H)
off_n, off_e = s[1] - C.H[1], s[0] - C.H[0]          # 추정 위치 원점(S) → H 기준으로
u = ULog(f, ["vehicle_local_position", "vehicle_local_position_groundtruth", "distance_sensor"])
d = {x.name: x.data for x in u.data_list}
e, g, r = d["vehicle_local_position"], d["vehicle_local_position_groundtruth"], d["distance_sensor"]
t0 = g["timestamp"][0]
T = lambda x: (x["timestamp"] - t0) / 1e6
print(f)
print("    t  실제고도 거리계   추정오차(북, 동 cm)   실제위치-H(북, 동 m)")
for t in np.arange(t_start, T(g)[-1], step):
    i, j, k = np.argmin(abs(T(e) - t)), np.argmin(abs(T(g) - t)), np.argmin(abs(T(r) - t))
    dn = (e["x"][i] + off_n - g["x"][j]) * 100
    de = (e["y"][i] + off_e - g["y"][j]) * 100
    print(f"  {t:5.0f}  {-g['z'][j]:5.2f}  {r['current_distance'][k]:5.2f}      {dn:+5.0f} {de:+5.0f}           {g['x'][j]:+6.2f} {g['y'][j]:+6.2f}")
