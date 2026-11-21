"""weight_sweep.sh 결과를 표(sweep.txt)와 그래프(sweep.png)로 모은다.

사용: python3 tools/analysis/sweep_table.py results/결과폴더
판정 기준 (팀 기준 + 여유 시험)
  호버 스로틀 ≤ 60 %  : 설계 목표 (발표 10/14: 호버 추력 ≤ 60 %, T/W ≥ 1.67)
  호버 스로틀 ≤ 70 %  : 한계 (넘으면 튜닝보다 경량화)
  여유 시험 중 모터 포화(≥ 0.95) 시간 ≤ 1 %, 기울기·고도 넘침이 무게에 따라 급격히 커지지 않을 것
"""
import re
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def num(pat, s):
    m = re.search(pat, s)
    return float(m.group(1)) if m else float("nan")


def main():
    root = Path(sys.argv[1])
    rows = []
    for d in sorted(root.glob("*g"), key=lambda p: float(p.name[:-1])):
        s = (d / "summary.txt").read_text(encoding="utf-8") if (d / "summary.txt").exists() else ""
        rows.append(dict(
            w=float(d.name[:-1]),
            hover=num(r"PX4 추정 호버 추력 ([\d.]+) %", s),
            calc=num(r"최대추력 = [\d.]+ / [\d.]+ kgf = ([\d.]+) %", s),
            mmax=num(r"모터 출력 최대 ([\d.]+)", s),
            sat=num(r"포화\) 시간 ([\d.]+) %", s),
            tilt=num(r"기울기 최대 ([\d.]+)°", s),
            over=num(r"넘친 양 (\d+) cm\)\n  북쪽", s),
            up=num(r"상승 속도 최대 ([\d.]+)", s),
            ok=bool(s)))
    if not rows:
        sys.exit("결과 없음")

    def verdict(r):
        if not r["ok"] or r["hover"] != r["hover"]:
            return "비행 실패/기록 없음"
        if r["sat"] > 1 or r["hover"] > 70:
            return "불가 (포화 또는 70 % 초과)"
        if r["hover"] > 60:
            return "가능하나 여유 적음 (60~70 %)"
        return "정상 (≤ 60 %)"

    L = ["무게 g | 호버 스로틀 % (계산) | 모터 최대 | 포화 시간 % | 기울기 최대 ° | 급상승 고도 넘침 cm | 상승 최대 m/s | 판정",
         "---|---|---|---|---|---|---|---"]
    for r in rows:
        L.append(f"{r['w']:.0f} | {r['hover']:.1f} ({r['calc']:.1f}) | {r['mmax']:.3f} | {r['sat']:.1f} | {r['tilt']:.1f} | "
                 f"{r['over']:.0f} | {r['up']:.2f} | {verdict(r)}")
    good = [r["w"] for r in rows if verdict(r).startswith("정상")]
    okw = [r["w"] for r in rows if not verdict(r).startswith(("불가", "비행"))]
    L += ["", f"설계 목표(≤ 60 %) 안의 최대 무게: {max(good):.0f} g" if good else "설계 목표 안에 드는 무게 없음",
          f"비행 가능(포화 없음, ≤ 70 %) 최대 무게: {max(okw):.0f} g" if okw else "비행 가능한 무게 없음"]
    text = "\n".join(L)
    print(text)
    (root / "sweep.txt").write_text(text + "\n", encoding="utf-8")

    w = [r["w"] for r in rows]
    fig, ax = plt.subplots(1, 2, figsize=(11, 4))
    ax[0].plot(w, [r["hover"] for r in rows], "o-", label="hover thrust (sim)")
    ax[0].plot(w, [r["calc"] for r in rows], "--", color="gray", label="weight / max thrust")
    ax[0].axhline(60, color="green", lw=1, ls=":", label="design 60 %")
    ax[0].axhline(70, color="red", lw=1, ls=":", label="limit 70 %")
    ax[0].set_xlabel("total mass [g]"); ax[0].set_ylabel("hover thrust [%]"); ax[0].legend(fontsize=8); ax[0].grid(alpha=0.3)
    ax[1].plot(w, [r["mmax"] for r in rows], "o-", label="max motor output (margin test)")
    ax[1].axhline(0.95, color="red", lw=1, ls=":", label="saturation 0.95")
    ax[1].set_xlabel("total mass [g]"); ax[1].set_ylabel("motor output"); ax[1].legend(fontsize=8); ax[1].grid(alpha=0.3)
    fig.suptitle(root.name); fig.tight_layout(); fig.savefig(root / "sweep.png", dpi=150)
    print(f"\n저장: {root}/sweep.txt, sweep.png")


if __name__ == "__main__":
    main()
