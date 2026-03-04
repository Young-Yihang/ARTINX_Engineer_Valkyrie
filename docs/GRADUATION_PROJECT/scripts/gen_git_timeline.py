#!/usr/bin/env python3
"""
Git 开发时间线可视化 - PPT用高清图
完全复刻参考图比例: 顶部条=背景色带=同一x坐标, P4+P5压缩, 彩虹phase标注
"""

import subprocess
import re
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from collections import defaultdict
from datetime import datetime, timedelta

# ── 配置 ──────────────────────────────────────────────────────────────
OUTPUT = "docs/GRADUATION_PROJECT/generated_images/git_timeline.png"
BG = "#F8F9FA"
GRID_CLR = "#E1E4E8"
TEXT_CLR = "#24292E"
ACCENT = "#0366D6"
CURVE_CLR = "#0D7377"

# 12 phases: (name, start, end, accent_color)
# 配色序列(参考图): 浅蓝→深蓝→紫→淡绿→深绿→橙→浅红→深红, 过渡到12色
PHASES = [
    ("P1 URDF\n+MoveIt2",       "2025-10-27", "2025-10-28", "#0288D1"),  # 浅蓝
    ("P2 KDL\n+PD Ctrl",        "2025-10-29", "2025-11-01", "#1565C0"),  # 蓝
    ("P3 Kalman\n+MuJoCo",      "2025-11-02", "2025-11-07", "#3949AB"),  # 靛蓝
    ("P4 PD\nTuning",           "2025-11-08", "2025-11-30", "#5E35B1"),  # 紫
    ("P5 RealSense\n& Internship","2025-12-01","2025-12-28", "#00897B"),  # 青绿
    ("P6 NUC\n+Serial",         "2025-12-29", "2026-01-09", "#43A047"),  # 绿
    ("P7 HotReload\n+Scripts",  "2026-01-10", "2026-01-17", "#7CB342"),  # 黄绿
    ("P8 CascadePID\n+TrajMgr", "2026-01-18", "2026-01-28", "#C0CA33"),  # 黄
    ("P9 HW\nInterface",        "2026-02-01", "2026-02-05", "#FB8C00"),  # 橙
    ("P10 Cartesian\n+Mission", "2026-02-06", "2026-02-21", "#E53935"),  # 浅红
    ("P11 ImGui\n+ImPlot",      "2026-02-22", "2026-02-24", "#D32F2F"),  # 红
    ("P12 PickPlace\n+Sim2Real","2026-02-25", "2026-03-04", "#B71C1C"),  # 深红
]

# 顶部条色 — 严格彩虹渐变: 浅蓝→蓝→靛→紫→青绿→绿→黄绿→黄→橙→浅红→红→深红
PHASE_TOP = [
    "#4FC3F7",  # P1  浅蓝
    "#2196F3",  # P2  蓝
    "#5C6BC0",  # P3  靛蓝
    "#7E57C2",  # P4  紫
    "#26A69A",  # P5  青绿
    "#66BB6A",  # P6  绿
    "#9CCC65",  # P7  黄绿
    "#D4E157",  # P8  黄
    "#FFA726",  # P9  橙
    "#EF5350",  # P10 浅红
    "#E53935",  # P11 红
    "#C62828",  # P12 深红
]

# 背景色 — 对应淡色
PHASE_BG = [
    "#B3E5FC",  # P1  淡蓝
    "#BBDEFB",  # P2  淡蓝
    "#C5CAE9",  # P3  淡靛
    "#D1C4E9",  # P4  淡紫
    "#B2DFDB",  # P5  淡青绿
    "#C8E6C9",  # P6  淡绿
    "#DCEDC8",  # P7  淡黄绿
    "#F0F4C3",  # P8  淡黄
    "#FFE0B2",  # P9  淡橙
    "#FFCDD2",  # P10 淡红
    "#FFCDD2",  # P11 淡红
    "#FFCDD2",  # P12 淡红
]


def get_git_data():
    out = subprocess.check_output(
        ["git", "log", "--format=%ad", "--date=short"]).decode().strip()
    commits = defaultdict(int)
    for line in out.splitlines():
        commits[line] += 1

    out2 = subprocess.check_output(
        ["git", "log", "--format=%ad", "--date=short", "--stat"]).decode()
    lines = defaultdict(int)
    cur = None
    for line in out2.splitlines():
        line = line.strip()
        if re.match(r'^\d{4}-\d{2}-\d{2}$', line):
            cur = line
        elif cur and 'changed' in line:
            m = re.findall(r'(\d+) insertion', line)
            if m:
                lines[cur] += int(m[0])
    return commits, lines


# 参考图中每个phase的宽度比例 (从像素测量)
PHASE_WIDTHS = [6.0, 5.3, 7.5, 6.0, 9.0, 9.8, 9.8, 10.5, 7.5, 9.0, 7.5, 12.0]
_total_w = sum(PHASE_WIDTHS)
# 归一化到总宽100单位
PHASE_WIDTHS_NORM = [w / _total_w * 100 for w in PHASE_WIDTHS]

# 预计算每个phase的x起止
_PHASE_X = []
_cx = 0
for w in PHASE_WIDTHS_NORM:
    _PHASE_X.append((_cx, _cx + w))
    _cx += w


def _phase_index_for_date(d):
    """返回日期所属的phase index"""
    base = datetime(2025, 10, 27)
    if isinstance(d, str):
        d = datetime.strptime(d, "%Y-%m-%d")
    for i, (name, ps, pe, _) in enumerate(PHASES):
        ps_dt = datetime.strptime(ps, "%Y-%m-%d")
        pe_dt = datetime.strptime(pe, "%Y-%m-%d")
        if ps_dt <= d <= pe_dt:
            return i, ps_dt, pe_dt
    # 在phase间隙 — 找最近的phase
    for i, (name, ps, pe, _) in enumerate(PHASES):
        pe_dt = datetime.strptime(pe, "%Y-%m-%d")
        if i < len(PHASES) - 1:
            next_ps = datetime.strptime(PHASES[i+1][1], "%Y-%m-%d")
            if pe_dt < d < next_ps:
                # 归入下一个phase的起始
                return i+1, next_ps, datetime.strptime(PHASES[i+1][2], "%Y-%m-%d")
    return len(PHASES)-1, datetime.strptime(PHASES[-1][1], "%Y-%m-%d"), datetime.strptime(PHASES[-1][2], "%Y-%m-%d")


def date_to_x(d):
    """按参考图比例映射: 每个phase按固定宽度, 日期在phase内线性插值"""
    if isinstance(d, str):
        d = datetime.strptime(d, "%Y-%m-%d")
    i, ps, pe = _phase_index_for_date(d)
    x0, x1 = _PHASE_X[i]
    span = (pe - ps).days
    if span == 0:
        frac = 0.5
    else:
        frac = max(0, min(1, (d - ps).days / span))
    return x0 + frac * (x1 - x0)


def main():
    commits_per_day, lines_per_day = get_git_data()

    start = datetime(2025, 10, 27)
    end = datetime(2026, 3, 4)
    all_dates = []
    d = start
    while d <= end:
        all_dates.append(d)
        d += timedelta(days=1)

    xs = [date_to_x(d) for d in all_dates]
    date_strs = [d.strftime("%Y-%m-%d") for d in all_dates]
    daily_commits = [commits_per_day.get(ds, 0) for ds in date_strs]

    cum_lines = []
    total = 0
    for ds in date_strs:
        total += lines_per_day.get(ds, 0)
        cum_lines.append(total)

    total_commits = sum(commits_per_day.values())
    total_lines = cum_lines[-1]
    cmax = max(cum_lines)
    max_c = max(daily_commits)

    # ── 画布 (扁横幅, PPT友好 ~3.7:1) ──────────────────────────────────
    fig, ax = plt.subplots(figsize=(22, 6), dpi=240, facecolor=BG)
    ax.set_facecolor(BG)

    y_max = cmax * 1.30
    ax.set_ylim(0, y_max)
    ax.set_xlim(min(xs) - 1, max(xs) + 4)

    ax.grid(axis="y", color=GRID_CLR, linewidth=0.5, zorder=0)
    for sp in ax.spines.values():
        sp.set_visible(False)

    # ── 顶部条 + 背景色带 (同一x坐标, 完全对齐) ──────────────────────
    phase_y_top = y_max * 0.98
    phase_y_bot = y_max * 0.90
    phase_h = phase_y_top - phase_y_bot

    for i, (name, ps, pe, _) in enumerate(PHASES):
        x0 = date_to_x(ps)
        # 无缝: 延伸到下一个phase的start
        if i < len(PHASES) - 1:
            x1 = date_to_x(PHASES[i + 1][1])
        else:
            x1 = max(xs) + 4

        bg = PHASE_BG[i]

        # 背景色带 (全高度)
        ax.axvspan(x0, x1, color=bg, alpha=0.18, zorder=0)

        # 顶部条 (加深色, 留微小间隙避免干涉)
        gap = 0.15
        top_clr = PHASE_TOP[i]
        rect = mpatches.FancyBboxPatch(
            (x0 + gap, phase_y_bot), x1 - x0 - gap * 2, phase_h,
            boxstyle="round,pad=0.08",
            facecolor=top_clr, edgecolor="none", alpha=0.9, zorder=5)
        ax.add_patch(rect)

        # 文字 (窄条自动缩小字体, 扁图字体整体放大)
        w = x1 - x0
        fs = 8.0 if w < 4 else (9.5 if w < 7 else 10.5)
        ax.text((x0 + x1) / 2, (phase_y_top + phase_y_bot) / 2,
                name, ha="center", va="center",
                fontsize=fs, fontweight="bold", color="#333333",
                zorder=6, linespacing=0.85)

    # ── 提交柱图 ────────────────────────────────────────────────────────
    bar_scale = cmax / max(max_c, 1) * 0.72

    for xi, c in zip(xs, daily_commits):
        if c > 0:
            t = c / max_c
            r = int(0x26 + (0x0E - 0x26) * t)
            g = int(0xA6 + (0x44 - 0xA6) * t)
            b = int(0x41 + (0x29 - 0x41) * t)
            ax.bar(xi, c * bar_scale, width=0.7, bottom=0,
                   color=f"#{r:02x}{g:02x}{b:02x}", alpha=0.7, zorder=2)

    # ── 累积曲线 ────────────────────────────────────────────────────────
    cxs = [xi for xi, cl in zip(xs, cum_lines) if cl > 0]
    cys = [cl for cl in cum_lines if cl > 0]

    ax.plot(cxs, cys, color=CURVE_CLR, linewidth=3.0, zorder=4,
            solid_capstyle="round", linestyle="-")

    # 散点仅在有commit的日子
    dxs = [xi for xi, c in zip(xs, daily_commits) if c > 0]
    dys = [cl for cl, c in zip(cum_lines, daily_commits) if c > 0]
    ax.scatter(dxs, dys, color="#FF9800", s=35, zorder=5,
               edgecolors="white", linewidths=0.6)

    # ── Phase 标注 (曲线上拉开, 带箭头, 交错避让) ─────────────────────────
    # 手动微调每个phase标注的y偏移方向和幅度, 避免互相遮挡
    # (direction: +1=上, -1=下, offset_mult: 偏移倍数)
    LABEL_OFFSETS = [
        (+1, 0.14),  # P1  上
        (-1, 0.14),  # P2  下
        (+1, 0.16),  # P3  上
        (-1, 0.12),  # P4  下
        (+1, 0.14),  # P5  上
        (-1, 0.14),  # P6  下
        (+1, 0.20),  # P7  上 (拉远, 避免和P8重叠)
        (-1, 0.18),  # P8  下 (拉远, 避免和P7重叠)
        (+1, 0.14),  # P9  上
        (-1, 0.14),  # P10 下
        (+1, 0.14),  # P11 上
        (-1, 0.12),  # P12 下
    ]

    for i, (name, ps, pe, clr) in enumerate(PHASES):
        if pe in date_strs:
            idx = date_strs.index(pe)
            px, py = xs[idx], cum_lines[idx]
        else:
            px = date_to_x(pe)
            py = cum_lines[min(range(len(xs)), key=lambda j: abs(xs[j] - px))]

        direction, mult = LABEL_OFFSETS[i]
        oy = py + direction * cmax * mult
        va = "bottom" if direction > 0 else "top"
        # 限制在可见区域内
        oy = max(cmax * 0.06, min(oy, y_max * 0.82))

        ax.annotate(
            name.replace("\n", " "), xy=(px, py), xytext=(px, oy),
            fontsize=14, fontweight="bold", color=clr, ha="center", va=va,
            bbox=dict(boxstyle="round,pad=0.25", facecolor=PHASE_BG[i],
                      edgecolor=clr, alpha=0.92, linewidth=1.2),
            arrowprops=dict(arrowstyle="-|>", color=clr, lw=1.0,
                            mutation_scale=10, shrinkA=0, shrinkB=2),
            zorder=7)

    # ── X轴 ─────────────────────────────────────────────────────────────
    ticks, labels = [], []
    for ms in ["2025-11-01", "2025-12-01", "2026-01-01", "2026-02-01", "2026-03-01"]:
        dt = datetime.strptime(ms, "%Y-%m-%d")
        if start <= dt <= end:
            ticks.append(date_to_x(dt))
            labels.append(dt.strftime("%m/%d"))
    ax.set_xticks(ticks)
    ax.set_xticklabels(labels, fontsize=14, color="#586069")
    ax.tick_params(axis="x", length=0, pad=6)

    ax.set_ylabel("Cumulative Lines Added", fontsize=15, color="#586069")
    ax.tick_params(axis="y", colors="#586069", labelsize=13, length=0)

    # ── 标题 ─────────────────────────────────────────────────────────────
    fig.text(0.5, 0.97,
             f"ARV_V1 ROS2 Development  ·  {total_commits} commits  ·  "
             f"+{total_lines:,} lines  ·  12 phases  ·  5 months",
             ha="center", va="top", fontsize=18, fontweight="bold", color=ACCENT)

    # ── 图例 ─────────────────────────────────────────────────────────────
    ax.text(max(xs) + 3.5, -cmax * 0.03,
            "Bars = daily commits  |  Curve = cumulative code  |  "
            "Dec compressed (RealSense in separate repo)",
            ha="right", va="top", fontsize=9, color="#8B949E", style="italic")

    plt.tight_layout(rect=[0.02, 0.03, 0.99, 0.94])
    plt.savefig(OUTPUT, dpi=240, bbox_inches="tight", facecolor=BG)
    print(f"Saved: {OUTPUT}")


if __name__ == "__main__":
    main()
