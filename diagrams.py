from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import matplotlib.patches as mpatches


plt.rcParams.update(
    {
        "font.family": "DejaVu Sans",
        "figure.facecolor": "white",
        "axes.facecolor": "white",
    }
)


OUTPUT_DIR = Path("docs/mechanical/assets/torque-sizing")
OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

C_LINK_BG = "#D8D8D8"
C_LINK_FG = "#1A1A1A"
C_BODY = "#EAE8FB"
C_BODY_EDGE = "#5A3EB8"
C_JOINT = "#FFFFFF"
C_FOOT = "#555555"
C_GROUND = "#1A1A1A"
C_HATCH = "#777777"
C_COM = "#FFFFFF"


def setup_axis(ax, xlim, ylim):
    ax.set_xlim(*xlim)
    ax.set_ylim(*ylim)
    ax.set_aspect("equal")
    ax.axis("off")


def link(ax, p0, p1, lw_bg=14, lw_fg=3):
    ax.plot(
        [p0[0], p1[0]],
        [p0[1], p1[1]],
        color=C_LINK_BG,
        lw=lw_bg,
        solid_capstyle="round",
        zorder=2,
    )
    ax.plot(
        [p0[0], p1[0]],
        [p0[1], p1[1]],
        color=C_LINK_FG,
        lw=lw_fg,
        solid_capstyle="round",
        zorder=3,
    )


def joint(ax, p, r=0.055):
    ax.add_patch(
        plt.Circle(p, r, facecolor=C_JOINT, edgecolor=C_LINK_FG, lw=2, zorder=5)
    )


def foot(ax, p, rx=0.13, ry=0.055):
    ax.add_patch(
        mpatches.Ellipse(
            p, 2 * rx, 2 * ry, facecolor=C_FOOT, edgecolor=C_LINK_FG, lw=1.5, zorder=5
        )
    )


def com_marker(ax, p, r=0.045):
    ax.add_patch(
        plt.Circle(
            p,
            r,
            facecolor=C_COM,
            edgecolor=C_HATCH,
            lw=1.5,
            linestyle="--",
            zorder=6,
        )
    )


def ground(ax, x0, x1, y, hatch_len=0.11, n=14):
    ax.plot([x0, x1], [y, y], color=C_GROUND, lw=2, zorder=1)
    for i in range(n):
        x = x0 + (x1 - x0) * i / (n - 1)
        ax.plot([x, x - hatch_len], [y, y - hatch_len], color=C_HATCH, lw=1, zorder=1)


def midpoint(p0, p1):
    return ((p0[0] + p1[0]) / 2, (p0[1] + p1[1]) / 2)


def save(fig, name):
    pdf_path = OUTPUT_DIR / f"{name}.pdf"
    png_path = OUTPUT_DIR / f"{name}.png"
    fig.savefig(pdf_path, bbox_inches="tight", dpi=220)
    fig.savefig(png_path, bbox_inches="tight", dpi=220)
    plt.close(fig)
    print(f"saved {pdf_path}")
    print(f"saved {png_path}")


def draw_overview():
    fig, ax = plt.subplots(figsize=(5.2, 7.2))
    setup_axis(ax, (-0.2, 3.2), (-0.25, 5.1))

    hip = (1.5, 4.0)
    knee = (0.8, 2.2)
    foot_pt = (1.5, 0.5)
    com_thigh = midpoint(hip, knee)
    com_calf = midpoint(knee, foot_pt)

    body = mpatches.FancyBboxPatch(
        (0.6, 4.2),
        1.8,
        0.7,
        boxstyle="round,pad=0.05",
        facecolor=C_BODY,
        edgecolor=C_BODY_EDGE,
        lw=1.5,
        zorder=2,
    )
    ax.add_patch(body)

    link(ax, hip, knee)
    link(ax, knee, foot_pt)
    joint(ax, hip)
    joint(ax, knee)
    foot(ax, foot_pt)
    com_marker(ax, com_thigh)
    com_marker(ax, com_calf)
    ground(ax, 0.2, 2.8, 0.35)

    save(fig, "fbd_overview")


def draw_thigh():
    fig, ax = plt.subplots(figsize=(4.6, 5.8))
    setup_axis(ax, (-0.45, 1.75), (0.9, 3.7))

    hip = (1.0, 3.2)
    knee = (0.3, 1.4)
    com_thigh = midpoint(hip, knee)

    link(ax, hip, knee)
    joint(ax, hip)
    joint(ax, knee)
    com_marker(ax, com_thigh)

    save(fig, "fbd_thigh")


def draw_calf():
    fig, ax = plt.subplots(figsize=(4.8, 5.8))
    setup_axis(ax, (0.15, 1.9), (0.45, 3.35))

    knee = (0.6, 3.0)
    foot_pt = (1.4, 0.8)
    com_calf = midpoint(knee, foot_pt)

    link(ax, knee, foot_pt)
    joint(ax, knee)
    foot(ax, foot_pt)
    com_marker(ax, com_calf)
    ground(ax, 0.5, 1.85, 0.65)

    save(fig, "fbd_calf")


if __name__ == "__main__":
    draw_overview()
    draw_thigh()
    draw_calf()
