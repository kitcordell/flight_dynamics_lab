import matplotlib.pyplot as plt


AERO_COLORS = {
    "background": "#08111f",
    "axes": "#0f1b2d",
    "grid": "#28425f",
    "text": "#d8e6f3",
    "muted": "#8fa6ba",
    "cyan": "#4dd8ff",
    "amber": "#ffbf4d",
    "green": "#78e08f",
    "magenta": "#ff6ec7",
    "red": "#ff5c5c",
    "blue": "#7aa7ff",
}


def set_aerospace_theme():
    plt.rcParams.update({
        "figure.facecolor": AERO_COLORS["background"],
        "axes.facecolor": AERO_COLORS["axes"],
        "axes.edgecolor": AERO_COLORS["muted"],
        "axes.labelcolor": AERO_COLORS["text"],
        "axes.titlecolor": AERO_COLORS["text"],
        "xtick.color": AERO_COLORS["muted"],
        "ytick.color": AERO_COLORS["muted"],
        "text.color": AERO_COLORS["text"],
        "grid.color": AERO_COLORS["grid"],
        "grid.linestyle": "--",
        "grid.linewidth": 0.6,
        "legend.facecolor": AERO_COLORS["axes"],
        "legend.edgecolor": AERO_COLORS["grid"],
        "legend.framealpha": 0.9,
        "lines.linewidth": 1.6,
        "font.size": 9,
        "axes.titlesize": 11,
        "axes.titleweight": "bold",
    })


def style_axes(ax):
    ax.grid(True, alpha=0.55)
    ax.minorticks_on()
    ax.grid(True, which="minor", alpha=0.18)
    for spine in ax.spines.values():
        spine.set_color(AERO_COLORS["muted"])
        spine.set_linewidth(0.8)
