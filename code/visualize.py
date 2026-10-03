"""Matplotlib animation of a recorded simulation."""

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.lines import Line2D
from matplotlib.patches import Circle, Rectangle

SURFACE = "#fcfcfb"
INK = "#0b0b0b"
MUTED = "#898781"
SHELF = "#52514e"
WALL = "#d9d8d3"
HOME = "#cde2fb"
STATION = "#c9ecd9"
PATH = "#2a78d6"
REPLANNED = "#eb6834"
OBSTACLE = "#d03b3b"
STATE_COLOR = {"active": "#2a78d6", "frozen": "#eda100",
               "stalled": "#eb6834", "broken": "#d03b3b"}


def animate(sim, save=None, show=True, substeps=3, fps=12):
    """Replay `sim` (run with record=True). Robots are coloured by state and
    labelled by id; a plan that was just repaired is drawn in orange."""
    grid, frames = sim.grid, sim.frames
    w, h = grid.width, grid.height
    fig = plt.figure(figsize=(13, 6.6), facecolor=SURFACE)
    ax = fig.add_axes([0.01, 0.07, 0.68, 0.86])
    side = fig.add_axes([0.70, 0.07, 0.29, 0.86])
    side.axis("off")
    ax.set_xlim(-0.5, w - 0.5)
    ax.set_ylim(h - 0.5, -0.5)
    ax.set_aspect("equal")
    ax.axis("off")
    ax.set_facecolor(SURFACE)

    def square(cell, color, **kw):
        return Rectangle((cell[0] - 0.5, cell[1] - 0.5), 1, 1, color=color,
                         lw=0, **kw)

    for c in grid.walls:
        ax.add_patch(square(c, WALL))
    for c in grid.shelves:
        ax.add_patch(Rectangle((c[0] - 0.46, c[1] - 0.46), 0.92, 0.92,
                               color=SHELF, lw=0))
    for c in grid.home_owner:
        ax.add_patch(square(c, HOME))
    for c in grid.stations:
        ax.add_patch(square(c, STATION))

    n = len(sim.agents)
    lines = [ax.plot([], [], lw=1.2, alpha=0.45, color=PATH, zorder=2)[0]
             for _ in range(n)]
    bodies = [Circle((0, 0), 0.38, zorder=4, ec=SURFACE, lw=1) for _ in range(n)]
    labels = [ax.text(0, 0, str(i), ha="center", va="center", fontsize=6.5,
                      color="white", zorder=5, fontweight="bold")
              for i in range(n)]
    for b in bodies:
        ax.add_patch(b)
    pickups = ax.scatter([], [], marker="^", s=34, facecolors="none",
                         edgecolors=INK, linewidths=1.0, zorder=3)
    urgent = ax.scatter([], [], marker="*", s=150, color=OBSTACLE,
                        edgecolors=SURFACE, linewidths=0.6, zorder=3)
    obstacles = []

    title = ax.set_title("", loc="left", fontsize=12, color=INK)
    info = side.text(0, 1, "", va="top", ha="left", fontsize=8.5, color=INK,
                     family="monospace", wrap=True)
    handles = [
        Line2D([], [], marker="o", ls="", color=STATE_COLOR["active"], label="robot"),
        Line2D([], [], marker="o", ls="", color=STATE_COLOR["frozen"], label="temporarily down"),
        Line2D([], [], marker="o", ls="", color=STATE_COLOR["stalled"], label="holding (safe stop)"),
        Line2D([], [], marker="o", ls="", color=STATE_COLOR["broken"], label="broken down"),
        Line2D([], [], marker="s", ls="", color=OBSTACLE, label="dynamic obstacle"),
        Line2D([], [], marker="^", ls="", mfc="none", color=INK, label="pickup"),
        Line2D([], [], marker="*", ls="", color=OBSTACLE, ms=10, label="emergency pickup"),
        Line2D([], [], color=PATH, label="planned path"),
        Line2D([], [], color=REPLANNED, lw=2, label="plan just repaired"),
        Line2D([], [], marker="s", ls="", color=HOME, label="parking"),
        Line2D([], [], marker="s", ls="", color=STATION, label="delivery station"),
    ]
    fig.legend(handles=handles, loc="lower left", ncol=6, frameon=False,
               fontsize=8, bbox_to_anchor=(0.01, 0.0))

    def draw(k):
        i, sub = divmod(k, substeps)
        f = frames[i]
        nxt = frames[min(i + 1, len(frames) - 1)]
        a = sub / substeps
        t = f["t"]
        hot = set(f["hot"])
        for j in range(n):
            (x0, y0), (x1, y1) = f["pos"][j], nxt["pos"][j]
            x, y = x0 + (x1 - x0) * a, y0 + (y1 - y0) * a
            bodies[j].center = (x, y)
            bodies[j].set_facecolor(STATE_COLOR[f["status"][j]])
            labels[j].set_position((x, y))
            rest = f["paths"][j][t + 1:]
            lines[j].set_data([x] + [c[0] for c in rest],
                              [y] + [c[1] for c in rest])
            lines[j].set_color(REPLANNED if j in hot else PATH)
            lines[j].set_alpha(0.9 if j in hot else 0.4)
            lines[j].set_linewidth(2.0 if j in hot else 1.1)
        while obstacles:
            obstacles.pop().remove()
        for c in f["blocked"]:
            if c in f["pos"]:
                continue  # the broken robot itself is drawn in red
            obstacles.append(ax.add_patch(square(c, OBSTACLE, zorder=3)))
            obstacles.append(ax.text(c[0], c[1], "×", ha="center", va="center",
                                     color="white", fontsize=9, zorder=3.5))
        pend = [k for k in f["tasks"] if k[2] == "pending"]
        pickups.set_offsets([k[0] for k in pend if not k[4]] or [(-9, -9)])
        urgent.set_offsets([k[0] for k in pend if k[4]] or [(-9, -9)])
        done = sum(1 for k in f["tasks"] if k[2] == "delivered")
        lost = sum(1 for k in f["tasks"] if k[2] == "lost")
        title.set_text(f"t = {t}    delivered {done}/{len(f['tasks'])}"
                       + (f"    lost {lost}" if lost else ""))
        recent = [f"[{lt:3d}] {msg}" for lt, msg in sim.log[:f["log"]]][-14:]
        info.set_text("Event log\n\n" + "\n".join(_wrap(m, 46) for m in recent))
        return []

    total = (len(frames) - 1) * substeps + 1
    anim = FuncAnimation(fig, draw, frames=total, interval=1000 / fps,
                         blit=False, repeat=False)
    if save:
        anim.save(save, writer=PillowWriter(fps=fps), dpi=80)
        print(f"animation saved to {save}")
    if show:
        plt.show()
    plt.close(fig)
    return anim


def _wrap(text, width):
    out = []
    while len(text) > width:
        cut = text.rfind(" ", 0, width)
        cut = cut if cut > 0 else width
        out.append(text[:cut])
        text = "      " + text[cut:].lstrip()
    out.append(text)
    return "\n".join(out)
