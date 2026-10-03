"""Batch experiments: effect of team size and dynamic-obstacle density."""

import csv
import os
from multiprocessing import Pool
from statistics import mean, stdev

from .scenario import make_simulation

AGENT_COUNTS = [5, 10, 15, 20, 25, 30, 40]
DENSITIES = [0.0, 0.025, 0.05, 0.075, 0.10, 0.15]
MODES = ["repair", "replan"]
MODE_LABEL = {"repair": "PathWorks plan repair", "replan": "Full replan (baseline)"}
MODE_COLOR = {"repair": "#2a78d6", "replan": "#eb6834"}
MUTED = "#898781"
KINDS = ["block", "breakdown", "emergency"]
FIELDS = ["experiment", "mode", "agents", "density", "seed", "tasks",
          "delivered", "lost", "unfinished", "total_time", "makespan",
          "initial_total", "overhead_pct", "disruptions", "disruptions_hit",
          "affected_per_disruption", "changed_per_disruption",
          "changed_per_hit", "max_changed", "messages", "yielders", "stalls",
          "repair_ms", "plan_ms", "collisions",
          "changed_block", "n_block", "changed_breakdown", "n_breakdown",
          "changed_emergency", "n_emergency"]


def run_one(job):
    experiment, mode, n_agents, density, seed = job
    sim = make_simulation(n_agents=n_agents, density=density, seed=seed,
                          mode=mode)
    r = sim.run()
    r.update(experiment=experiment, mode=mode, density=density, seed=seed)
    r["overhead_pct"] = 100.0 * (r["total_time"] - r["initial_total"]) \
        / max(1, r["initial_total"])
    for kind, (changed, count) in r.pop("by_kind").items():
        r[f"changed_{kind}"], r[f"n_{kind}"] = changed, count
    return r


def run_all(out_dir="results", seeds=10, fixed_agents=20, fixed_density=0.05,
            workers=None):
    os.makedirs(out_dir, exist_ok=True)
    jobs = []
    for mode in MODES:
        for seed in range(seeds):
            jobs += [("agents", mode, n, fixed_density, seed)
                     for n in AGENT_COUNTS]
            jobs += [("density", mode, fixed_agents, d, seed)
                     for d in DENSITIES]
    print(f"running {len(jobs)} simulations ...")
    with Pool(workers) as pool:
        rows = pool.map(run_one, jobs, chunksize=1)

    with open(os.path.join(out_dir, "runs.csv"), "w", newline="") as f:
        writer = csv.DictWriter(f, FIELDS, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)

    summary = summarise(rows)
    with open(os.path.join(out_dir, "summary.csv"), "w", newline="") as f:
        writer = csv.DictWriter(f, list(summary[0]))
        writer.writeheader()
        writer.writerows(summary)
    print_summary(summary)
    plot_all(rows, out_dir, fixed_agents, fixed_density)
    print(f"\nresults written to {out_dir}/")
    return rows


def summarise(rows):
    out = []
    for exp, key in (("agents", "agents"), ("density", "density")):
        for x in sorted({r[key] for r in rows if r["experiment"] == exp}):
            for mode in MODES:
                sel = [r for r in rows if r["experiment"] == exp
                       and r[key] == x and r["mode"] == mode]
                avg = lambda k: round(mean(r[k] for r in sel), 2)
                out.append({
                    "experiment": exp, "x": x, "mode": mode, "runs": len(sel),
                    "total_time": avg("total_time"),
                    "no_disruption_total": avg("initial_total"),
                    "overhead_pct": avg("overhead_pct"),
                    "makespan": avg("makespan"),
                    "disruptions": avg("disruptions"),
                    "affected_per_disruption": avg("affected_per_disruption"),
                    "changed_per_disruption": avg("changed_per_disruption"),
                    "changed_per_hit": avg("changed_per_hit"),
                    "max_changed": max(r["max_changed"] for r in sel),
                    "messages": avg("messages"),
                    "delivered_pct": round(100 * sum(r["delivered"] for r in sel)
                                           / sum(r["tasks"] for r in sel), 1),
                    "repair_ms": avg("repair_ms"),
                    "collisions": sum(r["collisions"] for r in sel),
                })
    return out


def print_summary(summary):
    cols = ["x", "mode", "total_time", "no_disruption_total", "overhead_pct",
            "affected_per_disruption", "changed_per_disruption",
            "max_changed", "messages", "delivered_pct", "repair_ms"]
    for exp in ("agents", "density"):
        print(f"\n=== varying {exp} ===")
        print("  ".join(f"{c[:14]:>14}" for c in cols))
        for row in summary:
            if row["experiment"] == exp:
                print("  ".join(f"{str(row[c]):>14}" for c in cols))


# ------------------------------------------------------------------ plots
def _series(rows, exp, key, mode, metric):
    xs = sorted({r[key] for r in rows if r["experiment"] == exp})
    means, errs = [], []
    for x in xs:
        vals = [r[metric] for r in rows if r["experiment"] == exp
                and r[key] == x and r["mode"] == mode]
        means.append(mean(vals))
        errs.append(stdev(vals) / len(vals) ** 0.5 if len(vals) > 1 else 0)
    return xs, means, errs


def _line_chart(rows, exp, key, metric, reference, title, xlabel, ylabel,
                path, percent_x=False):
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(6.4, 4.2))
    scale = 100 if percent_x else 1
    if reference:
        ref_metric, ref_label = reference
        xs, ms, _ = _series(rows, exp, key, "repair", ref_metric)
        ax.plot([x * scale for x in xs], ms, color=MUTED, lw=2, ls="--",
                marker="s", ms=5, label=ref_label)
    for mode in MODES:
        xs, ms, es = _series(rows, exp, key, mode, metric)
        ax.errorbar([x * scale for x in xs], ms, yerr=es, color=MODE_COLOR[mode],
                    lw=2, marker="o", ms=6, capsize=3, label=MODE_LABEL[mode])
    _style(ax, title, xlabel, ylabel)
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)


def _style(ax, title, xlabel, ylabel):
    ax.set_title(title, loc="left", fontsize=11, fontweight="bold")
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    ax.set_ylim(bottom=0)
    ax.grid(axis="y", color="#e6e5e0", lw=0.8)
    ax.set_axisbelow(True)
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)
    ax.legend(frameon=False, fontsize=9)


def plot_all(rows, out_dir, fixed_agents, fixed_density):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    d_txt = f"{fixed_density:.1%} obstacle density"
    a_txt = f"{fixed_agents} agents"
    join = lambda name: os.path.join(out_dir, name)
    _line_chart(rows, "agents", "agents", "total_time",
                ("initial_total", "Original plan (no disruptions)"),
                f"Total time steps vs. team size\n({d_txt})",
                "Number of agents", "Sum of task completion times (steps)",
                join("total_time_vs_agents.png"))
    _line_chart(rows, "agents", "agents", "changed_per_disruption",
                ("affected_per_disruption", "Directly disrupted agents"),
                f"Plans changed per disruption vs. team size\n({d_txt})",
                "Number of agents", "Agents whose plan changed",
                join("changed_vs_agents.png"))
    _line_chart(rows, "density", "density", "total_time",
                ("initial_total", "Original plan (no disruptions)"),
                f"Total time steps vs. obstacle density\n({a_txt})",
                "Dynamic obstacle density (% of aisle cells)",
                "Sum of task completion times (steps)",
                join("total_time_vs_density.png"), percent_x=True)
    _line_chart(rows, "density", "density", "changed_per_disruption",
                ("affected_per_disruption", "Directly disrupted agents"),
                f"Plans changed per disruption vs. obstacle density\n({a_txt})",
                "Dynamic obstacle density (% of aisle cells)",
                "Agents whose plan changed",
                join("changed_vs_density.png"), percent_x=True)

    # plans changed per disruption, by disruption type
    fig, ax = plt.subplots(figsize=(6.4, 4.2))
    width = 0.36
    for i, mode in enumerate(MODES):
        sel = [r for r in rows if r["mode"] == mode]
        vals = [sum(r[f"changed_{k}"] for r in sel)
                / max(1, sum(r[f"n_{k}"] for r in sel)) for k in KINDS]
        xs = [j + (i - 0.5) * (width + 0.03) for j in range(len(KINDS))]
        bars = ax.bar(xs, vals, width, color=MODE_COLOR[mode],
                      label=MODE_LABEL[mode])
        ax.bar_label(bars, fmt="%.1f", fontsize=9, padding=2)
    ax.set_xticks(range(len(KINDS)))
    ax.set_xticklabels(["Blocked cell", "Agent breakdown", "Emergency task"])
    _style(ax, "Plans changed per disruption, by disruption type (all runs)",
           "", "Agents whose plan changed")
    fig.tight_layout()
    fig.savefig(join("changed_by_type.png"), dpi=160)
    plt.close(fig)
