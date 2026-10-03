#!/usr/bin/env python3
"""PathWorks: negotiation-based plan repair for multi-agent warehouse navigation.

    python main.py demo                 # one simulation with animation
    python main.py demo --save run.gif --no-show
    python main.py experiments          # agent-count and density sweeps
"""

import argparse
from collections import Counter


def demo(args):
    from code.scenario import make_simulation
    visual = args.save or not args.no_show
    sim = make_simulation(
        n_agents=args.agents, tasks_per_agent=args.tasks, density=args.density,
        n_breakdowns=args.breakdowns, n_emergencies=args.emergencies,
        temporary=args.temporary, seed=args.seed, mode=args.mode,
        comm_radius=args.radius, delay_tolerance=args.tolerance,
        negotiation=not args.no_negotiation, verbose=not args.quiet,
        record=visual)
    r = sim.run()

    print("\nDisruptions")
    print(f"{'t':>4}  {'type':<10} {'direct':>6} {'changed':>7} {'msgs':>5} "
          f"{'ms':>7}  how / what")
    for rec in sim.records:
        how = ", ".join(f"{n}x {lvl}" for lvl, n in
                        Counter(rec.levels.values()).items()) or "-"
        print(f"{rec.t:>4}  {rec.kind:<10} {len(rec.affected):>6} "
              f"{len(rec.changed):>7} {rec.messages:>5} {rec.cpu_ms:>7.1f}  "
              f"{rec.detail}  [{how}]")

    print("\nSummary")
    print(f"  agents / tasks              {r['agents']} / {r['tasks']}")
    print(f"  tasks delivered / lost      {r['delivered']} / {r['lost']}"
          f"   (unfinished {r['unfinished']})")
    print(f"  total time steps            {r['total_time']}"
          f"   (original plan: {r['initial_total']})")
    print(f"  makespan                    {r['makespan']}"
          f"   (original plan: {r['initial_makespan']})")
    print(f"  disruptions                 {r['disruptions']}"
          f"   ({r['disruptions_hit']} hit at least one plan)")
    print(f"  plans changed / disruption  {r['changed_per_disruption']:.2f}"
          f"   (max {r['max_changed']}, directly disrupted "
          f"{r['affected_per_disruption']:.2f})")
    print(f"  negotiation messages        {r['messages']}")
    print(f"  collisions                  {r['collisions']}")

    if visual:
        import matplotlib
        if args.no_show:
            matplotlib.use("Agg")
        from code.visualize import animate
        animate(sim, save=args.save, show=not args.no_show,
                substeps=args.substeps, fps=args.fps)


def experiments(args):
    from code.experiments import run_all
    run_all(args.out, seeds=args.seeds, workers=args.workers)


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="command", required=True)

    d = sub.add_parser("demo", help="run and animate one simulation")
    d.add_argument("--agents", type=int, default=12)
    d.add_argument("--tasks", type=int, default=2, help="tasks per agent")
    d.add_argument("--density", type=float, default=0.03,
                   help="fraction of aisle cells that become obstacles")
    d.add_argument("--breakdowns", type=int, default=1)
    d.add_argument("--emergencies", type=int, default=1)
    d.add_argument("--temporary", type=float, default=0.0,
                   help="probability that a blockage/breakdown is temporary")
    d.add_argument("--seed", type=int, default=1)
    d.add_argument("--mode", choices=["repair", "replan"], default="repair",
                   help="repair = negotiated local repair, replan = baseline")
    d.add_argument("--radius", type=int, default=6, help="communication radius")
    d.add_argument("--tolerance", type=int, default=6,
                   help="delay an agent accepts before asking neighbours")
    d.add_argument("--no-negotiation", action="store_true")
    d.add_argument("--save", help="write the animation to this .gif")
    d.add_argument("--no-show", action="store_true")
    d.add_argument("--substeps", type=int, default=3)
    d.add_argument("--fps", type=int, default=12)
    d.add_argument("--quiet", action="store_true")
    d.set_defaults(func=demo)

    e = sub.add_parser("experiments", help="run the evaluation sweeps")
    e.add_argument("--seeds", type=int, default=20)
    e.add_argument("--out", default="results")
    e.add_argument("--workers", type=int, default=None)
    e.set_defaults(func=experiments)

    args = p.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
