# PathWorks
A Negotiation-Based Plan Repair Framework for Multi-Agent Warehouse Navigation

A team of robots moves on a 2D warehouse grid, each picking up objects and
carrying them to delivery stations. Paths are planned with Space-Time A*.
While the plans are being executed, cells get blocked, robots break down and
emergency tasks arrive. The affected robots repair their plans locally,
negotiating with neighbours when they cannot solve the problem alone, without
re-running the multi-agent planner and while changing as few plans as possible.

![demo](results/demo.gif)

## Running

Needs Python 3.10+ and matplotlib (the planner itself uses only the standard
library).

```bash
pip install -r requirements.txt

python main.py demo                         # one simulation, animated
python main.py demo --agents 20 --density 0.05 --seed 3
python main.py demo --save run.gif --no-show
python main.py demo --mode replan           # baseline: full replanning
python main.py experiments                  # sweeps -> results/
```

`python main.py demo --help` lists all options (team size, tasks per agent,
obstacle density, number of breakdowns and emergencies, share of temporary
disruptions, communication radius, delay tolerance, `--no-negotiation`).

## Model

- **Map** (`code/grid.py`): 32 x 20 grid, 4-connected moves plus wait,
  one step per time step. Shelf blocks with one-cell aisles, a ring corridor,
  delivery stations on the right wall. Every robot has a private parking cell
  that no other robot may enter, so a parked robot never blocks anyone.
- **Tasks**: each robot has an ordered list of (pickup cell, delivery station)
  tasks and returns to its parking cell afterwards.
- **Conflicts**: two robots may not be in the same cell at the same time
  (vertex conflict) or swap cells (edge conflict).
- **Initial plan** (`simulation.py: plan_initial`): prioritized planning.
  Robots plan one by one with Space-Time A* (`planner.py`) over states
  (cell, time, next-goal index), reserving their path in a space-time
  reservation table. The heuristic is the exact BFS distance through the
  remaining goals.

## Disruptions

| Type | Effect |
|---|---|
| Blocked cell | An aisle cell becomes an obstacle, permanently or for a known duration. Robots whose remaining path uses it are disrupted. |
| Breakdown | Permanent: the robot becomes an obstacle and all its unfinished tasks are re-allocated; a load it was carrying is collected from the cell next to it. Temporary: it stands still for a number of steps. |
| Emergency task | A robot receives a high-priority pickup and delivery that it must serve before its other work. |

Random blockages and breakdowns never cut any cell off from the rest of the
warehouse and never land on a task's pickup cell, so every task stays
achievable.

## Plan repair algorithm (`code/repair.py`)

Only disrupted robots react, and each one escalates only as far as needed:

0. **Self-repair.** The robot re-plans its own remaining route with Space-Time
   A*, treating every other robot's current plan as a hard reservation. If a
   route exists and costs at most `delay_tolerance` extra steps it is adopted
   and nobody else changes.
1. **Negotiation with neighbours.** Otherwise the robot plans a route in which
   collisions with robots inside its communication radius are allowed but
   penalised, which yields the route that needs the fewest neighbours to move.
   It sends a yield REQUEST to exactly the neighbours that route collides
   with. Each of them tries to re-plan around the proposal and answers ACCEPT
   (with its extra delay). A neighbour that cannot fit its own tasks around
   the proposal offers to step aside to the nearest cell where it is in
   nobody's way and resume later; if even that is impossible it answers
   REJECT, becomes a hard constraint, and the initiator proposes again. The deal is committed only if
   it saves at least `gain_per_agent` steps per extra robot it alters compared
   with self-repair; an emergency task always takes the faster option.
2. **Broadcast.** The same protocol with the whole team as candidates.
3. **Safe stop.** The robot holds its cell and retries with increasing
   back-off, again alone first and then by negotiating; robots routed through
   that cell repair themselves. Plans changed by a retry are charged to the
   disruption that caused the stop.

A broken robot's open tasks are re-allocated by a contract-net auction. Bids
are estimated completion times, with a penalty for robots whose plan is still
untouched, so the work goes to robots that already had to change. The winner
keeps its plan up to its last delivery and only plans the new tail.

Every step is a single-agent search against the reservations of the plans that
stay as they are. A final consistency check verifies that the joint plan is
collision-free after each repair.

## Evaluation

`python main.py experiments` runs 780 simulations (20 seeds per setting, three
methods) and writes `results/runs.csv`, `results/summary.csv` and the plots.
Each run has two tasks per robot, one permanent breakdown, one emergency task
and `density x (number of aisle cells)` permanent blockages at random times.

Methods compared:

- **Repair**: the algorithm above.
- **Full replan** (`--mode replan`): baseline that discards all plans at every
  disruption and re-runs prioritized planning for the whole team from the
  current positions.
- **Self-repair only** (`--no-negotiation`): ablation without levels 1 and 2.

Metrics:

- **Total time steps**: sum over robots of the time of their last delivery.
- **Plans changed per disruption**: robots whose remaining path differs after
  the repair, including the directly disrupted ones.

Team size, at 5% obstacle density:

| Robots | Original plan | Repair: total | Replan: total | Directly disrupted | Repair: plans changed | Replan: plans changed |
|---:|---:|---:|---:|---:|---:|---:|
| 5  | 425.0  | 464.7  | 471.9  | 1.10 | 1.16 | 1.39 |
| 10 | 880.6  | 956.3  | 950.0  | 1.99 | 2.04 | 3.02 |
| 15 | 1327.7 | 1413.4 | 1422.6 | 2.53 | 2.59 | 4.37 |
| 20 | 1794.1 | 1921.4 | 1896.3 | 3.58 | 3.70 | 6.65 |
| 25 | 2300.4 | 2495.5 | 2497.8 | 4.99 | 5.11 | 9.56 |
| 30 | 2799.6 | 3060.2 | 3033.4 | 5.97 | 6.14 | 12.28 |
| 40 | 3807.1 | 4152.6 | 4134.2 | 7.29 | 7.51 | 17.74 |

Obstacle density, with 20 robots (original plan: 1794.1 steps):

| Density | Repair: total | Overhead | Replan: total | Repair: plans changed | Replan: plans changed |
|---:|---:|---:|---:|---:|---:|
| 0%   | 1835.4 | 2.3%  | 1824.9 | 3.12 | 10.95 |
| 2.5% | 1864.9 | 3.9%  | 1857.3 | 3.65 | 7.01 |
| 5%   | 1921.4 | 7.1%  | 1896.3 | 3.70 | 6.65 |
| 7.5% | 2047.9 | 14.2% | 2022.1 | 3.98 | 6.95 |
| 10%  | 2141.5 | 19.5% | 2162.1 | 4.18 | 7.05 |
| 15%  | 2627.1 | 46.6% | 2640.1 | 4.47 | 7.29 |

Findings:

- **Plans changed.** Repair alters almost exactly the robots the disruption
  hit directly (0.05 to 0.25 extra per disruption; 0.5 at 0% density, where
  the only disruptions are the breakdown and the emergency task). Full replanning alters
  about twice as many, and the gap widens with team size: 7.5 against 17.7
  plans at 40 robots.
- **Total time.** Repair and full replanning are within about 1.5% of each
  other in every setting, with neither consistently ahead. Local repair costs
  essentially nothing in solution quality.
- **Team size.** Total time grows linearly with the number of robots and the
  overhead over the undisturbed plan stays at 6 to 10%. Plans changed per
  disruption grows with team size because more robots route through any
  given cell.
- **Obstacle density.** Overhead grows faster than linearly, from 2% with no
  blockages to 47% at 15%, as detours get longer and aisles turn into dead
  ends. Plans changed per disruption rises only slightly (3.1 to 4.5).
- **By disruption type.** Repair changes 4.1 plans for a blocked cell, 7.4 for
  a breakdown (including the robots that take over its tasks) and 1.6 for an
  emergency task; replanning changes 6.9, 13.2 and 12.7.
- **What negotiation buys.** About 96% of disrupted robots repair alone at
  level 0 (sampled over 40 runs). Negotiation
  matters for the rest: without it, robots that cannot re-plan alone block
  each other, and 5 of 260 runs end with tasks never delivered (41 tasks);
  with it every run completes. It does not noticeably shorten emergency
  response time (45 against 47 steps at 20 robots).
- **Safety and completion.** No collisions in any of the 780 runs. Repair
  delivers 99.9% of tasks; the remainder are tasks whose pickup cell ended up
  under a broken-down robot.

## Layout

```
main.py                  command line entry point
code/grid.py        warehouse map, dynamic obstacles, BFS distance maps
code/planner.py     Space-Time A*, reservation table
code/agent.py       Agent, Task, Goal
code/simulation.py  initial planning, execution loop, metrics
code/repair.py      plan repair, negotiation, auction, replan baseline
code/scenario.py    instance and disruption generation
code/visualize.py   animation
code/experiments.py sweeps, CSV output, plots
```
