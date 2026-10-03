"""Scenario generation: warehouse instances and random disruptions."""

import random
from dataclasses import dataclass

from .agent import Agent, Goal, Task
from .grid import INF, make_warehouse
from .simulation import Simulation


@dataclass
class Event:
    """A disruption. Fields left as None are chosen at random when it fires.

    kind = "block"      cell becomes an obstacle (duration None = permanent)
           "breakdown"  agent stops (duration None = permanent)
           "emergency"  agent must first serve a new pickup -> delivery task
    """
    t: int
    kind: str
    cell: tuple = None
    agent: int = None
    duration: int = None
    pickup: tuple = None
    delivery: tuple = None


def build_scenario(n_agents=10, tasks_per_agent=2, width=32, height=20,
                   seed=0, pace_min=0.8, pace_max=1.8):
    rng = random.Random(seed)
    grid = make_warehouse(width, height, n_agents)
    homes = sorted(grid.home_owner, key=grid.home_owner.get)
    agents, tasks = [], []
    for i, home in enumerate(homes):
        agent = Agent(i, home, pace=rng.uniform(pace_min, pace_max))
        for _ in range(tasks_per_agent):
            task = Task(len(tasks), rng.choice(grid.pick_cells),
                        rng.choice(grid.stations), i)
            tasks.append(task)
            agent.goals += [Goal(task.pickup, "pickup", task),
                            Goal(task.delivery, "delivery", task)]
        agent.goals.append(Goal(home, "home"))
        agents.append(agent)
    return grid, agents, tasks


def random_events(rng, horizon, n_blocks=0, n_breakdowns=0, n_emergencies=0,
                  temporary=0.0):
    """Disruptions at random times in the first 80% of the nominal run."""
    last = max(2, int(0.8 * horizon))
    events = []
    for kind, n in (("block", n_blocks), ("breakdown", n_breakdowns),
                    ("emergency", n_emergencies)):
        for _ in range(n):
            dur = rng.randint(5, 20) if rng.random() < temporary else None
            events.append(Event(rng.randint(1, last), kind,
                                duration=dur if kind != "emergency" else None))
    return sorted(events, key=lambda e: e.t)


def resolve_event(sim, ev):
    """Fill in the random parts of an event for the current world state.
    Returns False if the event cannot happen (and is skipped)."""
    grid, rng, t = sim.grid, sim.rng, sim.t
    here = {a.at(t) for a in sim.agents}

    if ev.kind == "block":
        if ev.cell is not None:
            return ev.cell not in here and ev.cell not in grid.blocked
        goal_cells = {g.cell for a in sim.agents for g in a.goals[a.gi:]}
        cells = [c for c in grid.aisle_cells
                 if c not in here and c not in grid.blocked
                 and c not in goal_cells]
        rng.shuffle(cells)
        for c in cells:
            # never cut the warehouse in two: every task stays reachable
            if ev.duration is not None or grid.connected_if_blocked(c):
                ev.cell = c
                return True
        return False

    working = [a for a in sim.agents if a.status == "active"
               and a.frozen_until <= t and a.gi < len(a.goals) - 1]
    if ev.kind == "breakdown":
        if ev.agent is None:
            cand = [a for a in working if a.at(t) in grid.aisle_cells
                    and (ev.duration is not None
                         or grid.connected_if_blocked(a.at(t)))]
            if not cand:
                return False
            ev.agent = rng.choice(cand).id
        return sim.agents[ev.agent].status == "active"

    if ev.kind == "emergency":
        if ev.agent is None:
            if not working:
                return False
            ev.agent = rng.choice(working).id
        a = sim.agents[ev.agent]
        if a.status != "active":
            return False
        if ev.pickup is None:
            reach = [c for c in grid.pick_cells if not grid.perm_blocked(c)
                     and grid.dist(a.at(t), c) < INF and c != a.at(t)]
            ev.pickup = rng.choice(reach)
        if ev.delivery is None:
            ev.delivery = rng.choice(
                [s for s in grid.stations if grid.dist(ev.pickup, s) < INF])
        return True
    return False


def make_simulation(n_agents=10, tasks_per_agent=2, density=0.05,
                    n_breakdowns=1, n_emergencies=1, temporary=0.0, seed=0,
                    width=32, height=20, events=None, pace_min=0.8,
                    pace_max=1.8, **sim_kwargs):
    """Build an instance, plan it, and schedule disruptions.

    `density` is the fraction of aisle cells that turn into dynamic obstacles
    during the run.

    `pace_min` and `pace_max` define the bounds for per-agent movement speeds.
    Values greater than 1.0 make a robot move faster than the baseline pace.
    """
    grid, agents, tasks = build_scenario(n_agents, tasks_per_agent, width,
                                         height, seed, pace_min, pace_max)
    sim = Simulation(grid, agents, tasks, seed=seed + 7919, **sim_kwargs)
    sim.plan_initial()
    if events is None:
        n_blocks = round(density * len(grid.aisle_cells))
        events = random_events(random.Random(seed + 104729),
                               sim.initial_makespan, n_blocks, n_breakdowns,
                               n_emergencies, temporary)
    sim.add_events(events)
    return sim
