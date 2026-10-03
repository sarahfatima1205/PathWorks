"""Discrete-time execution of the agents' plans with online disruptions."""

import random
import time
from collections import defaultdict

from .grid import INF
from .planner import Reservations, goal_times, space_time_astar
from .repair import Repairer

RETRY_INTERVAL = 3  # a stalled agent retries planning every few steps


class Simulation:
    def __init__(self, grid, agents, tasks, mode="repair", seed=0,
                 comm_radius=6, delay_tolerance=6, negotiation=True,
                 verbose=False, record=False):
        self.grid = grid
        self.agents = agents
        self.tasks = list(tasks)
        self.mode = mode            # "repair" (ours) | "replan" (baseline)
        self.rng = random.Random(seed)
        self.verbose = verbose
        self.record = record
        self.t = 0
        self.pending = defaultdict(list)
        self.log = []               # (t, message)
        self.records = []           # one EventRecord per disruption
        self.frames = []
        self.collisions = 0
        self.last_change = {}       # agent id -> time its plan last changed
        self.initial_total = 0
        self.initial_makespan = 0
        self.plan_ms = 0.0
        self.repairer = Repairer(self, comm_radius=comm_radius,
                                 delay_tolerance=delay_tolerance,
                                 negotiation=negotiation)

    # ---------------------------------------------------------- helpers
    def say(self, msg):
        self.log.append((self.t, msg))
        if self.verbose:
            print(f"[t={self.t:3d}] {msg}")

    def reservations(self, t, exclude=(), only=None):
        """Reservation table of the current plans from time t on."""
        res = Reservations()
        for a in self.agents:
            if a.id in exclude or (only is not None and a.id not in only):
                continue
            res.add(a.id, a.path, t)
        return res

    def drop_unreachable(self, a, t):
        """Give up goals that permanent obstacles have cut off."""
        cur = a.at(t)
        keep = a.goals[:a.gi]
        dead = set()
        for g in a.goals[a.gi:]:
            if g.task is not None and g.task.id in dead:
                continue
            if self.grid.dist(cur, g.cell) == INF:
                if g.task is not None:
                    dead.add(g.task.id)
                    g.task.status = "lost"
                    self.say(f"A{a.id}: task {g.task.id} is unreachable, dropped")
                continue
            keep.append(g)
            cur = g.cell
        a.goals = [g for g in keep
                   if g.task is None or g.task.id not in dead]

    def plan_agent(self, a, t, hard, soft=None, penalty=0):
        """Re-plan agent `a` from its position at time t through its remaining
        goals. Returns a complete new path (executed prefix included) or None.
        A temporarily broken-down agent first waits until it is operational."""
        self.drop_unreachable(a, t)
        pos = a.at(t)
        ts = max(t, a.frozen_until)
        tail = space_time_astar(self.grid, a.id, pos, ts, a.remaining_cells(),
                                hard, soft, penalty)
        if tail is None:
            return None
        return a.prefix(t)[:t] + [pos] * (ts - t) + tail

    def set_path(self, a, path):
        a.path = path
        if a.status == "stalled":
            a.status = "active"

    # --------------------------------------------------- initial planning
    def plan_initial(self):
        """Prioritized planning: agents plan one after another with Space-Time
        A*, each treating the earlier agents' paths as reservations."""
        start = time.perf_counter()
        for a in self.agents:
            a.path = [a.home]
        hard = Reservations()
        for a in self.agents:
            path = self.plan_agent(a, 0, hard)
            if path is None:
                a.status, a.stalled_since = "stalled", 0
                self.say(f"A{a.id}: no initial plan, waiting at home")
            else:
                a.path = path
            hard.add(a.id, a.path, 0)
        self.plan_ms = (time.perf_counter() - start) * 1000
        finishes = [self.planned_finish(a) for a in self.agents]
        self.initial_total = sum(finishes)
        self.initial_makespan = max(finishes, default=0)
        self._progress()
        self._snapshot()

    def planned_finish(self, a):
        """Time at which the current plan completes the agent's last delivery."""
        cells = [g.cell for g in a.goals[a.gi:] if g.kind != "home"]
        if not cells:
            return a.finish_time
        times = goal_times(a.path, min(self.t, len(a.path) - 1), cells)
        return times[-1] if times[-1] is not None else len(a.path) - 1

    # ------------------------------------------------------------ events
    def add_events(self, events):
        for ev in events:
            self.pending[ev.t].append(ev)

    def find_conflicts(self, t):
        """First vertex/edge conflict between the current plans from time t."""
        occ, rest = {}, {}
        for a in self.agents:
            p = a.path
            for tau in range(t, len(p)):
                key = (p[tau], tau)
                if key in occ:
                    return occ[key], a
                occ[key] = a
            rest[p[-1]] = (len(p) - 1, a)
        for a in self.agents:
            p = a.path
            for tau in range(t, len(p)):
                r = rest.get(p[tau])
                if r is not None and r[1] is not a and tau >= r[0]:
                    return a, r[1]
                if tau + 1 < len(p) and p[tau] != p[tau + 1]:
                    b = occ.get((p[tau + 1], tau))
                    if b is not None and b is not a \
                            and occ.get((p[tau], tau + 1)) is b:
                        return a, b
        return None

    def blocked_agent(self, t):
        """An operational agent whose plan runs into a blocked cell, if any."""
        for a in self.agents:
            if a.status == "broken":
                continue
            p = a.path
            for tau in range(t + 1, len(p)):
                if self.grid.is_blocked(p[tau], tau):
                    return a
            if self.grid.is_blocked(p[-1], max(t + 1, len(p))) \
                    and self.grid.perm_blocked(p[-1]):
                return a
        return None

    # --------------------------------------------------------- execution
    def step(self):
        from .scenario import resolve_event
        t = self.t
        for ev in self.pending.pop(t, []):
            if resolve_event(self, ev):
                self.records.append(self.repairer.handle(ev, t))
        for a in self.agents:
            if a.status == "stalled" and t > a.stalled_since \
                    and (t - a.stalled_since) % RETRY_INTERVAL == 0:
                path = self.plan_agent(a, t, self.reservations(t, {a.id}))
                if path is not None:
                    self.set_path(a, path)
                    self.say(f"A{a.id}: found a way out, resuming")

        seen = {}
        for a in self.agents:
            c = a.at(t + 1)
            other = seen.get(c)
            if other is not None:
                self.collisions += 1
                self.say(f"COLLISION A{a.id}/A{other.id} at {c}")
            seen[c] = a
        for a in self.agents:
            b = seen.get(a.at(t))
            if b is not None and b is not a and a.at(t) != a.at(t + 1) \
                    and b.at(t) == a.at(t + 1):
                self.collisions += 1
                self.say(f"SWAP COLLISION A{a.id}/A{b.id}")
        self.t = t + 1
        self._progress()
        self._snapshot()

    def _progress(self):
        """Mark goals reached at the current time (pickups / deliveries)."""
        t = self.t
        for a in self.agents:
            if a.status == "broken":
                continue
            pos = a.at(t)
            while a.gi < len(a.goals) and a.goals[a.gi].cell == pos:
                g = a.goals[a.gi]
                a.gi += 1
                if g.kind == "pickup":
                    g.task.status, g.task.t_pickup = "carried", t
                elif g.kind == "delivery":
                    g.task.status, g.task.t_delivery = "delivered", t
                    a.finish_time = t
                    a.deliveries += 1

    def _snapshot(self):
        if not self.record:
            return
        self.frames.append({
            "t": self.t,
            "pos": [a.at(self.t) for a in self.agents],
            "status": [("frozen" if a.frozen_until > self.t and
                        a.status == "active" else a.status)
                       for a in self.agents],
            "paths": [a.path for a in self.agents],
            "blocked": self.grid.blocked_at(self.t),
            "tasks": [(k.pickup, k.delivery, k.status, k.owner, k.emergency)
                      for k in self.tasks],
            "hot": [i for i, tc in self.last_change.items()
                    if self.t - tc <= 6],
            "log": len(self.log),
        })

    def finished(self):
        return all(a.status == "broken" or a.done(self.t) for a in self.agents)

    def run(self, max_t=None):
        if max_t is None:
            max_t = 4 * self.initial_makespan + 200
        while not self.finished() and self.t < max_t:
            self.step()
        return self.results()

    # ----------------------------------------------------------- metrics
    def results(self):
        status = defaultdict(int)
        for k in self.tasks:
            status[k.status] += 1
        recs = self.records
        hit = [r for r in recs if r.affected]
        return {
            "agents": len(self.agents),
            "tasks": len(self.tasks),
            "delivered": status["delivered"],
            "lost": status["lost"],
            "unfinished": status["pending"] + status["carried"],
            "total_time": sum(a.finish_time for a in self.agents),
            "makespan": max(a.finish_time for a in self.agents),
            "initial_total": self.initial_total,
            "initial_makespan": self.initial_makespan,
            "disruptions": len(recs),
            "disruptions_hit": len(hit),
            "affected_per_disruption":
                sum(len(r.affected) for r in recs) / len(recs) if recs else 0.0,
            "changed_per_disruption":
                sum(len(r.changed) for r in recs) / len(recs) if recs else 0.0,
            "changed_per_hit":
                sum(len(r.changed) for r in hit) / len(hit) if hit else 0.0,
            "by_kind": {
                kind: [sum(len(r.changed) for r in recs if r.kind == kind),
                       sum(1 for r in recs if r.kind == kind)]
                for kind in ("block", "breakdown", "emergency")},
            "max_changed": max((len(r.changed) for r in recs), default=0),
            "messages": sum(r.messages for r in recs),
            "yielders": sum(r.yielders for r in recs),
            "stalls": sum(len(r.stalled) for r in recs),
            "repair_ms": sum(r.cpu_ms for r in recs),
            "plan_ms": self.plan_ms,
            "collisions": self.collisions,
            "steps": self.t,
        }
