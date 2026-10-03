"""Negotiation-based plan repair.

When a disruption invalidates some plans, only the disrupted agents react, and
they escalate in levels so that as few plans as possible are touched:

  Level 0  self-repair   The agent re-plans its own remaining route with
                         Space-Time A*, treating every other agent's current
                         plan as a hard reservation. Nobody else changes.
  Level 1  negotiation   If that is impossible (or too slow), the agent plans
                         a route in which collisions with *neighbouring* agents
                         are allowed but penalised, and sends a yield REQUEST
                         to exactly the neighbours that route collides with.
                         Each neighbour tries to re-plan around the proposal
                         and answers ACCEPT (with its extra delay) or REJECT.
                         A rejecting neighbour becomes a hard constraint and
                         the initiator proposes again. The deal is committed
                         only if it beats self-repair by enough to justify
                         altering the extra agents.
  Level 2  broadcast     Same protocol with the whole team as candidates.
  Level 3  safe stop     The agent holds its cell and retries periodically;
                         agents heading through that cell repair themselves.

The multi-agent planner is never re-run: every step is a single-agent search
against the reservations of plans that stay as they are. Tasks orphaned by a
broken-down robot are re-allocated with a contract-net auction whose bids
favour agents that already had to change their plan.
"""

import time
from dataclasses import dataclass, field

from .grid import INF
from .planner import Reservations, future, goal_times, path_conflicts, space_time_astar


RETRY_INTERVAL = 3        # a holding agent retries after this many steps,
MAX_RETRY_INTERVAL = 24   # backing off up to this
ASIDE_HORIZON = 40


@dataclass
class EventRecord:
    t: int
    kind: str
    detail: str = ""
    affected: list = field(default_factory=list)   # directly disrupted agents
    changed: list = field(default_factory=list)    # agents whose plan changed
    touched: set = field(default_factory=set)
    yielders: int = 0      # neighbours that gave way through negotiation
    messages: int = 0
    stalled: list = field(default_factory=list)
    levels: dict = field(default_factory=dict)     # agent id -> repair level
    added_steps: int = 0
    cpu_ms: float = 0.0


class Repairer:
    def __init__(self, sim, comm_radius=6, delay_tolerance=6, negotiation=True,
                 conflict_penalty=8, priority_penalty=2, gain_per_agent=3,
                 change_penalty=10):
        self.sim = sim
        self.comm_radius = comm_radius
        self.delay_tolerance = delay_tolerance    # self-repair delay accepted
        self.negotiation = negotiation
        self.conflict_penalty = conflict_penalty
        self.priority_penalty = priority_penalty  # emergency: neighbours yield
        self.gain_per_agent = gain_per_agent      # steps saved per extra agent
        self.change_penalty = change_penalty      # auction bias to changed agents
        self._aside = set()   # yielders of the latest deal that only step aside

    # ============================================================ entry
    def handle(self, ev, t):
        sim = self.sim
        start = time.perf_counter()
        rec = EventRecord(t=t, kind=ev.kind)
        before = {a.id: (future(a.path, t), len(a.path)) for a in sim.agents}
        queue, orphans, vip, broken = [], [], None, None

        if ev.kind == "block":
            t_end = INF if ev.duration is None else t + 1 + ev.duration
            sim.grid.block(ev.cell, t + 1, t_end)
            queue = self._hit_by(ev.cell, t, t_end)
            rec.detail = f"cell {ev.cell} blocked" + (
                "" if ev.duration is None else f" for {ev.duration} steps")
            rec.affected = [a.id for a in queue]

        elif ev.kind == "breakdown":
            a = sim.agents[ev.agent]
            pos = a.at(t)
            if ev.duration is None:
                broken = a
                a.status = "broken"
                a.path = a.prefix(t)
                sim.grid.block(pos, t, INF)
                aisle = set(sim.grid.aisle_cells)
                for g in a.goals[a.gi:]:
                    if g.kind == "pickup":
                        orphans.append(g.task)
                    elif g.kind == "delivery" and g.task.status == "carried":
                        # the load on board can be recovered by a robot that
                        # pulls up next to the broken one
                        beside = [c for c in sim.grid.neighbors(pos)
                                  if c in aisle and not sim.grid.perm_blocked(c)]
                        if beside:
                            g.task.pickup = min(
                                beside, key=lambda c: sim.grid.dist(c, g.task.delivery))
                            g.task.status = "pending"
                            g.task.recovered = True
                            orphans.append(g.task)
                        else:
                            g.task.status = "lost"
                a.goals = a.goals[:a.gi]
                queue = self._hit_by(pos, t, INF, skip=a)
                rec.detail = f"A{a.id} broke down at {pos}"
            else:
                a.frozen_until = t + ev.duration
                a.path = a.prefix(t) + [pos] * ev.duration
                queue = self._hit_by(pos, t, t + ev.duration + 1, skip=a) + [a]
                rec.detail = f"A{a.id} stalled at {pos} for {ev.duration} steps"
            rec.affected = [a.id] + [b.id for b in queue if b is not a]

        elif ev.kind == "emergency":
            from .agent import Task
            vip = sim.agents[ev.agent]
            task = Task(len(sim.tasks), ev.pickup, ev.delivery, vip.id,
                        emergency=True, t_created=t)
            sim.tasks.append(task)
            vip.insert_task(task, vip.gi)
            sim._progress()
            queue = [vip]
            rec.detail = (f"A{vip.id} gets emergency task "
                          f"{ev.pickup} -> {ev.delivery}")
            rec.affected = [vip.id]

        sim.say(f"DISRUPTION: {rec.detail}; directly affected: "
                f"{['A%d' % i for i in rec.affected]}")

        if sim.mode == "replan":
            self._assign_orphans(broken, orphans, t, rec)
            if not self._global_replan(t, vip):
                for a in queue:
                    self.repair_agent(a, t, rec, priority=a is vip)
        else:
            for a in queue:
                self.repair_agent(a, t, rec, priority=a is vip)
            for w in self._assign_orphans(broken, orphans, t, rec):
                self._replan_suffix(w, t, rec)
        self._settle(t, rec)

        for a in sim.agents:
            old, old_len = before[a.id]
            if future(a.path, t) != old:
                rec.changed.append(a.id)
                sim.last_change[a.id] = t
                if a.status != "broken":
                    rec.added_steps += len(a.path) - old_len
        rec.cpu_ms = (time.perf_counter() - start) * 1000
        sim.say(f"repair done: {len(rec.changed)} plan(s) changed "
                f"{['A%d' % i for i in rec.changed]}, "
                f"{rec.messages} messages, {rec.cpu_ms:.1f} ms")
        return rec

    def _hit_by(self, cell, t, t_end, skip=None):
        """Agents whose remaining path uses `cell` during (t, t_end)."""
        hits = []
        for a in self.sim.agents:
            if a is skip or a.status == "broken":
                continue
            p = a.path
            first = next((tau for tau in range(t + 1, len(p))
                          if tau < t_end and p[tau] == cell), None)
            if first is None and p[-1] == cell and len(p) - 1 < t_end:
                first = max(t + 1, len(p))
            if first is not None:
                hits.append((first, a.id, a))
        return [a for _, _, a in sorted(hits)]

    # ================================================= single-agent repair
    def repair_agent(self, a, t, rec, priority=False):
        """Repair agent `a`'s plan, escalating self-repair -> negotiation ->
        safe stop. Returns False if the agent had to stall."""
        sim = self.sim
        rec.touched.add(a.id)
        old_finish = len(a.path) - 1
        solo = sim.plan_agent(a, t, sim.reservations(t, {a.id}))
        solo_delay = INF if solo is None else len(solo) - 1 - old_finish
        if solo is not None and (not self.negotiation or
                                 (not priority and
                                  solo_delay <= self.delay_tolerance)):
            return self._commit(a, solo, {}, rec, "self-repair")

        deal = self._negotiate(a, t, rec, priority) if self.negotiation else None
        if deal is not None:
            path, yields, extra = deal
            if not yields:
                take = solo is None or len(path) < len(solo)
            elif priority:
                # an emergency outranks the neighbours' convenience
                take = solo is None or \
                    self._urgent_time(a, path, t) < self._urgent_time(a, solo, t)
            else:
                total = len(path) - 1 - old_finish + extra
                take = solo_delay - total >= self.gain_per_agent * len(yields)
            if take:
                rec.messages += len(yields)  # COMMIT
                return self._commit(a, path, yields, rec, "negotiation", t)
            rec.messages += len(yields)      # ABORT
        if solo is not None:
            return self._commit(a, solo, {}, rec, "self-repair")
        self._stall(a, t, rec)
        return False

    def _urgent_time(self, a, path, t):
        """Completion time of the agent's next two goals (the emergency task)."""
        times = goal_times(path, t, a.remaining_cells()[:2])
        return times[-1] if times and times[-1] is not None else INF

    def _commit(self, a, path, yields, rec, level, t=None):
        sim = self.sim
        sim.set_path(a, path)
        rec.levels[a.id] = level
        if yields:
            for bid, pb in yields.items():
                b = sim.agents[bid]
                if bid in self._aside:
                    # b only moved out of the way; it resumes its tasks later
                    b.path = pb
                    self._hold(b, t, rec)
                    rec.levels[bid] = "step-aside"
                else:
                    sim.set_path(b, pb)
                    rec.levels[bid] = "yield"
                rec.touched.add(bid)
            rec.yielders += len(yields)
            sim.say(f"A{a.id}: negotiated new route; "
                    f"{', '.join('A%d' % b for b in yields)} give way")
        else:
            sim.say(f"A{a.id}: {level}, no other agent changed")
        return True

    def _hold(self, a, t, rec):
        a.status, a.stalled_since = "stalled", t
        a.retry_gap, a.retry_at = RETRY_INTERVAL, t + RETRY_INTERVAL
        a.stall_rec = rec

    def _stall(self, a, t, rec):
        a.path = a.prefix(t)
        self._hold(a, t, rec)
        rec.stalled.append(a.id)
        rec.levels[a.id] = "safe-stop"
        self.sim.say(f"A{a.id}: no feasible route, holding position")

    def retry(self, a, t):
        """A holding agent tries again: alone first, then by asking the agents
        in its way to yield or step aside. Plans changed here are charged to
        the disruption that made the agent stop."""
        sim = self.sim
        rec = a.stall_rec or EventRecord(t=t, kind="retry")
        path = sim.plan_agent(a, t, sim.reservations(t, {a.id}))
        yields = {}
        if path is None and self.negotiation:
            deal = self._negotiate(a, t, rec, False)
            if deal is not None:
                path, yields, _ = deal
        if path is None:
            a.retry_gap = min(2 * a.retry_gap, MAX_RETRY_INTERVAL)
            a.retry_at = t + a.retry_gap
            return False
        rec.messages += len(yields)
        self._commit(a, path, yields, rec, "resume", t)
        for bid in yields:
            if bid not in rec.changed:
                rec.changed.append(bid)
            sim.last_change[bid] = t
        return True

    # ======================================================== negotiation
    def _negotiate(self, a, t, rec, priority):
        """Returns (path of a, {neighbour: its new path}, neighbours' total
        extra delay) or None. Tries radio neighbours first, then everybody."""
        sim = self.sim
        penalty = self.priority_penalty if priority else self.conflict_penalty
        pos = a.at(t)
        movable = [b for b in sim.agents
                   if b is not a and b.status in ("active", "stalled")
                   and b.frozen_until <= t]
        tried = None
        for radius in (self.comm_radius, INF):
            group = {b.id for b in movable
                     if abs(b.at(t)[0] - pos[0]) + abs(b.at(t)[1] - pos[1])
                     <= radius}
            if not group or group == tried:
                continue
            tried = set(group)
            deal = self._negotiate_with(a, t, rec, group, penalty)
            if deal is not None:
                return deal
        return None

    def _negotiate_with(self, a, t, rec, soft_ids, penalty):
        sim = self.sim
        soft_ids = set(soft_ids)
        t_plan = max(t, a.frozen_until)
        while True:
            hard = sim.reservations(t, exclude=soft_ids | {a.id})
            soft = sim.reservations(t, only=soft_ids)
            proposal = sim.plan_agent(a, t, hard, soft, penalty)
            if proposal is None:
                return None
            asked = path_conflicts(proposal[t_plan:], t_plan, soft)
            if not asked:
                self._aside = set()
                return proposal, {}, 0

            # tentatively adopt the proposal and ask each neighbour to yield
            saved = {a.id: a.path}
            a.path = proposal
            yields, extra, rejected, aside = {}, 0, None, set()
            for bid in sorted(asked):
                b = sim.agents[bid]
                rec.messages += 2  # REQUEST + ACCEPT/REJECT
                others = sim.reservations(t, {bid})
                pb = sim.plan_agent(b, t, others) \
                    if b.status == "active" else None
                if pb is None:
                    # cannot fit its own tasks around the proposal: offer to
                    # just step aside to the nearest cell where it is in
                    # nobody's way, and resume later
                    tail = space_time_astar(sim.grid, bid, b.at(t), t, [],
                                            others, max_t=t + ASIDE_HORIZON)
                    if tail is None:
                        rejected = bid
                        break
                    pb = b.prefix(t)[:t] + tail
                    aside.add(bid)
                saved[bid] = b.path
                extra += len(pb) - len(b.path)
                b.path = pb
                yields[bid] = pb
            for aid, path in saved.items():
                sim.agents[aid].path = path
            if rejected is None:
                self._aside = aside
                return proposal, yields, extra
            sim.say(f"A{a.id}: A{rejected} cannot give way, re-proposing")
            soft_ids.discard(rejected)

    # ============================================ task re-allocation
    def _assign_orphans(self, broken, orphans, t, rec):
        """Contract-net auction of a broken agent's tasks. Returns winners."""
        sim, grid = self.sim, self.sim.grid
        winners = []
        if broken is None:
            return winners
        pos = broken.at(t)
        for task in orphans:
            alive = [b for b in sim.agents if b.status != "broken"]
            near = [b for b in alive
                    if abs(b.at(t)[0] - pos[0]) + abs(b.at(t)[1] - pos[1])
                    <= self.comm_radius]
            bids = []
            for b in near or alive:
                rec.messages += 2  # ANNOUNCE + BID
                work = [g.cell for g in b.goals[b.gi:] if g.kind != "home"]
                last = work[-1] if work else b.at(t)
                busy = max(sim.planned_finish(b), t) - t
                cost = busy + grid.dist(last, task.pickup) \
                    + grid.dist(task.pickup, task.delivery)
                if b.id not in rec.touched and all(b is not w for w in winners):
                    cost += self.change_penalty
                if cost < INF:
                    bids.append((cost, b.id, b))
            if not bids:
                task.status = "lost"
                sim.say(f"task {task.id}: no bidder, lost")
                continue
            cost, _, w = min(bids)
            rec.messages += 1      # AWARD
            w.insert_task(task, len(w.goals) - 1 if w.goals and
                          w.goals[-1].kind == "home" else len(w.goals))
            if all(w is not x for x in winners):
                winners.append(w)
            sim._progress()   # the winner may already stand on the pickup cell
            sim.say(f"task {task.id} of A{broken.id} auctioned to A{w.id} "
                    f"(bid {cost:.0f}, {len(bids)} bidder(s))")
        return winners

    def _replan_suffix(self, w, t, rec):
        """An auction winner keeps its plan up to its last original delivery
        and only re-plans the tail that now serves the new task(s)."""
        sim = self.sim
        rec.touched.add(w.id)
        old = [g.cell for g in w.goals[w.gi:] if g.kind != "home"]
        new = [g for g in w.goals[w.gi:] if g.task is not None
               and g.task.status == "pending" and g.task.t_created <= t
               and g.task.owner == w.id]
        cells = w.remaining_cells()
        times = goal_times(w.path, min(t, len(w.path) - 1), cells)
        done = [i for i, tm in enumerate(times[:len(old)]) if tm is not None]
        split = None
        for i in range(len(cells)):
            if times[i] is None:
                split = i
                break
        if w.status == "active" and split and new and done:
            t_split = times[split - 1]
            if t_split > max(t, w.frozen_until):
                hard = sim.reservations(t, {w.id})
                tail = space_time_astar(sim.grid, w.id, w.path[t_split],
                                        t_split, cells[split:], hard)
                if tail is not None:
                    w.path = w.path[:t_split] + tail
                    rec.levels[w.id] = "suffix"
                    sim.say(f"A{w.id}: appended new task after its own work")
                    return
        self.repair_agent(w, t, rec)

    # ============================================= baseline: full replan
    def _global_replan(self, t, first=None):
        """Baseline for comparison: throw all plans away and re-run prioritized
        planning for every operational agent from the current positions."""
        sim = self.sim
        team = [a for a in sim.agents if a.status != "broken"]
        fixed = [a for a in sim.agents if a.status == "broken"]
        order = sorted(team, key=lambda a: (a.frozen_until <= t, a is not first,
                                            a.id))
        saved = {a.id: a.path for a in team}
        for _ in range(8):
            hard = Reservations()
            for a in fixed:
                hard.add(a.id, a.path, t)
            failed = None
            for a in order:
                a.path = a.prefix(t) + [a.at(t)] * max(0, a.frozen_until - t)
                path = sim.plan_agent(a, t, hard)
                if path is None:
                    failed = a
                    break
                a.path = path
                hard.add(a.id, path, t)
            if failed is None:
                for a in team:
                    sim.set_path(a, a.path)
                sim.say("global replan: all agents re-planned from scratch")
                return True
            for a in team:
                a.path = saved[a.id]
            order.remove(failed)
            order.insert(0, failed)
        sim.say("global replan failed, falling back to local repair")
        return False

    # ================================================== consistency net
    def _settle(self, t, rec):
        """Safety net: make sure the joint plan is conflict-free after the
        repair, repairing (or as a last resort stopping) whoever still
        conflicts. Prefers to move agents that were already altered."""
        sim = self.sim
        attempts = {}
        for _ in range(6 * len(sim.agents) + 10):
            a = sim.blocked_agent(t)
            if a is None:
                pair = sim.find_conflicts(t)
                if pair is None:
                    return
                movable = [x for x in pair
                           if x.status == "active" and x.frozen_until <= t]
                if not movable:
                    movable = [x for x in pair if x.status == "active"]
                if not movable:
                    return
                movable.sort(key=lambda x: (x.id not in rec.touched, x.id))
                a = movable[0]
            attempts[a.id] = attempts.get(a.id, 0) + 1
            if a.status == "stalled":
                return
            if attempts[a.id] > 2:
                self._stall(a, t, rec)
            else:
                self.repair_agent(a, t, rec)
