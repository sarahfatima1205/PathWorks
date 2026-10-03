"""Space-Time A* with a reservation table (single-agent planner used both for
the initial prioritized plan and for every local repair)."""

import heapq
from itertools import count

from .grid import INF


class Reservations:
    """Space-time occupancy of a set of agents' plans.

    occ[(cell, t)] = agent id, rest[cell] = (t_from, agent id) for the cell an
    agent occupies forever once its path ends.
    """

    __slots__ = ("occ", "rest", "last")

    def __init__(self):
        self.occ = {}
        self.rest = {}
        self.last = {}

    def add(self, aid, path, t0):
        occ, last = self.occ, self.last
        for t in range(t0, len(path)):
            c = path[t]
            occ[(c, t)] = aid
            if last.get(c, -1) < t:
                last[c] = t
        self.rest[path[-1]] = (len(path) - 1, aid)

    def blocker(self, c_from, c_to, t_next):
        """Agent that forbids moving c_from -> c_to arriving at t_next, or None."""
        b = self.occ.get((c_to, t_next))
        if b is not None:
            return b
        r = self.rest.get(c_to)
        if r is not None and t_next >= r[0]:
            return r[1]
        if c_from != c_to:  # edge (swap) conflict
            b = self.occ.get((c_to, t_next - 1))
            if b is not None and self.occ.get((c_from, t_next)) == b:
                return b
        return None

    def free_forever(self, cell, t):
        return cell not in self.rest and self.last.get(cell, -1) <= t


def space_time_astar(grid, agent_id, start, t0, goals, hard, soft=None,
                     penalty=0, max_t=None, max_expansions=150_000):
    """Plan from `start` at time `t0` through the ordered `goals` cells.

    `hard` reservations may never be violated. Moves that collide with `soft`
    reservations are allowed but cost `penalty` extra: this is how a disrupted
    agent finds the path that needs the fewest neighbours to give way.
    Returns the list of cells for times t0, t0+1, ... or None.
    """
    n = len(goals)
    dmaps = [grid.dist_map(g) for g in goals]
    suffix = [0] * (n + 1)
    for i in range(n - 2, -1, -1):
        suffix[i] = suffix[i + 1] + dmaps[i + 1].get(goals[i], INF)

    def h(cell, gi):
        return 0 if gi >= n else dmaps[gi].get(cell, INF) + suffix[gi]

    gi0 = 0
    while gi0 < n and start == goals[gi0]:
        gi0 += 1
    h0 = h(start, gi0)
    if h0 == INF:
        return None
    if max_t is None:
        max_t = t0 + h0 + 3 * (grid.width + grid.height)

    tie = count()
    start_state = (start, t0, gi0)
    open_heap = [(h0, h0, next(tie), 0, start_state)]
    best = {start_state: 0}
    parent = {}
    expansions = 0
    while open_heap:
        _, _, _, g, state = heapq.heappop(open_heap)
        if g > best[state]:
            continue
        cell, t, gi = state
        if gi >= n and hard.free_forever(cell, t):
            path = [cell]
            while state in parent:
                state = parent[state]
                path.append(state[0])
            path.reverse()
            return path
        expansions += 1
        if t >= max_t or expansions > max_expansions:
            continue
        nt = t + 1
        for nc in (cell,) + grid.neighbors(cell):
            if not grid.passable(nc, nt, agent_id):
                continue
            if hard.blocker(cell, nc, nt) is not None:
                continue
            ngi = gi
            while ngi < n and nc == goals[ngi]:
                ngi += 1
            nh = h(nc, ngi)
            if nh == INF:
                continue
            ng = g + 1
            if soft is not None and soft.blocker(cell, nc, nt) is not None:
                ng += penalty
            nstate = (nc, nt, ngi)
            if ng < best.get(nstate, INF):
                best[nstate] = ng
                parent[nstate] = state
                heapq.heappush(open_heap, (ng + nh, nh, next(tie), ng, nstate))
    return None


def path_conflicts(path, t0, res):
    """Ids of the agents in `res` that `path` (starting at time t0) collides with."""
    hit = set()
    for i in range(1, len(path)):
        b = res.blocker(path[i - 1], path[i], t0 + i)
        if b is not None:
            hit.add(b)
    r = res.rest.get(path[-1])
    if r is not None:
        hit.add(r[1])
    end_t = t0 + len(path) - 1
    for (c, t), b in res.occ.items():
        if c == path[-1] and t > end_t:
            hit.add(b)
    return hit


def goal_times(path, t0, cells):
    """Times at which the ordered goal `cells` are reached walking `path`
    from time index t0. Unreached goals get None."""
    times = [None] * len(cells)
    i = 0
    for t in range(t0, len(path)):
        while i < len(cells) and path[t] == cells[i]:
            times[i] = t
            i += 1
        if i == len(cells):
            break
    return times


def future(path, t):
    """Canonical remaining path from time t (trailing waits stripped)."""
    p = path[t:] if t < len(path) else path[-1:]
    n = len(p)
    while n > 1 and p[n - 1] == p[n - 2]:
        n -= 1
    return tuple(p[:n])
