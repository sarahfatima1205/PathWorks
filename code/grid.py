"""Warehouse grid map: static layout, dynamic blockages and distance maps."""

from collections import deque

INF = float("inf")
MOVES = ((1, 0), (-1, 0), (0, 1), (0, -1))


class Grid:
    """4-connected grid. Cells are (x, y) tuples.

    Static obstacles are `walls` (border) and `shelves`. Every agent owns a
    private parking cell (`home_owner`) that no other agent may enter, which
    keeps the instance well-formed: an agent resting at home never blocks
    anybody. Dynamic obstacles live in `blocked` as cell -> (t_start, t_end).
    """

    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.walls = set()
        self.shelves = set()
        self.stations = []      # delivery stations
        self.pick_cells = []    # aisle cells next to a shelf
        self.home_owner = {}    # cell -> agent id
        self.blocked = {}       # cell -> (t_start, t_end), t_end may be INF
        self._nbrs = {}
        self._dist = {}

    # ------------------------------------------------------------ layout
    def finalize(self):
        """Precompute static adjacency once the layout is fixed."""
        static = self.walls | self.shelves
        self.free_cells = [(x, y) for x in range(self.width)
                           for y in range(self.height) if (x, y) not in static]
        free = set(self.free_cells)
        self._nbrs = {
            c: tuple(n for n in ((c[0] + dx, c[1] + dy) for dx, dy in MOVES)
                     if n in free)
            for c in self.free_cells
        }
        special = set(self.home_owner) | set(self.stations)
        self.aisle_cells = [c for c in self.free_cells if c not in special]

    def neighbors(self, cell):
        return self._nbrs[cell]

    # ------------------------------------------------- dynamic obstacles
    def block(self, cell, t_start, t_end=INF):
        self.blocked[cell] = (t_start, t_end)
        if t_end == INF:
            self._dist.clear()  # permanent change: heuristics are stale

    def is_blocked(self, cell, t):
        b = self.blocked.get(cell)
        return b is not None and b[0] <= t < b[1]

    def perm_blocked(self, cell):
        b = self.blocked.get(cell)
        return b is not None and b[1] == INF

    def blocked_at(self, t):
        return [c for c, (s, e) in self.blocked.items() if s <= t < e]

    def passable(self, cell, t, agent_id):
        b = self.blocked.get(cell)
        if b is not None and b[0] <= t < b[1]:
            return False
        return self.home_owner.get(cell, agent_id) == agent_id

    # ------------------------------------------------------- distances
    def dist_map(self, goal):
        """True shortest distance from every cell to `goal` (BFS, cached).

        Permanent blockages are respected; home cells are dead ends that are
        never routed through. Used as the Space-Time A* heuristic.
        """
        d = self._dist.get(goal)
        if d is not None:
            return d
        d = {}
        if not self.perm_blocked(goal):
            d[goal] = 0
            queue = deque([goal])
            while queue:
                c = queue.popleft()
                if c in self.home_owner and c != goal:
                    continue
                nd = d[c] + 1
                for n in self._nbrs[c]:
                    if n not in d and not self.perm_blocked(n):
                        d[n] = nd
                        queue.append(n)
        self._dist[goal] = d
        return d

    def dist(self, a, b):
        return self.dist_map(b).get(a, INF)

    def connected_if_blocked(self, cell):
        """Would every free cell stay reachable if `cell` were blocked forever?

        Parking cells are private dead ends, so they may be reached but never
        travelled through."""
        free = [c for c in self.free_cells
                if c != cell and not self.perm_blocked(c)]
        start = next((c for c in free if c not in self.home_owner), None)
        if start is None:
            return False
        seen = {start}
        queue = deque([start])
        while queue:
            c = queue.popleft()
            if c in self.home_owner:
                continue
            for n in self._nbrs[c]:
                if n != cell and n not in seen and not self.perm_blocked(n):
                    seen.add(n)
                    queue.append(n)
        return len(seen) == len(free)


def make_warehouse(width=32, height=20, n_agents=10, block_w=4, block_h=2):
    """Build a warehouse: shelf blocks separated by one-cell aisles, a two-cell
    ring corridor, parking docks on the left/top/bottom border and delivery
    stations on the right border."""
    g = Grid(width, height)
    for x in range(width):
        g.walls.add((x, 0))
        g.walls.add((x, height - 1))
    for y in range(height):
        g.walls.add((0, y))
        g.walls.add((width - 1, y))

    for by in range(3, height - 2 - block_h, block_h + 1):
        for bx in range(3, width - 2 - block_w, block_w + 1):
            for x in range(bx, bx + block_w):
                for y in range(by, by + block_h):
                    g.shelves.add((x, y))

    g.stations = [(width - 1, y) for y in range(2, height - 2, 2)]
    g.walls.difference_update(g.stations)

    left = [(0, y) for y in range(1, height - 1)]
    top = [(x, 0) for x in range(2, width - 2, 2)]
    bottom = [(x, height - 1) for x in range(2, width - 2, 2)]
    if n_agents <= len(left):
        homes = [left[i * len(left) // n_agents] for i in range(n_agents)]
    else:
        extra = [c for pair in zip(top, bottom) for c in pair]
        homes = (left + extra)[:n_agents]
        if len(homes) < n_agents:
            raise ValueError(f"map too small for {n_agents} agents")
    for i, cell in enumerate(homes):
        g.home_owner[cell] = i
    g.walls.difference_update(homes)

    g.finalize()
    g.pick_cells = sorted(
        c for c in g.aisle_cells
        if any((c[0] + dx, c[1] + dy) in g.shelves for dx, dy in MOVES))
    return g
