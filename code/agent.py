"""Agents, tasks and goals."""

from dataclasses import dataclass, field


@dataclass
class Task:
    id: int
    pickup: tuple
    delivery: tuple
    owner: int
    emergency: bool = False
    status: str = "pending"   # pending -> carried -> delivered | lost
    t_created: int = 0
    t_pickup: int = None
    t_delivery: int = None


@dataclass
class Goal:
    cell: tuple
    kind: str                 # pickup | delivery | home
    task: Task = None


@dataclass
class Agent:
    id: int
    home: tuple
    goals: list = field(default_factory=list)   # ordered; last one is home
    gi: int = 0                                 # index of the next goal
    path: list = field(default_factory=list)    # path[t] = cell at time t
    status: str = "active"                      # active | stalled | broken
    frozen_until: int = 0                       # temporary breakdown
    stalled_since: int = 0
    finish_time: int = 0                        # time of the last delivery
    deliveries: int = 0

    def at(self, t):
        return self.path[t] if t < len(self.path) else self.path[-1]

    def prefix(self, t):
        """Executed part of the path, i.e. cells for times 0..t (new list)."""
        if t < len(self.path):
            return self.path[:t + 1]
        return self.path + [self.path[-1]] * (t + 1 - len(self.path))

    def remaining_cells(self):
        return [g.cell for g in self.goals[self.gi:]]

    def done(self, t):
        return self.gi >= len(self.goals) and t >= len(self.path) - 1

    def insert_task(self, task, index):
        self.goals[index:index] = [Goal(task.pickup, "pickup", task),
                                   Goal(task.delivery, "delivery", task)]
        if self.gi > index:
            self.gi = index
        task.owner = self.id
