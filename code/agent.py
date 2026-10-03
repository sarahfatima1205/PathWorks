"""Agents, tasks and goals."""

from dataclasses import dataclass, field


@dataclass
class Task:
    id: int
    pickup: tuple
    delivery: tuple
    owner: int
    emergency: bool = False
    recovered: bool = False   # load taken over from a broken-down robot
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
    retry_at: int = 0
    retry_gap: int = 3
    stall_rec: object = None                    # disruption that stopped it
    finish_time: int = 0                        # time of the last delivery
    deliveries: int = 0
    pace: float = 1.0                            # cells per time step; >1 faster

    def time_index(self, t):
        """Convert simulation time to the path index for this robot.

        A robot with pace=2 moves two cells per time step, so its route is
        effectively compressed in time. The base route plan remains the same, but
        the execution timeline is scaled by the robot's speed.
        """
        if self.pace <= 0:
            return len(self.path) - 1
        return min(len(self.path) - 1, max(0, int(round(t * self.pace))))

    def at(self, t):
        return self.path[self.time_index(t)] if self.path else self.home

    def prefix(self, t):
        """Executed part of the path, i.e. cells for times 0..t (new list)."""
        if not self.path:
            return [self.home]
        out = []
        for i in range(t + 1):
            out.append(self.path[self.time_index(i)])
        return out

    def remaining_cells(self):
        return [g.cell for g in self.goals[self.gi:]]

    def done(self, t):
        effective_end = max(0, (len(self.path) - 1) / max(self.pace, 1e-9))
        return self.gi >= len(self.goals) and t >= effective_end

    def insert_task(self, task, index):
        self.goals[index:index] = [Goal(task.pickup, "pickup", task),
                                   Goal(task.delivery, "delivery", task)]
        if self.gi > index:
            self.gi = index
        task.owner = self.id
