"""Cooperative wall budget: never turn unfinished validation into permission to move."""
import math
import time


class PlanningBudgetExceeded(RuntimeError):
    pass


class PlanningBudget:
    def __init__(self, seconds, *, clock=time.monotonic_ns):
        if type(seconds) not in (int, float) or not math.isfinite(seconds) or not 0.01 <= seconds <= 60:
            raise ValueError("Computation budget must be 0.01..60 wall seconds")
        self.limit = float(seconds)
        self.clock = clock
        self.started = clock()
        self.finished = None

    def elapsed(self):
        return ((self.clock() if self.finished is None else self.finished) - self.started) / 1e9

    def check(self):
        if self.elapsed() >= self.limit:
            raise PlanningBudgetExceeded("Planning/validation wall budget exhausted; no dispatch is allowed")

    def remaining(self):
        elapsed = self.elapsed()
        if elapsed >= self.limit:
            raise PlanningBudgetExceeded("Planning/validation wall budget exhausted; no dispatch is allowed")
        return self.limit - elapsed

    def finish(self):
        if self.finished is None:
            self.finished = self.clock()
        elapsed = self.elapsed()
        return dict(limit_wall_sec=self.limit, elapsed_wall_sec=elapsed,
                    exceeded=elapsed >= self.limit,
                    enforcement="Cooperative checks between bounded native samples and ROS calls; calls are not preempted")
