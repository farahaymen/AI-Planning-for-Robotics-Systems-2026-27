import math


def displacement(start, end):
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    return math.hypot(dx, dy)


class RetryBudget:
    def __init__(self, limit=2):
        if limit < 0:
            raise ValueError('Retry limit cannot be negative')
        self.limit = limit
        self.used = 0

    def decide(self, made_progress):
        if made_progress:
            self.used = 0
            return 'continue'
        if self.used >= self.limit:
            return 'abort'
        self.used += 1
        return 'retry'


def main():
    budget = RetryBudget(limit=2)
    for end in [(0.01, 0), (0.02, 0), (0.03, 0), (0.3, 0)]:
        distance = displacement((0, 0), end)
        print(distance, budget.decide(distance >= 0.15))
