"""Optional Pygame display for the tabular experiment. It never chooses actions."""

from time import monotonic

import numpy as np


class TrainingView:
    def __init__(self, env, episodes):
        try:
            import pygame as pg
        except ImportError as exc:
            raise ImportError('Install Pygame with: sudo apt install python3-pygame') from exc
        self.pg, self.env, self.episodes = pg, env, episodes
        try:
            pg.display.init()
            pg.font.init()
            self.screen = pg.display.set_mode((850, 590))
        except pg.error as exc:
            pg.display.quit()
            pg.font.quit()
            raise RuntimeError(f'Cannot open the Pygame window: {exc}. Run without --live.') from exc
        pg.display.set_caption('ARC Lab 6 | Watch Q-learning')
        self.font = pg.font.Font(None, 23)
        self.small = pg.font.Font(None, 17)
        self.heading = pg.font.Font(None, 32)
        self.clock = pg.time.Clock()
        self.active, self.paused, self.slow = True, False, None
        self.updates = 0
        self.last_draw = -1.0

    def close(self):
        if self.active:
            self.pg.display.quit()
            self.pg.font.quit()
            self.active = False

    def __call__(self, q, step):
        if not self.active:
            return
        self.updates += 1
        slow = self.slow if self.slow is not None else self.updates <= 24
        if not slow and not self.paused and monotonic() - self.last_draw < 1 / 30:
            return
        while self.active:
            for event in self.pg.event.get():
                if event.type == self.pg.QUIT or (
                    event.type == self.pg.KEYDOWN and event.key == self.pg.K_ESCAPE
                ):
                    self.close()
                    return
                if event.type == self.pg.KEYDOWN:
                    if event.key == self.pg.K_SPACE:
                        self.paused = not self.paused
                    if event.key == self.pg.K_f:
                        slow = self.slow = not slow
            self.draw(q, step, slow)
            self.last_draw = monotonic()
            if not self.paused:
                if slow:
                    self.clock.tick(8)
                return
            self.clock.tick(30)

    def draw(self, q, step, slow):
        pg, screen = self.pg, self.screen
        ink, muted = '#152d40', '#486477'
        self.screen.fill('#f1f6f8')

        def label(message, pos, colour=ink, font=None, center=False):
            surface = (font or self.font).render(str(message), True, colour)
            screen.blit(surface, surface.get_rect(center=pos) if center else pos)

        label('Watch the Q-table learn', (22, 17), font=self.heading)
        label('Each cell contains four Q-values: north above, east to the right.', (23, 53), muted)
        for row in range(6):
            for col in range(6):
                cell = (row, col)
                state = self.env.state(cell)
                rect = pg.Rect(22 + col * 79, 85 + (5 - row) * 79, 77, 77)
                wall = self.env.grid[cell]
                colour = '#596c7b' if wall else '#ffffff'
                if cell == self.env.goal:
                    colour = '#dcf4e4'
                pg.draw.rect(screen, colour, rect, border_radius=5)
                if state == step.state:
                    pg.draw.rect(screen, '#af3c07', rect, 2, border_radius=5)
                if wall:
                    label('WALL', rect.center, '#ffffff', self.small, True)
                    continue
                if cell == self.env.goal:
                    label('GOAL', (rect.centerx, rect.y + 16), font=self.small, center=True)
                else:
                    best = int(np.argmax(q[state]))
                    for action, (dx, dy) in enumerate(((21, 0), (0, -25), (-21, 0), (0, 25))):
                        x, y = rect.centerx + dx, rect.centery + dy
                        updated = state == step.state and action == step.action
                        if updated:
                            pg.draw.rect(screen, '#ffebca', (x - 19, y - 8, 38, 16), border_radius=3)
                        label(f'{q[state, action]:.2f}', (x, y),
                              '#9d3303' if updated else '#006c54' if action == best else muted,
                              self.small, True)
                if state == step.next_state:
                    pg.draw.circle(screen, '#0b609b', rect.center, 6)

        names = ('east', 'north', 'west', 'south')
        x = 522
        label('One transition', (x, 85), font=self.heading)
        lines = [
            f'Episode {step.episode:,} / {self.episodes:,}   |   Step {step.step}',
            f'Epsilon: {step.epsilon:.3f}',
            f'From: {self.env.cell(step.state)}',
            f'Chosen action: {names[step.action]}',
            f'Result: {self.env.cell(step.next_state)}',
            f'Reward: {step.reward:+.2f}' + ('  (goal)' if step.terminated else ''),
            f'Target: {step.target:.3f}',
            f'Old Q: {step.old_value:.3f}',
            f'New Q: {q[step.state, step.action]:.3f}',
        ]
        for i, line in enumerate(lines):
            label(line, (x, 130 + i * 27))
        label('Blue dot: robot after the action', (x, 399), '#0b609b')
        label('Orange: Q-entry just updated', (x, 426), '#9d3303')
        label('Green: best current action', (x, 453), '#006c54')
        mode = 'Paused' if self.paused else 'Slow' if slow else 'Fast'
        label(f'{mode}  |  Space: pause/resume  |  F: slow/fast  |  Esc: close view, keep training',
              (23, 565), muted)
        pg.display.flip()
