"""Known-free grid routes. No truth-map access, ROS or velocity publisher."""

from collections import deque
import math

# CAD footprint includes the outer wheel edges. Lidar sits behind base_link.
BODY_RADIUS_M = math.hypot(0.0725, 0.081)
LIDAR_X_M = -0.0227
TURN_RANGE_M = BODY_RADIUS_M + abs(LIDAR_X_M) + 0.05


class GridRouter:
    """Inflate observed obstacles, flood known free space, forbid corner cuts."""

    def __init__(self, grid, pose, clearance=0.38):
        self.grid = grid
        self.start = grid.world_to_cell(*pose[:2])
        self.free = {(x, y) for y in range(grid.height) for x in range(grid.width)
                     if 0 <= grid.cell(x, y) <= 25}
        radius = math.ceil(clearance / grid.resolution)
        offsets = [(dx, dy) for dy in range(-radius, radius + 1)
                   for dx in range(-radius, radius + 1)
                   if math.hypot(dx, dy) * grid.resolution <= clearance]
        # Unknown cells are never driven into, but are not invented obstacles.
        for y in range(grid.height):
            for x in range(grid.width):
                if grid.cell(x, y) > 25:
                    self.free.difference_update((x + dx, y + dy) for dx, dy in offsets)
        self.parents = {self.start: None}
        self.depth = {self.start: 0}
        queue = deque([self.start])
        while queue:
            x, y = queue.popleft()
            for dx, dy in ((1, 0), (0, 1), (-1, 0), (0, -1)):
                cell = (x + dx, y + dy)
                if cell not in self.free or cell in self.parents:
                    continue
                self.parents[cell] = (x, y)
                self.depth[cell] = self.depth[(x, y)] + 1
                queue.append(cell)

    def path(self, cell):
        if cell not in self.parents:
            return []
        cells = []
        while cell is not None:
            cells.append(cell)
            cell = self.parents[cell]
        return [self.grid.cell_to_world(*item) for item in reversed(cells)]

    def home(self, target):
        cell = self.grid.world_to_cell(*target)
        if cell in self.parents:
            return self.path(cell)
        # A start close to an obstacle may be inside the planning margin. Return
        # to a safe reachable cell inside the existing home acceptance disk.
        candidates = [(math.dist(self.grid.cell_to_world(*item), target), item)
                      for item in self.parents if item in self.free]
        if candidates:
            distance, cell = min(candidates)
            if distance <= 0.20:
                return self.path(cell)
        return []

    def frontier(self, pose, excluded=()):
        # Approach from known free space. Frontier *unknown* cells are never goals.
        candidates = []
        g = self.grid
        for cell in self.parents:
            if cell not in self.free or cell in excluded or self.depth[cell] < 6:
                continue
            x, y = cell
            gain = sum(g.cell(x + dx, y + dy) == -1
                       for dx, dy in ((1, 0), (0, 1), (-1, 0), (0, -1)))
            if gain:
                wx, wy = g.cell_to_world(x, y)
                # Prefer useful views away from the current acceptance disk.
                if math.hypot(wx - pose[0], wy - pose[1]) > 0.28:
                    candidates.append((self.depth[cell] / gain, cell))
        return self.path(min(candidates)[1]) if candidates else []


def waypoint(path, pose, lookahead=0.18):
    """Small lookahead along a grid route; do not shortcut obstacle corners."""
    if not path:
        return None
    nearest = min(range(len(path)), key=lambda i: math.hypot(path[i][0] - pose[0], path[i][1] - pose[1]))
    distance = 0.0
    index = nearest
    while index + 1 < len(path):
        distance += math.dist(path[index], path[index + 1])
        if distance > lookahead:
            break
        index += 1
    return path[index]
