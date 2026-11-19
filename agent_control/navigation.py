"""Known-free grid routes. No truth-map access, ROS or velocity publisher."""

from collections import deque
import math

from .protocol import ActionProposal

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
    nearest = nearest_on_route(path, pose)
    distance = 0.0
    index = nearest
    while index + 1 < len(path):
        distance += math.dist(path[index], path[index + 1])
        if distance > lookahead:
            break
        index += 1
    return path[index]


def nearest_on_route(path, pose):
    return min(range(len(path)), key=lambda i: math.dist(path[i], pose[:2]))


def segment_is_free(grid, free, start, end):
    """Check every cell touched by a straight segment, including both sides of a corner."""
    x, y = grid.world_to_cell(*start)
    end_x, end_y = grid.world_to_cell(*end)
    dx, dy = end[0] - start[0], end[1] - start[1]
    sx, sy = (1 if dx > 0 else -1), (1 if dy > 0 else -1)
    res = grid.resolution
    tx = (grid.origin_x + (x + (sx > 0)) * res - start[0]) / dx if dx else math.inf
    ty = (grid.origin_y + (y + (sy > 0)) * res - start[1]) / dy if dy else math.inf
    step_x = res / abs(dx) if dx else math.inf
    step_y = res / abs(dy) if dy else math.inf
    while (x, y) in free:
        if (x, y) == (end_x, end_y):
            return True
        # Do not step past the destination along either axis, e.g. when the
        # destination lies exactly on a grid line and the segment approaches it.
        next_x = tx if x != end_x else math.inf
        next_y = ty if y != end_y else math.inf
        if abs(next_x - next_y) <= 1e-9:
            if (x + sx, y) not in free or (x, y + sy) not in free:
                return False
            x, y = x + sx, y + sy
            tx, ty = tx + step_x, ty + step_y
        elif next_x < next_y:
            x, tx = x + sx, tx + step_x
        else:
            y, ty = y + sy, ty + step_y
    return False


class PointNavigator:
    """Follow a fixed destination through observed free space; no motion output."""

    tolerance_m = 0.06
    corner_tolerance_m = 0.025

    def __init__(self, target, *, max_linear, max_angular, clearance):
        self.target = target
        self.max_linear = max_linear
        self.max_angular = max_angular
        self.clearance = max(0.38, clearance)
        self._grid = None
        self._free = set()
        self._route = []
        self._checked_at = -math.inf
        self._aligning = True
        self._facts = {"phase": "PLANNING", "navigation": {"goal": list(target)}}

    def start(self, **_world):
        return self.snapshot()

    def rebase(self, *, now):
        self._grid = None
        self._aligning = True

    def stop(self, *, reason):
        self._facts = {**self._facts, "phase": "STOPPED", "reason": reason}

    def snapshot(self):
        return self._facts

    def _plan(self, router, start, cell):
        route = router.path(cell)
        return smooth_route(router, [start, *route, self.target]) if route else []

    def step(self, *, now, pose, ranges, grid=None, **_scan):
        position = pose[:2] if pose is not None else None
        distance = math.dist(position, self.target) if position is not None else None
        if position is None or grid is None:
            return self._result("BLOCKED", "map_or_pose_missing", distance)
        if sum(math.isfinite(value) and value > 0 for value in ranges) < 12:
            return self._result("BLOCKED", "scan_invalid", distance)
        cell = grid.world_to_cell(*self.target)
        if not (0 <= cell[0] < grid.width and 0 <= cell[1] < grid.height):
            return self._result("BLOCKED", "target_outside_map", distance)
        if grid.cell(*cell) < 0:
            return self._result("BLOCKED", "target_unknown", distance)
        if grid.cell(*cell) > 25:
            return self._result("BLOCKED", "target_occupied", distance)

        while len(self._route) > 1 and math.dist(position, self._route[0]) <= self.corner_tolerance_m:
            self._route.pop(0)
            self._aligning = True
        # Keep the selected route when new rays update the map. Replan only
        # if the remaining path has lost clearance or the robot has left it.
        path_invalid = bool(self._route) and not segment_is_free(grid, self._free, position, self._route[0])
        if self._grid is None or (grid is not self._grid and (path_invalid or now - self._checked_at >= 1.0)):
            router = GridRouter(grid, pose, self.clearance)
            self._grid, self._free, self._checked_at = grid, router.free, now
            if cell not in self._free:
                return self._result("BLOCKED", "target_clearance", distance)
            points = [position, *self._route]
            if not self._route or any(
                not segment_is_free(grid, self._free, left, right)
                for left, right in zip(points, points[1:])
            ):
                self._route = self._plan(router, position, cell)
                self._aligning = True
        if not self._route:
            return self._result("BLOCKED", "target_unreachable", distance)
        if distance <= self.tolerance_m:
            return self._result("DONE", "destination_reached", distance)
        point = self._route[0]
        if not segment_is_free(grid, self._free, position, point):
            return self._result("BLOCKED", "route_clearance", distance)

        error = (math.atan2(point[1] - position[1], point[0] - position[0]) - pose[2] + math.pi) % (2 * math.pi) - math.pi
        self._aligning = abs(error) > (0.06 if self._aligning else 0.20)
        angular = max(-self.max_angular, min(self.max_angular, 1.5 * error)) if abs(error) > 0.025 else 0.0
        if self._aligning:
            return self._result("ALIGN", "align_to_route", distance, angular=angular, error=error)
        linear = min(self.max_linear, math.dist(position, point) * 0.7)
        return self._result("DRIVE", "follow_route", distance, linear, angular, error)

    def _result(self, phase, reason, distance, linear=0.0, angular=0.0, error=None):
        point = self._route[0] if self._route else None
        self._facts = {
            "phase": phase,
            "reason": reason,
            "navigation": {
                "goal": list(self.target),
                "target": point,
                "route": self._route.copy(),
                "distance_m": distance,
                "tolerance_m": self.tolerance_m,
                "bearing_error_rad": error,
                "explorer_state": phase,
                "explorer_reason": reason,
            },
        }
        return ActionProposal(linear, angular, 250, "navigate", reason), self._facts


def smooth_route(router, points):
    """Remove grid turns only when the shortcut stays in inflated free space."""
    result = []
    index = 0
    while index < len(points) - 1:
        for candidate in range(len(points) - 1, index, -1):
            if segment_is_free(router.grid, router.free, points[index], points[candidate]):
                result.append(points[candidate])
                index = candidate
                break
        else:
            return []
    return result
