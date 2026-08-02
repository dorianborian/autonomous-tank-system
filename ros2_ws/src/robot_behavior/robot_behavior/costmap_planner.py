"""Obstacle-aware A* path search over a Nav2 local costmap.

Added 2026-07-14 to fix a real gap: intent_to_goal previously handed
controller_server a NAIVE STRAIGHT LINE from robot to goal as "the path to
follow." DWB (controller_server's local planner) is a trajectory SCORER/
tracker, not a path-finder — its PathAlign/PathDist/GoalDist critics
penalize departing from the given reference path, so when that reference
line ran through a real obstacle, DWB had no way to invent a wide detour
around it and just stalled at the obstacle edge. This module does the actual
routing: a grid search over the local OccupancyGrid that treats lethal cells
as impassable and inflated cells as costly-but-passable (so the result
naturally prefers a wider berth, not a wall-hugging clip), producing a real
curved path for controller_server to track.

This is intentionally a LOCAL, small-grid search (the local costmap is an
80x80 rolling window, ~6400 cells) — cheap enough to re-run every tick
(~4Hz) so the route keeps adapting as the world/costmap updates, without
needing a full global planner or a persistent map.
"""

import heapq
import math

# Cells at or above this cost are treated as impassable (matches "lethal" in
# the costmap color scheme — see costmap_to_image_node).
LETHAL_THRESHOLD = 99

# Extra edge cost for stepping into an inflated (nonzero, non-lethal) cell,
# scaled by cost/98. This is what makes the search prefer a wider gap around
# an obstacle rather than clipping as close as geometrically possible —
# TUNABLE: raise for more clearance-seeking behavior, lower to let it cut
# closer to obstacles when gaps are tight.
INFLATION_PENALTY_WEIGHT = 6.0

# If the goal cell itself is blocked/unknown/off-grid (e.g. the joystick
# projected it past an obstacle, or right at its edge), search outward in
# rings up to this many cells for the nearest usable substitute target,
# rather than failing outright.
GOAL_SEARCH_RADIUS_CELLS = 12

MAX_EXPANSIONS = 20000  # safety bound so a pathological grid can't hang a tick


def _cell_of(x, y, ox, oy, res):
    return int((x - ox) / res), int((y - oy) / res)


def _world_of(gx, gy, ox, oy, res):
    return ox + (gx + 0.5) * res, oy + (gy + 0.5) * res


def _is_blocked(data, w, h, gx, gy):
    if not (0 <= gx < w and 0 <= gy < h):
        return True
    v = data[gy * w + gx]
    return v < 0 or v >= LETHAL_THRESHOLD  # unknown or lethal


def _step_cost(data, w, h, gx, gy, base):
    v = data[gy * w + gx]
    penalty = (max(v, 0) / 98.0) * INFLATION_PENALTY_WEIGHT if v > 0 else 0.0
    return base + penalty


def _find_nearest_free_cell(data, w, h, gx0, gy0, max_radius):
    """Ring search outward from (gx0, gy0) for the nearest non-blocked cell."""
    if not _is_blocked(data, w, h, gx0, gy0):
        return gx0, gy0
    for r in range(1, max_radius + 1):
        for dx in range(-r, r + 1):
            for dy in (-r, r):
                gx, gy = gx0 + dx, gy0 + dy
                if not _is_blocked(data, w, h, gx, gy):
                    return gx, gy
        for dy in range(-r + 1, r):
            for dx in (-r, r):
                gx, gy = gx0 + dx, gy0 + dy
                if not _is_blocked(data, w, h, gx, gy):
                    return gx, gy
    return None


_NEIGHBORS = [
    (1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
    (1, 1, math.sqrt(2)), (1, -1, math.sqrt(2)),
    (-1, 1, math.sqrt(2)), (-1, -1, math.sqrt(2)),
]


def astar_path(costmap_msg, start_xy, goal_xy):
    """A* search from start_xy to goal_xy (both (x, y) in the costmap's own
    frame, i.e. odom) over the given nav_msgs/OccupancyGrid.

    Returns a list of (x, y) world-frame waypoints from start to goal
    (inclusive), or None if no path could be found at all. If the goal cell
    itself is blocked, silently substitutes the nearest free cell within
    GOAL_SEARCH_RADIUS_CELLS as the actual search target.
    """
    w, h = costmap_msg.info.width, costmap_msg.info.height
    res = costmap_msg.info.resolution
    ox = costmap_msg.info.origin.position.x
    oy = costmap_msg.info.origin.position.y
    data = costmap_msg.data

    sx, sy = _cell_of(start_xy[0], start_xy[1], ox, oy, res)
    gx, gy = _cell_of(goal_xy[0], goal_xy[1], ox, oy, res)

    # The robot is physically AT the start cell regardless of what the
    # costmap says there (sensor noise / self-inflation shouldn't make the
    # search refuse to start) — only the GOAL gets substituted if blocked.
    sx = min(max(sx, 0), w - 1)
    sy = min(max(sy, 0), h - 1)

    goal_cell = _find_nearest_free_cell(data, w, h, gx, gy, GOAL_SEARCH_RADIUS_CELLS)
    if goal_cell is None:
        return None  # goal region entirely blocked, nothing reasonable to plan to
    gx, gy = goal_cell

    if (sx, sy) == (gx, gy):
        return [start_xy, _world_of(gx, gy, ox, oy, res)]

    open_heap = [(0.0, (sx, sy))]
    came_from = {}
    g_score = {(sx, sy): 0.0}
    expansions = 0

    while open_heap and expansions < MAX_EXPANSIONS:
        _, current = heapq.heappop(open_heap)
        expansions += 1
        if current == (gx, gy):
            break
        cx, cy = current
        for dx, dy, base in _NEIGHBORS:
            nx, ny = cx + dx, cy + dy
            if _is_blocked(data, w, h, nx, ny):
                continue
            tentative = g_score[current] + _step_cost(data, w, h, nx, ny, base)
            if tentative < g_score.get((nx, ny), math.inf):
                g_score[(nx, ny)] = tentative
                came_from[(nx, ny)] = current
                f = tentative + math.hypot(gx - nx, gy - ny)
                heapq.heappush(open_heap, (f, (nx, ny)))
    else:
        if (gx, gy) not in came_from and (sx, sy) != (gx, gy):
            return None  # exhausted search without reaching goal

    if (gx, gy) not in came_from and (sx, sy) != (gx, gy):
        return None

    # Reconstruct path goal -> start, then reverse.
    cells = [(gx, gy)]
    cur = (gx, gy)
    while cur != (sx, sy):
        cur = came_from[cur]
        cells.append(cur)
    cells.reverse()

    waypoints = [start_xy]
    for cx, cy in cells[1:]:
        waypoints.append(_world_of(cx, cy, ox, oy, res))
    return waypoints


# =========================================================================
# Added for the target_state_estimator / behavior_state_machine refactor
# (Part B recovery maneuver): small, read-only costmap sampling helpers, on
# top of the same cell/blocked primitives astar_path already uses above.
# =========================================================================

def region_is_clear(costmap_msg, world_points):
    """True if every point in world_points (list of (x, y) in the costmap's
    own frame, i.e. odom) is either off-grid or a non-lethal, non-unknown
    cell. Used before a semi-blind backup maneuver -- the rolling local
    costmap retains recently-observed obstacles even when they've left
    current camera FOV, which is what makes a rear-blind backup checkable at
    all (see behavior_state_machine_node.py module docstring). An off-grid
    point is treated as clear (outside the rolling window entirely) rather
    than unknown-and-blocked, since MAX_ATTEMPT distances are chosen to stay
    within the window in practice; callers relying on this for safety should
    keep their sample distance well inside the costmap's half-width."""
    w, h = costmap_msg.info.width, costmap_msg.info.height
    res = costmap_msg.info.resolution
    ox = costmap_msg.info.origin.position.x
    oy = costmap_msg.info.origin.position.y
    data = costmap_msg.data
    for x, y in world_points:
        gx, gy = _cell_of(x, y, ox, oy, res)
        if not (0 <= gx < w and 0 <= gy < h):
            continue  # off the rolling window -- no information, not blocked
        v = data[gy * w + gx]
        if v < 0 or v >= LETHAL_THRESHOLD:
            return False
    return True


def heading_clearance(costmap_msg, origin_xy, heading_rad, distance_m, num_samples=6):
    """Cost of a straight probe of `distance_m` from origin_xy along
    heading_rad, sampled at num_samples points. Returns the WORST (highest)
    cost cell encountered as an int in [-1, 127] (-1 == unknown, matching the
    OccupancyGrid convention), or -1 if any sample is off-grid/unknown, so
    callers treat "no information" the same as "not provably clear" here
    (this is used to pick a rotate-to heading, not just for the more lenient
    backup-safety check above -- prefer a heading we can actually see)."""
    w, h = costmap_msg.info.width, costmap_msg.info.height
    res = costmap_msg.info.resolution
    ox = costmap_msg.info.origin.position.x
    oy = costmap_msg.info.origin.position.y
    data = costmap_msg.data
    worst = 0
    for i in range(1, num_samples + 1):
        frac = i / num_samples
        x = origin_xy[0] + math.cos(heading_rad) * distance_m * frac
        y = origin_xy[1] + math.sin(heading_rad) * distance_m * frac
        gx, gy = _cell_of(x, y, ox, oy, res)
        if not (0 <= gx < w and 0 <= gy < h):
            return -1
        v = data[gy * w + gx]
        if v < 0:
            return -1
        worst = max(worst, v)
    return worst


def best_clear_heading(costmap_msg, origin_xy, distance_m, num_headings=12):
    """Sample num_headings evenly-spaced headings around a full circle and
    return the one with the lowest heading_clearance() cost (ties broken by
    first-found), or None if every heading came back unknown/off-grid.
    Used by RECOVERING to pick where to rotate to after backing up."""
    best_heading = None
    best_cost = None
    for i in range(num_headings):
        heading = -math.pi + (2.0 * math.pi * i / num_headings)
        cost = heading_clearance(costmap_msg, origin_xy, heading, distance_m)
        if cost < 0:
            continue
        if best_cost is None or cost < best_cost:
            best_cost = cost
            best_heading = heading
    return best_heading
