from __future__ import annotations

import math
from typing import List, Tuple

from src.models import CarPose, Cone, Path2D

Point = Tuple[float, float]


# ---------------- tiny vector helpers (plain tuples, no libraries) ----------------

def _add(a: Point, b: Point) -> Point:
    return (a[0] + b[0], a[1] + b[1])


def _sub(a: Point, b: Point) -> Point:
    return (a[0] - b[0], a[1] - b[1])


def _mul(a: Point, k: float) -> Point:
    return (a[0] * k, a[1] * k)


def _dot(a: Point, b: Point) -> float:
    return a[0] * b[0] + a[1] * b[1]


def _dist(a: Point, b: Point) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def _unit(a: Point) -> Point:
    d = math.hypot(a[0], a[1])
    return (a[0] / d, a[1] / d) if d > 1e-9 else (0.0, 0.0)


def _left_normal(a: Point) -> Point:
    """a rotated +90 deg: points to the LEFT of the direction."""
    return (-a[1], a[0])


def _right_normal(a: Point) -> Point:
    """a rotated -90 deg: points to the RIGHT of the direction."""
    return (a[1], -a[0])


def _mid(a: Point, b: Point) -> Point:
    return ((a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0)


def _snap(p: Point) -> Point:
    """Snap a point to the integer grid (deterministic half-up rounding)."""
    return (float(math.floor(p[0] + 0.5)), float(math.floor(p[1] + 0.5)))


class PathPlanning:
    """Simple 'virtual track' planner.

    Idea:
    1. Estimate the driving direction from the cones (yaw is only a fallback,
       because in some scenarios the yaw points away from the track).
    2. Build a rough centerline: through the midpoints of blue/yellow cone
       pairs ("gates"), or offset from the boundary if only one side is seen.
       The line is extended/truncated so it covers all cones (5-10 m total).
    3. Walk along that line in 1 m steps and place a LEFT and a RIGHT virtual
       cone on the integer grid at every step -> a complete virtual track.
       The track width is measured from the real gates when both sides are
       visible (clamped to 1-3 m), otherwise a default of 2 m is used.
    4. The path is simply the straight lines joining the midpoints of
       corresponding virtual cone pairs (densified to step <= 0.5 m).

    Assumptions (documented per the README):
    - cone colors are correct; the track is roughly ahead of the car;
    - unpaired extra cones are ignored; a single cone behind the car is ignored;
    - cones farther than ~9 m are not covered (10 m path length cap).
    """

    WIDTH = 2.0        # default virtual track width [m] (one side missing)
    MIN_WIDTH = 1.0    # clamp for measured gate widths
    MAX_WIDTH = 3.0
    CONE_SPACING = 1.0 # distance between virtual cones along the track [m]
    PATH_STEP = 0.5    # max spacing of returned path points [m] (task rule)
    MIN_LEN = 5.0      # task rule: total path length between 5 and 10 m
    MAX_LEN = 10.0
    MARGIN = 1.0       # keep building track this far beyond the farthest cone

    def __init__(self, car_pose: CarPose, cones: List[Cone]):
        self.car_pose = car_pose
        self.cones = list(cones)
        self.virtual_left: List[Point] = []
        self.virtual_right: List[Point] = []

    # ------------------------------------------------------------------ API --
    def generatePath(self) -> Path2D:
        blue = [(c.x, c.y) for c in self.cones if c.color == 1]
        yellow = [(c.x, c.y) for c in self.cones if c.color == 0]

        direction, waypoints, width = self._plan(blue, yellow)
        centerline = self._centerline(waypoints, direction)
        mids = self._virtual_track(centerline, width)
        path = self._densify(mids, self.PATH_STEP)
        return self._fix_length(path, direction)

    # ------------------------------------------------- 1. direction & waypoints --
    def _plan(self, blue: List[Point], yellow: List[Point]):
        """Decide driving direction, the waypoints, and the track width [m]."""
        car = (self.car_pose.x, self.car_pose.y)
        yaw = (math.cos(self.car_pose.yaw), math.sin(self.car_pose.yaw))

        all_pts = blue + yellow
        if not all_pts:                                  # nothing seen: straight ahead
            return yaw, [car], self.WIDTH

        centroid = (sum(p[0] for p in all_pts) / len(all_pts),
                    sum(p[1] for p in all_pts) / len(all_pts))
        to_cones = _unit(_sub(centroid, car))

        def orient(v: Point) -> Point:
            """Return v or -v: the sign pointing toward the cones (yaw as fallback)."""
            for ref in (to_cones, yaw):
                d = _dot(v, ref)
                if abs(d) > 1e-9:
                    return v if d > 0 else _mul(v, -1.0)
            return v

        if blue and yellow:
            # pair each blue with its nearest yellow -> "gates"
            gates = []
            for b in blue:
                y = min(yellow, key=lambda p: _dist(b, p))
                gates.append((_mid(b, y), _dist(b, y), _unit(_sub(b, y))))
            gates.sort(key=lambda g: _dist(g[0], car))
            if len(gates) >= 2:
                direction = orient(_unit(_sub(gates[-1][0], gates[0][0])))
                # FIX: order gates along the track (projection), not by distance
                gates.sort(key=lambda g: _dot(_sub(g[0], car), direction))
            else:  # single gate: drive perpendicular to the gate line
                direction = orient(_left_normal(gates[0][2]))
            # FIX: width measured from the real gates, so virtual cones line up
            # with the actual boundary rows (e.g. scenarios 7 and 14)
            width = sum(g[1] for g in gates) / len(gates)
            width = min(max(width, self.MIN_WIDTH), self.MAX_WIDTH)
            return direction, [car] + [g[0] for g in gates], width

        # only one side visible
        side = blue if blue else yellow
        left_side = bool(blue)
        side.sort(key=lambda p: _dist(p, car))

        if len(side) == 1:
            p = side[0]
            if _dot(_sub(p, car), yaw) < 0.0:            # cone behind the car: ignore
                return yaw, [car], self.WIDTH
            u = _unit(_sub(p, car))
            toward = _right_normal(u) if left_side else _left_normal(u)
            target = _add(p, _mul(toward, self.WIDTH / 2.0))  # cone stays on its side
            return _unit(_sub(target, car)), [car, target], self.WIDTH

        # 2+ cones on one side: boundary = line first->last cone, centerline =
        # that line shifted inward by half a track width
        e = orient(_unit(_sub(side[-1], side[0])))
        toward = _right_normal(e) if left_side else _left_normal(e)
        off = _mul(toward, self.WIDTH / 2.0)
        side.sort(key=lambda p: _dot(_sub(p, car), e))   # order along the boundary
        return e, [car] + [_add(p, off) for p in side], self.WIDTH

    # ------------------------------------------------------- 2. centerline polyline --
    def _centerline(self, waypoints: List[Point], direction: Point) -> List[Point]:
        """Polyline through the waypoints, made exactly the target length."""
        car = (self.car_pose.x, self.car_pose.y)
        # FIX: use the true distance to the farthest cone (projection
        # underestimated the length needed on bent tracks, e.g. 15)
        reach = 0.0
        for c in self.cones:
            reach = max(reach, _dist((c.x, c.y), car))
        target = min(max(reach + self.MARGIN, self.MIN_LEN), self.MAX_LEN)

        pts = [waypoints[0]]
        for p in waypoints[1:]:
            if _dist(p, pts[-1]) > 1e-9:
                pts.append(p)

        total = sum(_dist(a, b) for a, b in zip(pts, pts[1:]))
        if total < target - 1e-9:
            # FIX: extend along the estimated travel DIRECTION, not along the
            # last segment (that dragged scenario 14 out of its corridor)
            pts.append(_add(pts[-1], _mul(direction, target - total)))
        elif total > target + 1e-9:                      # truncate the end
            out, acc = [pts[0]], 0.0
            for a, b in zip(pts, pts[1:]):
                seg = _dist(a, b)
                if acc + seg >= target:
                    t = (target - acc) / seg
                    out.append(_add(a, _mul(_sub(b, a), t)))
                    break
                out.append(b)
                acc += seg
            pts = out
        return pts

    # --------------------------------------------------- 3. virtual cones on the grid --
    def _virtual_track(self, centerline: List[Point], width: float) -> List[Point]:
        """Place a left/right virtual cone pair (snapped to the integer grid) every
        CONE_SPACING meters along the centerline. Returns the midpoints of the
        pairs (the raw path)."""
        total = sum(_dist(a, b) for a, b in zip(centerline, centerline[1:]))
        half = width / 2.0
        left, right, mids = [], [], []
        s = 0.0
        while s <= total + 1e-9:
            p, t = self._point_at(centerline, s)
            if t == (0.0, 0.0):
                t = (1.0, 0.0)
            n = _left_normal(t)
            lv = _snap(_add(p, _mul(n, half)))           # virtual cone, LEFT
            rv = _snap(_sub(p, _mul(n, half)))           # virtual cone, RIGHT
            mid = _mid(lv, rv)
            # FIX: skip stations that collapsed onto the previous grid point
            # (caused duplicate cones / zero-length steps near corners)
            if not mids or _dist(mid, mids[-1]) > 1e-9:
                left.append(lv)
                right.append(rv)
                mids.append(mid)
            s += self.CONE_SPACING
        self.virtual_left, self.virtual_right = left, right
        return mids

    @staticmethod
    def _point_at(pts: List[Point], s: float):
        """Point and tangent on a polyline at arc length s."""
        acc = 0.0
        for a, b in zip(pts, pts[1:]):
            seg = _dist(a, b)
            if acc + seg >= s - 1e-9:
                t = 0.0 if seg < 1e-9 else (s - acc) / seg
                return _add(a, _mul(_sub(b, a), t)), _unit(_sub(b, a))
            acc += seg
        if len(pts) >= 2:
            return pts[-1], _unit(_sub(pts[-1], pts[-2]))
        return pts[-1], (1.0, 0.0)

    # --------------------------------------------------------------- 4. finishing --
    @staticmethod
    def _densify(points: List[Point], step: float) -> Path2D:
        """Insert points on the straight segments so no gap exceeds `step`.
        (Only needed because the task requires path step <= 0.5 m.)"""
        out = [points[0]]
        for a, b in zip(points, points[1:]):
            n = max(1, int(math.ceil(_dist(a, b) / step)))
            for k in range(1, n + 1):
                out.append(_add(a, _mul(_sub(b, a), k / n)))
        return out

    def _fix_length(self, path: Path2D, direction: Point) -> Path2D:
        """Safety net: keep total length inside the required 5-10 m."""
        total = sum(_dist(a, b) for a, b in zip(path, path[1:]))
        if total > self.MAX_LEN:
            out, acc = [path[0]], 0.0
            for a, b in zip(path, path[1:]):
                seg = _dist(a, b)
                if acc + seg >= self.MAX_LEN:
                    t = (self.MAX_LEN - acc) / seg
                    out.append(_add(a, _mul(_sub(b, a), t)))
                    break
                out.append(b)
                acc += seg
            return out
        if total < self.MIN_LEN - 1e-9:
            path.append(_add(path[-1], _mul(direction, self.MIN_LEN - total)))
        return path