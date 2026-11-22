#!/usr/bin/env python3
"""
traffic_sim.py  --  the "brain" of the Duckietown city traffic
=============================================================================
Pure Python (no ROS, no Gazebo) so it can be tested on its own.

Cars drive on the RIGHT-hand lane of the real road tiles, round every corner
on a smooth arc, slow down in curves, keep a safe gap to the car in front,
and stop for the TurtleBot if it is on their lane.
Pedestrians walk the sidewalks, pause at the ends and turn around.

Road facts used (your world, tiles are 1.2 m wide):
  * lane centre = 0.30 m to the right of the road centre-line
  * every corner tile is a quarter circle, centre-line radius 0.6 m
"""
import math

# ----------------------------------------------------------------- tuning --
LANE_OFFSET = 0.30      # m  right of road centre-line
CORNER_R    = 0.60      # m  centre-line radius of every corner tile
STEP        = 0.02      # m  path resolution
A_LAT       = 0.25      # m/s^2 side acceleration allowed -> slows in corners
A_ACC       = 0.50      # m/s^2 gentle start
A_BRAKE     = 1.00      # m/s^2 comfortable braking
LOOK        = 1.40      # m  how far ahead a car looks for obstacles
CAR_HALF_LEN, CAR_HALF_W = 0.31, 0.15
STOP_GAP    = 0.12      # m  bumper-to-bumper distance when stopped

# Road centre-lines (corner points).  Order = direction of travel.
#  BIG_LOOP : row0 east -> curve up -> row1 west -> curve up -> row2 west
#             -> down the centre connector -> back to row0
#  WEST_RING: the small block on the west side
BIG_LOOP  = [(-3.3, 0.6), (1.49, 0.6), (1.49, 1.8), (-0.9, 1.8), (-0.9, 3.0), (-3.3, 3.0)]
WEST_RING = [(-5.69, 0.6), (-3.3, 0.6), (-3.3, 3.0), (-5.69, 3.0)]   # (spare route, no cars use it)
ROUTES = {'big': BIG_LOOP, 'west': WEST_RING}

# name, body style, colour, route, reverse?, start (0..1 along loop), cruise speed m/s
# Everyone shares ONE two-lane loop (3 cars one way, 2 the other way), so paths never
# cross and cars can never block each other forever.
CARS = [
    dict(name='npc_car_blue',    style='sedan', color=(0.10, 0.35, 0.90), route='big', reverse=False, start=0.08, speed=0.60),
    dict(name='npc_taxi_yellow', style='taxi',  color=(0.95, 0.78, 0.08), route='big', reverse=False, start=0.42, speed=0.52),
    dict(name='npc_car_green',   style='sedan', color=(0.12, 0.62, 0.25), route='big', reverse=False, start=0.75, speed=0.56),
    dict(name='npc_car_red',     style='sedan', color=(0.85, 0.12, 0.10), route='big', reverse=True,  start=0.20, speed=0.60),
    dict(name='npc_suv_white',   style='suv',   color=(0.92, 0.92, 0.92), route='big', reverse=True,  start=0.65, speed=0.50),
]

# Sidewalk walkers: go from a to b, wait, come back.
PEDESTRIANS = [
    dict(name='npc_ped_south', a=(-5.6, -0.08), b=(1.5, -0.08), speed=0.22, start=0.10, shirt=(0.15, 0.60, 0.30)),
    dict(name='npc_ped_north', a=(0.4, 3.76),  b=(-5.6, 3.76), speed=0.18, start=0.60, shirt=(0.85, 0.35, 0.15)),
]

# --------------------------------------------------------------- geometry --
def _wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def _round_polygon(pts, r):
    """Closed polygon -> same polygon with every corner replaced by an arc."""
    n, out = len(pts), []
    for i in range(n):
        p0, p1, p2 = pts[i - 1], pts[i], pts[(i + 1) % n]
        a = (p1[0] - p0[0], p1[1] - p0[1])
        b = (p2[0] - p1[0], p2[1] - p1[1])
        la, lb = math.hypot(*a), math.hypot(*b)
        ua, ub = (a[0] / la, a[1] / la), (b[0] / lb, b[1] / lb)
        turn = math.atan2(ua[0] * ub[1] - ua[1] * ub[0], ua[0] * ub[0] + ua[1] * ub[1])
        t = min(r * math.tan(abs(turn) / 2), 0.5 * la - 1e-6, 0.5 * lb - 1e-6)
        rr = t / math.tan(abs(turn) / 2)
        sign = 1 if turn > 0 else -1
        start = (p1[0] - ua[0] * t, p1[1] - ua[1] * t)
        c = (start[0] - ua[1] * rr * sign, start[1] + ua[0] * rr * sign)
        a0 = math.atan2(start[1] - c[1], start[0] - c[0])
        k = max(2, int(abs(turn) * rr / STEP) + 1)
        for j in range(k + 1):
            ang = a0 + sign * abs(turn) * j / k
            out.append((c[0] + rr * math.cos(ang), c[1] + rr * math.sin(ang)))
    return out


def _resample_closed(pts, step):
    """Make the points of a closed loop evenly spaced."""
    loop = pts + [pts[0]]
    cum = [0.0]
    for i in range(len(loop) - 1):
        cum.append(cum[-1] + math.hypot(loop[i + 1][0] - loop[i][0], loop[i + 1][1] - loop[i][1]))
    total = cum[-1]
    n = max(8, round(total / step))
    ds, out, j = total / n, [], 0
    for k in range(n):
        s = k * ds
        while cum[j + 1] < s:
            j += 1
        seg = cum[j + 1] - cum[j]
        t = (s - cum[j]) / seg if seg > 0 else 0.0
        out.append((loop[j][0] + t * (loop[j + 1][0] - loop[j][0]),
                    loop[j][1] + t * (loop[j + 1][1] - loop[j][1])))
    return out, ds


class LanePath:
    """A closed lane loop that a car can follow: pose(s) and speed limit(s)."""

    def __init__(self, polygon, reverse=False):
        poly = polygon[::-1] if reverse else polygon
        centre, _ = _resample_closed(_round_polygon(poly, CORNER_R), STEP)
        n = len(centre)
        lane = []
        for i in range(n):                       # shift to the RIGHT lane
            a, b = centre[i - 1], centre[(i + 1) % n]
            tx, ty = b[0] - a[0], b[1] - a[1]
            l = math.hypot(tx, ty)
            lane.append((centre[i][0] + LANE_OFFSET * ty / l, centre[i][1] - LANE_OFFSET * tx / l))
        self.pts, self.ds = _resample_closed(lane, STEP)
        self.n = len(self.pts)
        self.length = self.n * self.ds
        k = 3
        self.yaw = [math.atan2(self.pts[(i + k) % self.n][1] - self.pts[i - k][1],
                               self.pts[(i + k) % self.n][0] - self.pts[i - k][0]) for i in range(self.n)]
        # corner speed limit (a_lat = v^2 * curvature), then look-ahead braking
        w = 6
        v = []
        for i in range(self.n):
            kappa = abs(_wrap(self.yaw[(i + w) % self.n] - self.yaw[i - w])) / (2 * w * self.ds)
            v.append(min(2.0, math.sqrt(A_LAT / max(kappa, 1e-6))))
        for _ in range(2):
            for i in range(self.n - 1, -1, -1):
                j = (i + 1) % self.n
                v[i] = min(v[i], math.sqrt(v[j] ** 2 + 2 * 0.6 * A_BRAKE * self.ds))
        self.vlim = v

    def pose(self, s):
        f = (s % self.length) / self.ds
        i = int(f) % self.n
        t, j = f - int(f), (int(f) + 1) % self.n
        x = self.pts[i][0] + t * (self.pts[j][0] - self.pts[i][0])
        y = self.pts[i][1] + t * (self.pts[j][1] - self.pts[i][1])
        return x, y, self.yaw[i] + t * _wrap(self.yaw[j] - self.yaw[i])

    def limit(self, s):
        return self.vlim[int((s % self.length) / self.ds) % self.n]


# -------------------------------------------------------------- behaviour --
class TrafficSim:
    def __init__(self):
        cache = {}
        self.cars = []
        for idx, c in enumerate(CARS):
            key = (c['route'], c['reverse'])
            if key not in cache:
                cache[key] = LanePath(ROUTES[c['route']], c['reverse'])
            self.cars.append(dict(cfg=c, idx=idx, path=cache[key], s=c['start'] * cache[key].length, v=0.0))
        self.peds = [dict(cfg=p, pos=0.0, dirn=1, wait=0.0) for p in PEDESTRIANS]
        for p in self.peds:
            L = math.dist(p['cfg']['a'], p['cfg']['b'])
            p['pos'] = p['cfg']['start'] * L
        self.t = 0.0

    # --- helpers
    def _circles(self, poses):
        """Every car as 3 circles so long cars are detected side-on too."""
        out = []
        for i, (x, y, yaw) in enumerate(poses):
            for off in (-0.2, 0.0, 0.2):
                out.append((i, x + off * math.cos(yaw), y + off * math.sin(yaw), 0.16))
        return out

    def _free_distance(self, car, poses, circles, robot):
        p, s, me = car['path'], car['s'], car['idx']
        best = LOOK
        for k in range(int(LOOK / 0.06) + 1):
            d = k * 0.06
            x, y, yaw = p.pose(s + d)
            cy_, sy_ = math.cos(yaw), math.sin(yaw)
            hit = False
            for (j, cx, cy, r) in circles:
                if j == me:
                    continue
                for off in (0.0, 0.2, 0.3):         # middle, front, bumper of OUR body
                    if math.hypot(x + off * cy_ - cx, y + off * sy_ - cy) < r + CAR_HALF_W:
                        hit = True
                        break
                if hit:
                    break
            if hit:
                best = min(best, d)
            fx, fy = x + CAR_HALF_LEN * cy_, y + CAR_HALF_LEN * sy_
            if robot is not None and math.hypot(fx - robot[0], fy - robot[1]) < 0.24 + CAR_HALF_W:
                best = min(best, d)
            if best < LOOK:
                break
        return best

    # --- one time step; returns {model_name: (x, y, z, yaw)}
    def step(self, dt, robot=None):
        dt = min(dt, 0.1)
        self.t += dt
        poses = [c['path'].pose(c['s']) for c in self.cars]
        circles = self._circles(poses)
        out = {}
        for c in self.cars:
            gap = self._free_distance(c, poses, circles, robot)
            v_obst = 2.0 if gap >= LOOK else math.sqrt(2 * A_BRAKE * max(gap - STOP_GAP, 0.0))
            v_want = min(c['cfg']['speed'], c['path'].limit(c['s']), v_obst)
            if c['v'] < v_want:
                c['v'] = min(v_want, c['v'] + A_ACC * dt)
            else:
                c['v'] = max(v_want, c['v'] - 1.5 * A_BRAKE * dt)
            c['s'] += c['v'] * dt
            x, y, yaw = c['path'].pose(c['s'])
            out[c['cfg']['name']] = (x, y, 0.004, yaw)
        for p in self.peds:
            cfg = p['cfg']
            (ax, ay), (bx, by) = cfg['a'], cfg['b']
            L = math.hypot(bx - ax, by - ay)
            if p['wait'] > 0:
                p['wait'] -= dt
            else:
                p['pos'] += p['dirn'] * cfg['speed'] * dt
                if p['pos'] >= L or p['pos'] <= 0:
                    p['pos'] = min(max(p['pos'], 0.0), L)
                    p['dirn'] *= -1
                    p['wait'] = 2.5
            f = p['pos'] / L
            yaw = math.atan2(by - ay, bx - ax) + (0 if p['dirn'] > 0 else math.pi)
            bob = 0.006 * abs(math.sin(self.t * 7.0)) if p['wait'] <= 0 else 0.0
            out[cfg['name']] = (ax + f * (bx - ax), ay + f * (by - ay), bob, yaw)
        return out

    def initial_poses(self):
        poses = {}
        for c in self.cars:
            x, y, yaw = c['path'].pose(c['s'])
            poses[c['cfg']['name']] = (x, y, 0.004, yaw)
        for p in self.peds:
            cfg = p['cfg']
            L = math.dist(cfg['a'], cfg['b'])
            f = p['pos'] / L
            poses[cfg['name']] = (cfg['a'][0] + f * (cfg['b'][0] - cfg['a'][0]),
                                  cfg['a'][1] + f * (cfg['b'][1] - cfg['a'][1]), 0.0,
                                  math.atan2(cfg['b'][1] - cfg['a'][1], cfg['b'][0] - cfg['a'][0]))
        return poses
