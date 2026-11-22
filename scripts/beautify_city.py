#!/usr/bin/env python3
"""
beautify_city.py  --  make worlds/duckietown_city.sdf look like a modern smart city
=============================================================================
Run from the project folder (ES_Project):

    python3 scripts/beautify_city.py                 # day look (default)
    python3 scripts/beautify_city.py --mode night    # night look: lit windows + street lights
    python3 scripts/beautify_city.py --shadows       # sun shadows (prettier, slower)
    python3 scripts/beautify_city.py --gui           # also build a modern GUI theme (config/duckietown_gui.config)
    python3 scripts/beautify_city.py --remove        # undo everything, back to your original look

SAFETY RULES THIS SCRIPT FOLLOWS
  * Only ADDS new models whose names start with  deco_   -> your own models are never edited
  * Every decoration is VISUAL ONLY: no collision, no physics, no sensors, no plugins
  * Trees are kept 0.9 m away from the lanes; everything near the road is lower than the lidar plane
    (use --hide-from-sensors only if you want to experiment with GUI-only decorations)
  * Safe to run again and again;  the first backup is kept as  duckietown_city.sdf.pre_beautify.bak
"""
import argparse
import glob
import json
import math
import os
import random
import re
import shutil
import xml.etree.ElementTree as ET

WORLD = 'worlds/duckietown_city.sdf'
SIDECAR = 'worlds/.duckietown_original_look.json'
GUI_CFG_OUT = 'config/duckietown_gui.config'
GUI_ONLY_FLAG = 268435456            # 0x10000000 = "only the GUI camera renders this"
PREFIX = 'deco_'

# --------------------------------------------------------------- helpers ----
def f(v):
    return f'{v:.4f}'


def mat(c, spec=0.10, emis=None, alpha=1.0):
    r, g, b = c
    e = f'<emissive>{f(emis[0])} {f(emis[1])} {f(emis[2])} 1</emissive>' if emis else ''
    return (f'<material><ambient>{f(r * .8)} {f(g * .8)} {f(b * .8)} {alpha}</ambient>'
            f'<diffuse>{f(r)} {f(g)} {f(b)} {alpha}</diffuse>'
            f'<specular>{f(spec)} {f(spec)} {f(spec)} 1</specular>{e}</material>')


class Deco:
    """One decorative model made only of visuals (and optional lights)."""
    sensor_visible = True

    def __init__(self, name, x=0.0, y=0.0, z=0.0, yaw=0.0):
        self.name, self.pose, self.items, self.lights, self.n = PREFIX + name, (x, y, z, yaw), [], [], 0

    def _add(self, pos, rot, geom, material):
        self.n += 1
        flag = '' if Deco.sensor_visible else f'<visibility_flags>{GUI_ONLY_FLAG}</visibility_flags>'
        self.items.append(
            f'<visual name="v{self.n}"><pose>{f(pos[0])} {f(pos[1])} {f(pos[2])} {f(rot[0])} {f(rot[1])} {f(rot[2])}</pose>'
            f'<geometry>{geom}</geometry>{material}{flag}</visual>')

    def box(self, pos, size, color, rot=(0, 0, 0), spec=0.10, emis=None, alpha=1.0):
        self._add(pos, rot, f'<box><size>{f(size[0])} {f(size[1])} {f(size[2])}</size></box>', mat(color, spec, emis, alpha))

    def cyl(self, pos, r, l, color, rot=(0, 0, 0), spec=0.10, emis=None):
        self._add(pos, rot, f'<cylinder><radius>{f(r)}</radius><length>{f(l)}</length></cylinder>', mat(color, spec, emis))

    def sph(self, pos, r, color, spec=0.05, emis=None):
        self._add(pos, (0, 0, 0), f'<sphere><radius>{f(r)}</radius></sphere>', mat(color, spec, emis))

    def light(self, pos, color=(1.0, 0.82, 0.55), rng=2.0):
        r, g, b = color
        self.lights.append(
            f'<light name="l{len(self.lights) + 1}" type="point"><pose>{f(pos[0])} {f(pos[1])} {f(pos[2])} 0 0 0</pose>'
            f'<cast_shadows>false</cast_shadows><diffuse>{r} {g} {b} 1</diffuse><specular>0.1 0.1 0.1 1</specular>'
            f'<attenuation><range>{rng}</range><constant>0.3</constant><linear>0.9</linear><quadratic>0.6</quadratic></attenuation></light>')

    def xml(self):
        x, y, z, yaw = self.pose
        return (f'<model name="{self.name}"><static>true</static><pose>{f(x)} {f(y)} {f(z)} 0 0 {f(yaw)}</pose>'
                f'<link name="link">{"".join(self.items)}{"".join(self.lights)}</link></model>')


# ------------------------------------------------------------ world facts ---
def read_world(world):
    roads, buildings, lamps, signs = [], {}, [], []
    for m in world.findall('model'):
        n = m.get('name')
        p = m.find('pose')
        v = [float(t) for t in p.text.split()] if p is not None and p.text else [0] * 6
        box = m.find('.//box/size')
        if n.startswith('road') and box is not None:
            sx, sy = [float(t) for t in box.text.split()[:2]]
            if abs(math.sin(v[5])) > 0.5:
                sx, sy = sy, sx
            roads.append((v[0] - sx / 2, v[0] + sx / 2, v[1] - sy / 2, v[1] + sy / 2))
        elif n in ('apartment_block', 'house_1', 'house_2', 'house_3', 'gas_station', 'fast_food') and box is not None:
            sx, sy, h = [float(t) for t in box.text.split()]
            buildings[n] = dict(x=v[0], y=v[1], yaw=v[5], sx=sx, sy=sy, h=h)
        elif n.startswith('lamp_post'):
            lamps.append((v[0], v[1]))
        elif n in ('speed_limit_sign', 'traffic_light_north', 'traffic_light_south') or n.startswith('jersey'):
            signs.append((v[0], v[1]))
    return roads, buildings, lamps, signs


def footprint(b):
    c, s = abs(math.cos(b['yaw'])), abs(math.sin(b['yaw']))
    hx, hy = (b['sx'] * c + b['sy'] * s) / 2, (b['sx'] * s + b['sy'] * c) / 2
    return (b['x'] - hx, b['x'] + hx, b['y'] - hy, b['y'] + hy)


def in_rect(x, y, r, m=0.0):
    return r[0] - m <= x <= r[1] + m and r[2] - m <= y <= r[3] + m


# palette
WHITE, CONCRETE, DARK, ASPHALT = (0.92, 0.92, 0.90), (0.62, 0.62, 0.60), (0.14, 0.15, 0.17), (0.10, 0.10, 0.11)
GLASS, GLASS2, GLASS3 = (0.30, 0.52, 0.72), (0.22, 0.42, 0.55), (0.40, 0.62, 0.78)
GRASS = (0.20, 0.50, 0.17)
GREENS = [(0.14, 0.45, 0.14), (0.20, 0.55, 0.18), (0.11, 0.38, 0.16), (0.28, 0.58, 0.20), (0.16, 0.48, 0.24)]
WARM = (1.0, 0.82, 0.48)


# ------------------------------------------------------------- builders -----
def window_style(rnd, night):
    if night:
        if rnd.random() < 0.55:
            c = rnd.choice([(1.0, 0.84, 0.50), (1.0, 0.92, 0.70), (0.85, 0.92, 1.0)])
            return c, c, 0.2
        return (0.05, 0.08, 0.14), None, 0.5
    if rnd.random() < 0.12:
        return (0.88, 0.82, 0.68), None, 0.1          # curtains
    return rnd.choice([GLASS, GLASS2, GLASS3]), None, 0.8


def facade(d, b, rnd, night, door_face='+y', trim=WHITE):
    sx, sy, h = b['sx'], b['sy'], b['h']
    floors = max(1, int((h - 0.14) / 0.34))
    for face, (L, normal) in {'+x': (sy, (1, 0)), '-x': (sy, (-1, 0)), '+y': (sx, (0, 1)), '-y': (sx, (0, -1))}.items():
        cols = max(1, int((L - 0.12) / 0.30))
        gap = L / cols
        for c in range(cols):
            for fl in range(floors):
                if face == door_face and fl == 0 and c == cols // 2:
                    continue
                col, emis, spec = window_style(rnd, night)
                u = -L / 2 + gap * (c + 0.5)
                z = 0.17 + fl * (h - 0.26) / floors
                if normal[0]:
                    d.box((normal[0] * (sx / 2 + 0.003), u, z), (0.006, 0.12, 0.14), col, spec=spec, emis=emis)
                else:
                    d.box((u, normal[1] * (sy / 2 + 0.003), z), (0.12, 0.006, 0.14), col, spec=spec, emis=emis)
    # door
    dz = 0.075
    if door_face == '+y':
        d.box((0, sy / 2 + 0.004, dz), (0.13, 0.008, 0.15), (0.35, 0.22, 0.12))
        d.box((0, sy / 2 + 0.05, 0.17), (0.24, 0.10, 0.012), DARK)                      # little canopy
    elif door_face == '-y':
        d.box((0, -sy / 2 - 0.004, dz), (0.13, 0.008, 0.15), (0.35, 0.22, 0.12))
        d.box((0, -sy / 2 - 0.05, 0.17), (0.24, 0.10, 0.012), DARK)
    elif door_face == '+x':
        d.box((sx / 2 + 0.004, 0, dz), (0.008, 0.13, 0.15), (0.35, 0.22, 0.12))
        d.box((sx / 2 + 0.05, 0, 0.17), (0.10, 0.24, 0.012), DARK)
    # base plinth and corner pilasters
    d.box((0, 0, 0.02), (sx + 0.02, sy + 0.02, 0.04), (0.30, 0.30, 0.32))
    for sxn in (-1, 1):
        for syn in (-1, 1):
            d.box((sxn * sx / 2, syn * sy / 2, h / 2), (0.025, 0.025, h), trim)
    # roof slab with parapet
    d.box((0, 0, h + 0.012), (sx + 0.07, sy + 0.07, 0.024), (0.28, 0.28, 0.30))
    d.box((0, 0, h + 0.035), (sx + 0.03, sy + 0.03, 0.02), trim)


def gable_roof(d, b, color):
    sx, sy, h = b['sx'], b['sy'], b['h']
    a = 0.55
    span = sy / 2 + 0.06
    plen = span / math.cos(a)
    for s in (1, -1):
        d.box((0, s * span / 2, h + 0.03 + span / 2 * math.tan(a)), (sx + 0.10, plen, 0.02), color, rot=(-s * a, 0, 0), spec=0.05)
    d.box((0, 0, h + 0.03 + span * math.tan(a)), (sx + 0.10, 0.025, 0.025), (0.25, 0.12, 0.08))
    d.box((sx * 0.25, sy * 0.15, h + 0.20), (0.05, 0.05, 0.12), (0.55, 0.30, 0.25))   # chimney


def build_building(name, b, night, rnd):
    d = Deco(name, b['x'], b['y'], 0.0, b['yaw'])
    if name == 'apartment_block':
        facade(d, b, rnd, night, door_face='+x', trim=(0.85, 0.85, 0.82))
        floors = max(1, int((b['h'] - 0.14) / 0.27))
        for fl in range(1, floors):
            for k in (-1, 0, 1):
                z = 0.17 + fl * (b['h'] - 0.26) / floors - 0.08
                d.box((b['sx'] / 2 + 0.06, k * b['sy'] * 0.28, z), (0.12, 0.34, 0.012), WHITE)
                d.box((b['sx'] / 2 + 0.115, k * b['sy'] * 0.28, z + 0.03), (0.006, 0.34, 0.05), GLASS3, spec=0.8)
        for i in range(3):
            d.box((-0.4 + i * 0.35, 0.3 - i * 0.2, b['h'] + 0.10), (0.16, 0.12, 0.10), (0.55, 0.57, 0.6))   # roof AC units
        d.cyl((0.0, 0.0, b['h'] + 0.22), 0.012, 0.34, (0.7, 0.7, 0.72))                               # antenna
    elif name.startswith('house'):
        facade(d, b, rnd, night, door_face='-y',        # the road is to the south of all three houses
               
               trim=(0.95, 0.93, 0.88))
        gable_roof(d, b, [(0.62, 0.25, 0.18), (0.30, 0.34, 0.42), (0.45, 0.30, 0.22)][int(name[-1]) % 3])
        d.box((0, b['sy'] / 2 + 0.12, 0.012), (0.20, 0.22, 0.024), (0.7, 0.68, 0.64))                # doorstep path
    elif name == 'fast_food':
        facade(d, b, rnd, night, door_face='+y', trim=(0.95, 0.85, 0.30))
        for i in range(8):                                                                         # striped awning
            d.box((-0.50 + i * 0.143, b['sy'] / 2 + 0.07, 0.40), (0.143, 0.16, 0.012),
                  (0.85, 0.10, 0.10) if i % 2 == 0 else WHITE, rot=(0.35, 0, 0))
        d.box((0, 0, b['h'] + 0.24), (0.55, 0.05, 0.22), (0.95, 0.78, 0.10), emis=(0.6, 0.45, 0.0) if night else None)
        d.box((0, 0, b['h'] + 0.24), (0.50, 0.055, 0.16), (0.85, 0.10, 0.10))
        d.cyl((0.45, 0.0, b['h'] + 0.12), 0.012, 0.2, DARK)
    elif name == 'gas_station':
        facade(d, b, rnd, night, door_face='+x', trim=(0.20, 0.45, 0.30))
        cx = b['sx'] / 2 + 0.36                                                                    # canopy toward the road
        d.box((cx, 0, 0.44), (0.72, 1.60, 0.03), WHITE)
        d.box((cx, 0, 0.47), (0.72, 1.60, 0.012), (0.20, 0.45, 0.30))
        d.box((cx, 0, 0.405), (0.70, 1.58, 0.01), (1.0, 0.95, 0.85), emis=(0.9, 0.85, 0.7) if night else None)
        for px in (cx - 0.28, cx + 0.28):
            for py in (-0.7, 0.7):
                d.cyl((px, py, 0.21), 0.016, 0.42, (0.85, 0.85, 0.85))
        for py in (-0.35, 0.0, 0.35):
            d.box((cx, py, 0.065), (0.06, 0.05, 0.13), (0.85, 0.12, 0.10))
            d.box((cx, py, 0.14), (0.065, 0.055, 0.02), DARK)
        d.box((cx, 0, 0.005), (0.80, 1.70, 0.010), (0.22, 0.22, 0.24))                              # concrete apron
        if night:
            d.light((cx, 0, 0.40), (1.0, 0.95, 0.8), 1.6)
    return d


def build_lamp(i, x, y, roads, night):
    # arm points toward the nearest road
    best, tx, ty = 1e9, x, y
    for r in roads:
        cx, cy = min(max(x, r[0]), r[1]), min(max(y, r[2]), r[3])
        dist = math.hypot(cx - x, cy - y)
        if dist < best:
            best, tx, ty = dist, cx, cy
    ang = math.atan2(ty - y, tx - x) if best > 1e-6 else 0.0
    d = Deco(f'lamp_{i}', x, y, 0.0, ang)
    d.cyl((0, 0, 0.02), 0.035, 0.04, DARK)
    d.cyl((0, 0, 0.30), 0.016, 0.60, (0.25, 0.27, 0.30), spec=0.3)
    d.box((0.07, 0, 1.0), (0.16, 0.012, 0.012), (0.25, 0.27, 0.30))
    d.box((0.15, 0, 0.985), (0.10, 0.035, 0.016), (0.20, 0.22, 0.25))
    d.box((0.15, 0, 0.972), (0.085, 0.028, 0.008), (1.0, 0.95, 0.80), emis=WARM if night else (0.55, 0.52, 0.42))
    if night:
        d.light((0.15, 0, 0.94), (1.0, 0.82, 0.55), 1.8)
    return d


def build_tree(i, x, y, rnd):
    s = rnd.uniform(0.8, 1.25)
    d = Deco(f'tree_{i}', x, y)
    th = 0.22 * s
    d.cyl((0, 0, th / 2), 0.018 * s, th, (0.35, 0.23, 0.14))
    kind = rnd.random()
    c1, c2 = rnd.choice(GREENS), rnd.choice(GREENS)
    if kind < 0.65:                                              # round broadleaf
        d.sph((0, 0, th + 0.11 * s), 0.14 * s, c1)
        d.sph((0.07 * s, 0.03 * s, th + 0.18 * s), 0.10 * s, c2)
        d.sph((-0.06 * s, -0.05 * s, th + 0.17 * s), 0.10 * s, c1)
    else:                                                        # layered conifer
        for k in range(3):
            d.cyl((0, 0, th + 0.05 * s + k * 0.09 * s), 0.12 * s - k * 0.03 * s, 0.07 * s, (0.08, 0.34 + 0.04 * k, 0.16), spec=0.02)
    return d


def build_bench(name, x, y, yaw):
    d = Deco(name, x, y, 0.012, yaw)
    d.box((0, 0, 0.035), (0.20, 0.06, 0.012), (0.55, 0.38, 0.22))
    d.box((0, -0.03, 0.065), (0.20, 0.01, 0.05), (0.55, 0.38, 0.22), rot=(-0.2, 0, 0))
    for sx in (-0.085, 0.085):
        d.box((sx, 0, 0.017), (0.012, 0.05, 0.034), DARK)
    return d


def flowers(name, x, y, rnd, n=9, spread=0.12):
    d = Deco(name, x, y, 0.012)
    cols = [(0.95, 0.25, 0.35), (0.98, 0.85, 0.20), (0.95, 0.95, 0.95), (0.70, 0.30, 0.85), (0.98, 0.55, 0.15)]
    for k in range(n):
        a, r = rnd.uniform(0, 6.28), rnd.uniform(0.0, spread)
        d.sph((r * math.cos(a), r * math.sin(a), 0.012), 0.014, rnd.choice(cols))
    return d


def build_plaza(name, cx, cy, w, h, rnd, fountain=False):
    d = Deco(name, cx, cy)
    d.box((0, 0, 0.007), (w, h, 0.014), (0.24, 0.56, 0.20))                                   # raised lawn
    for sgn in (-1, 1):                                                                       # curb ring
        d.box((0, sgn * (h / 2 + 0.01), 0.011), (w + 0.04, 0.025, 0.022), (0.75, 0.75, 0.72))
        d.box((sgn * (w / 2 + 0.01), 0, 0.011), (0.025, h + 0.04, 0.022), (0.75, 0.75, 0.72))
    d.box((0, 0, 0.0145), (0.10, h - 0.04, 0.002), (0.78, 0.74, 0.66))                        # paved path
    if fountain:
        d.cyl((0, 0, 0.03), 0.17, 0.034, (0.78, 0.78, 0.76))
        d.cyl((0, 0, 0.049), 0.145, 0.006, (0.25, 0.55, 0.85), spec=0.9)
        d.cyl((0, 0, 0.08), 0.022, 0.07, (0.78, 0.78, 0.76))
        d.cyl((0, 0, 0.12), 0.07, 0.014, (0.78, 0.78, 0.76))
        d.cyl((0, 0, 0.135), 0.05, 0.006, (0.3, 0.6, 0.9), spec=0.9)
    for sx in (-1, 1):                                                                        # hedges
        d.box((sx * (w / 2 - 0.05), -h / 2 + 0.07, 0.04), (0.14, 0.07, 0.05), (0.12, 0.40, 0.14))
        d.box((sx * (w / 2 - 0.05), h / 2 - 0.07, 0.04), (0.14, 0.07, 0.05), (0.12, 0.40, 0.14))
    return d


def build_crosswalk(name, x, y, along_x, span, rot_bars=False):
    """Zebra crossing.  along_x=True: pedestrians cross a road that runs along X."""
    d = Deco(name, x, y, 0.0012 + 0.0003)
    n = int(span / 0.12)
    for k in range(n):
        u = -span / 2 + 0.06 + k * 0.12
        if along_x:
            d.box((0, u, 0), (0.30, 0.06, 0.0006), (0.93, 0.93, 0.93))
        else:
            d.box((u, 0, 0), (0.06, 0.30, 0.0006), (0.93, 0.93, 0.93))
    return d


def build_tower(i, x, y, w, dp, h, rnd, night):
    d = Deco(f'tower_{i}', x, y)
    glass = rnd.choice([(0.18, 0.30, 0.42), (0.25, 0.40, 0.50), (0.35, 0.45, 0.55), (0.15, 0.22, 0.30)])
    d.box((0, 0, h / 2), (w, dp, h), glass, spec=0.8)
    bands = int(h / 0.9)
    for k in range(1, bands + 1):
        d.box((0, 0, k * h / (bands + 1)), (w + 0.02, dp + 0.02, 0.03), (0.75, 0.78, 0.82))
    if night:
        for k in range(int(w / 0.30)):
            if rnd.random() < 0.7:
                d.box((-w / 2 + 0.12 + k * 0.30, dp / 2 + 0.003, h * rnd.uniform(0.2, 0.9)), (0.08, 0.004, h * 0.08),
                      (1.0, 0.85, 0.55), emis=(1.0, 0.85, 0.55))
    if rnd.random() < 0.5:
        d.cyl((0, 0, h + 0.3), 0.02, 0.6, (0.8, 0.8, 0.82))
        d.sph((0, 0, h + 0.62), 0.03, (1, 0.1, 0.1), emis=(1, 0.1, 0.1) if night else None)
    return d


# ------------------------------------------------------------- placement ----
def scatter_trees(rnd, roads, bldgs, lamps, signs, keep_out, n_target=28):
    pts, tries = [], 0
    foot = [footprint(b) for b in bldgs.values()]
    while len(pts) < n_target and tries < 20000:
        tries += 1
        x, y = rnd.uniform(-9.5, 4.8), rnd.uniform(-4.6, 8.0)
        if any(in_rect(x, y, r, 0.90) for r in roads):      # keep well clear of the lanes (and the lidar)
            continue
        if any(in_rect(x, y, r, 0.28) for r in foot):
            continue
        if any(math.hypot(x - lx, y - ly) < 0.30 for lx, ly in lamps + signs):
            continue
        if any(in_rect(x, y, r) for r in keep_out):
            continue
        if any(math.hypot(x - px, y - py) < 0.55 for px, py in pts):
            continue
        pts.append((x, y))
    return pts


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--mode', choices=['day', 'night'], default='day')
    ap.add_argument('--shadows', action='store_true', help='enable sun shadows (prettier, slower)')
    ap.add_argument('--gui', action='store_true', help='also generate a modern GUI theme config')
    ap.add_argument('--remove', action='store_true', help='remove all decorations / restore lighting')
    ap.add_argument('--hide-from-sensors', action='store_true', help='EXPERIMENTAL: tag decorations GUI-only')
    args = ap.parse_args()
    Deco.sensor_visible = not args.hide_from_sensors

    if not os.path.exists(WORLD):
        raise SystemExit(f'Cannot find {WORLD}. Run this from the ES_Project folder.')
    bak = WORLD + '.pre_beautify.bak'
    if not os.path.exists(bak):
        shutil.copyfile(WORLD, bak)

    tree = ET.parse(WORLD)
    world = tree.getroot().find('world')
    sun, scene = world.find("light[@name='sun']"), world.find('scene')

    # remember the original lighting once
    if not os.path.exists(SIDECAR):
        orig = {}
        if sun is not None:
            orig['sun_diffuse'] = sun.findtext('diffuse')
            orig['sun_shadows'] = sun.findtext('cast_shadows')
        if scene is not None:
            orig['ambient'] = scene.findtext('ambient')
            orig['background'] = scene.findtext('background')
        json.dump(orig, open(SIDECAR, 'w'))
    orig = json.load(open(SIDECAR))

    def restore_lighting():
        if sun is not None:
            sun.find('diffuse').text = orig.get('sun_diffuse', sun.findtext('diffuse'))
            sun.find('cast_shadows').text = orig.get('sun_shadows', 'false')
        if scene is not None:
            scene.find('ambient').text = orig.get('ambient', scene.findtext('ambient'))
            scene.find('background').text = orig.get('background', scene.findtext('background'))

    for el in list(world):
        if el.get('name', '').startswith(PREFIX):
            world.remove(el)
    restore_lighting()

    if args.remove:
        tree.write(WORLD, encoding='utf-8', xml_declaration=True)
        print('All decorations removed and original lighting restored.')
        return

    night = args.mode == 'night'
    rnd = random.Random(2026)
    roads, bldgs, lamps, signs = read_world(world)
    new = []

    # --- ground: big lawn (4 pieces so the green spawn marker at the origin stays visible)
    lawn = Deco('lawn')
    for (x0, x1, y0, y1) in [(-60, 60, 0.18, 50), (-60, 60, -50, -0.18), (-60, -0.18, -0.18, 0.18), (0.18, 60, -0.18, 0.18)]:
        lawn.box(((x0 + x1) / 2, (y0 + y1) / 2, 0.00025), (x1 - x0, y1 - y0, 0.0005), (0.18, 0.46, 0.16) if not night else (0.06, 0.16, 0.08))
    new.append(lawn)

    # --- sidewalks along the outer road edges
    sw = Deco('sidewalks')
    for (x0, x1, y0, y1) in [(-6.28, 2.09, -0.30, 0.0), (-6.28, -0.29, 3.6, 3.9), (-6.58, -6.28, 0.0, 3.6)]:
        sw.box(((x0 + x1) / 2, (y0 + y1) / 2, 0.010), (x1 - x0, y1 - y0, 0.020), (0.68, 0.68, 0.66))
        sw.box(((x0 + x1) / 2, (y0 + y1) / 2, 0.0205), (x1 - x0 - 0.02, y1 - y0 - 0.02, 0.001), (0.74, 0.74, 0.72))
    new.append(sw)

    # --- buildings (facade overlays; originals untouched)
    for name, b in bldgs.items():
        new.append(build_building(name, b, night, rnd))

    # --- street lights on top of your existing lamp posts
    for i, (lx, ly) in enumerate(lamps):
        new.append(build_lamp(i, lx, ly, roads, night))

    # --- zebra crossings
    new.append(build_crosswalk('crosswalk_row0', -2.0, 0.6, True, 1.08))
    new.append(build_crosswalk('crosswalk_row2', -2.0, 3.0, True, 1.08))
    new.append(build_crosswalk('crosswalk_link', -3.3, 1.8, False, 0.96))
    # --- pocket parks
    plazas = [(-4.49, 1.8, 1.0, 1.0, False), (-2.11, 1.8, 1.0, 1.0, True), (0.33, 3.25, 0.95, 1.2, False)]
    keep_out = []
    for i, (px, py, pw, ph, fount) in enumerate(plazas):
        new.append(build_plaza(f'plaza_{i}', px, py, pw, ph, rnd, fount))
        keep_out.append((px - pw / 2 - 0.05, px + pw / 2 + 0.05, py - ph / 2 - 0.05, py + ph / 2 + 0.05))
    new.append(build_bench('bench_0', -4.49, 1.45, 0.0))
    new.append(build_bench('bench_1', -2.11, 2.15, math.pi))
    new.append(build_bench('bench_2', 0.33, 2.8, 0.0))
    new.append(build_bench('bench_3', 0.33, 3.7, math.pi))
    for i, (px, py) in enumerate([(-4.49, 2.15), (-2.11, 1.45), (0.55, 3.25), (0.1, 3.25)]):
        new.append(flowers(f'flowers_{i}', px, py, rnd))
    # bus shelter on the south sidewalk
    bs = Deco('bus_stop', -1.3, -0.15)
    for px in (-0.28, 0.28):
        bs.cyl((px, 0.0, 0.17), 0.011, 0.34, DARK)
    bs.box((0, 0, 0.345), (0.64, 0.22, 0.014), (0.20, 0.45, 0.65))
    bs.box((0, -0.10, 0.17), (0.60, 0.006, 0.30), (0.55, 0.78, 0.9), spec=0.9, alpha=0.55)
    bs.box((0, -0.05, 0.065), (0.40, 0.06, 0.012), (0.55, 0.38, 0.22))
    bs.box((0.34, 0.08, 0.20), (0.012, 0.05, 0.40), DARK)
    bs.box((0.34, 0.08, 0.40), (0.07, 0.012, 0.07), (0.1, 0.4, 0.8))
    new.append(bs)

    # --- trees (inside the pocket parks first, then scattered on lawns and sidewalks)
    tid = 0
    for (px, py) in [(-4.49, 1.8), (0.33, 3.25), (0.55, 2.8), (0.1, 3.7)]:      # park trees (fountain stays clear)
        new.append(build_tree(tid, px, py, rnd))
        tid += 1
    for (x, y) in scatter_trees(rnd, roads, bldgs, lamps, signs, keep_out):
        new.append(build_tree(tid, x, y, rnd))
        tid += 1

    # --- skyline: tall glass towers far away (GUI only)
    cx, cy = -2.2, 2.0
    for i in range(14):
        a = i / 14 * 2 * math.pi + rnd.uniform(-0.08, 0.08)
        r = rnd.uniform(15.0, 24.0)
        new.append(build_tower(i, cx + r * math.cos(a), cy + r * math.sin(a),
                               rnd.uniform(0.9, 1.9), rnd.uniform(0.9, 1.9), rnd.uniform(2.0, 6.5), rnd, night))

    # --- lighting (visual only)
    sky_fill = ET.fromstring(
        f'<light name="{PREFIX}sky_fill" type="directional"><cast_shadows>false</cast_shadows><pose>0 0 12 0 0 0</pose>'
        f'<diffuse>{"0.05 0.06 0.12" if night else "0.30 0.34 0.42"} 1</diffuse><specular>0 0 0 1</specular>'
        f'<direction>0.4 -0.3 -0.8</direction></light>')

    for d in new:
        world.append(ET.fromstring(d.xml()))
    world.append(sky_fill)

    if sun is not None:
        sun.find('cast_shadows').text = 'true' if args.shadows else orig.get('sun_shadows', 'false')
        if night:
            sun.find('diffuse').text = '0.10 0.12 0.25 1'
    if night and scene is not None:
        scene.find('ambient').text = '0.16 0.18 0.28 1'
        scene.find('background').text = '0.03 0.05 0.12 1'

    tree.write(WORLD, encoding='utf-8', xml_declaration=True)
    print(f'Added {len(new)} decorative models ({tid} trees, {len(bldgs)} detailed buildings, '
          f'{len(lamps)} street lights, 14 skyline towers) in {args.mode} mode.')
    print(f'Visible to robot sensors: {"YES" if Deco.sensor_visible else "no (GUI only, experimental)"}')
    print('Undo any time:  python3 scripts/beautify_city.py --remove')

    if args.gui:
        make_gui_config()


# ------------------------------------------------------------- GUI theme ----
def find_default_gui_config():
    pats = ['/opt/ros/*/opt/gz_sim_vendor/share/gz/gz-sim*/gui/gui.config',
            '/opt/ros/*/share/gz/gz-sim*/gui/gui.config',
            '/usr/share/gz/gz-sim*/gui/gui.config',
            '/opt/ros/*/opt/gz_sim_vendor/**/gui.config']
    for p in pats:
        hits = sorted(glob.glob(p, recursive=True))
        if hits:
            return hits[-1]
    return None


def make_gui_config():
    src = find_default_gui_config()
    if not src:
        print('GUI: could not find Gazebo\'s default gui.config. Run:  find /opt/ros /usr -name gui.config 2>/dev/null\n'
              '     and send me the path.')
        return
    txt = open(src, encoding='utf-8').read()

    def set_tag(tag, value, txt):
        if re.search(f'<{tag}>.*?</{tag}>', txt, re.S):
            return re.sub(f'<{tag}>.*?</{tag}>', f'<{tag}>{value}</{tag}>', txt, count=1, flags=re.S)
        return re.sub(r'(<scene>.*?</scene>)', rf'\1\n      <{tag}>{value}</{tag}>', txt, count=1, flags=re.S)

    txt = set_tag('background_color', '0.62 0.78 0.95', txt)
    txt = set_tag('ambient_light', '0.55 0.55 0.58', txt)
    if '<sky' not in txt:
        txt = re.sub(r'(<scene>.*?</scene>)', r'\1\n      <sky></sky>', txt, count=1, flags=re.S)
    style = ('<style material_theme="Dark" material_primary="#11151c" material_accent="#00c2d1" '
             'toolBarColorLight="#11151c" toolBarTextColorLight="#e8f1f5" '
             'toolBarColorDark="#11151c" toolBarTextColorDark="#e8f1f5" '
             'plotBackgroundColorLight="#1b2130" plotBackgroundColorDark="#1b2130" '
             'plotTextColorLight="#e8f1f5" plotTextColorDark="#e8f1f5" '
             'plotGridColorLight="#3a4256" plotGridColorDark="#3a4256"/>')
    if re.search(r'<style[^>]*/>', txt):
        txt = re.sub(r'<style[^>]*/>', style, txt, count=1)
    elif re.search(r'<style[^>]*>.*?</style>', txt, re.S):
        txt = re.sub(r'<style[^>]*>.*?</style>', style, txt, count=1, flags=re.S)
    else:
        txt = re.sub(r'(<window>)', rf'\1\n    {style}', txt, count=1)
    os.makedirs(os.path.dirname(GUI_CFG_OUT), exist_ok=True)
    open(GUI_CFG_OUT, 'w', encoding='utf-8').write(txt)
    print(f'GUI theme written to {GUI_CFG_OUT} (based on {src}). The launch file uses it automatically.')


if __name__ == '__main__':
    main()