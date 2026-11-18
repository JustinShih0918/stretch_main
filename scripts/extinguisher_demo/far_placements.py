#!/usr/bin/env python3
"""Seeded spots to move an extinguisher to, far from where the map has it.

    python3 scripts/extinguisher_demo/far_placements.py --seed 3 --min 8 --max 30
    # -> seed 3: 1 -21.35 9.05 -80   (12.6 m from #1's place, 27 m walk from spawn)
    ./scripts/extinguisher_demo/move_extinguisher.sh 1 -21.35 9.05 -80

A spot is somewhere an extinguisher would stand: against a wall (the cells behind
it, across its width, are wall), on the floor clear of furniture, with room in
front for the robot to stand 0.8-1.6 m off and look at it, reachable from the
lobby, and not near #2 or the robot's spawn. Its yaw turns its front to the room,
as the stage's own placements are. `--seed` picks among the spots whose distance
from #1's place lies in [--min, --max] m, so a seed names the same spot every time.
`--png` draws it on the floor plan.

The floor plan is hospital_occupancy.png beside this script: Isaac's occupancy map
of stretch3_og_hospital.usda at 0.4-0.5 m above the floor, 0.1 m cells, black
occupied, white free, grey never reached. The patch around the robot's spawn reads
grey (the generator's flood fill did not reach it at that height), so reachability
is measured from the lobby just north of it.

System python3 (numpy, scipy, OpenCV).
"""
import argparse
import json
import math
from pathlib import Path

import cv2
import numpy as np
from scipy import ndimage

HERE = Path(__file__).resolve().parent
RES = 0.1
X0, Y1 = -49.6, 37.1                   # world x of column 0's edge, world y of row 0's edge
HOME_1, HOME_2 = (-7.5, 5.12), (-9.6, 5.12)
LOBBY = (0.0, 2.5)                     # a free cell next to spawn, to measure reach from
SPAWN = (0.0, 0.0)
# The asset's front points along heading yaw + 170 deg: yaw 100 faces south from
# the corridor's north wall, -80 faces north from the chairs.
FRONT_OFFSET_DEG = 170.0


def load_grid(path=HERE / "hospital_occupancy.png"):
    img = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise SystemExit(f"no floor plan at {path}")
    return img == 255, img == 0          # free, occupied


def rc(x, y):
    return int((Y1 - y) / RES), int((x - X0) / RES)


def xy(r, c):
    return X0 + (c + 0.5) * RES, Y1 - (r + 0.5) * RES


def candidates(free, occ):
    """Every wall spot, with its facing, its distance from #1's place and its walk
    from spawn."""
    clear = ndimage.distance_transform_edt(free) * RES             # m to the nearest non-free cell
    # Where the robot's body (0.3 m radius) fits, connected to the lobby.
    roam = clear >= 0.35
    lab, _ = ndimage.label(roam, structure=np.ones((3, 3)))
    region = lab == lab[rc(*LOBBY)]
    # Walking distance over that region, from the lobby (8-connected, metres).
    walk = _walk(region, rc(*LOBBY))
    smooth = cv2.GaussianBlur(clear.astype(np.float32), (0, 0), 1.5)
    gy, gx = np.gradient(smooth)        # rows grow southward
    out = []
    rows, cols = np.nonzero(free & (clear >= 0.2) & (clear <= 0.3))
    for r, c in zip(rows, cols):
        nx, ny = gx[r, c], -gy[r, c]     # away from the wall, in world axes
        n = math.hypot(nx, ny)
        if n < 1e-6:
            continue
        nx, ny = nx / n, ny / n
        x, y = xy(r, c)
        if not _wall_behind(occ, x, y, nx, ny):
            continue
        front = [rc(x + nx * d, y + ny * d) for d in (0.8, 1.2, 1.6)]
        if not all(_inside(region, f) and region[f] and clear[f] >= 0.45 for f in front):
            continue
        heading = math.degrees(math.atan2(ny, nx))
        out.append({
            "x": round(x, 2), "y": round(y, 2),
            "yaw": int(round((heading - FRONT_OFFSET_DEG + 180) % 360 - 180)),
            "heading": round(heading, 1),
            "from_home_m": round(math.dist((x, y), HOME_1), 1),
            "walk_from_spawn_m": round(float(walk[front[1]]), 1),
        })
    return [p for p in out
            if math.dist((p["x"], p["y"]), HOME_2) > 2.0 and math.dist((p["x"], p["y"]), SPAWN) > 3.0
            and np.isfinite(p["walk_from_spawn_m"])]


def _inside(a, rcpos):
    return 0 <= rcpos[0] < a.shape[0] and 0 <= rcpos[1] < a.shape[1]


def _wall_behind(occ, x, y, nx, ny):
    """Wall 0.35-0.5 m behind the spot, across the extinguisher's 0.4 m width."""
    tx, ty = -ny, nx
    for back in (0.35, 0.5):
        for side in (-0.2, 0.0, 0.2):
            p = rc(x - nx * back + tx * side, y - ny * back + ty * side)
            if not (_inside(occ, p) and occ[p]):
                return False
    return True


def _walk(region, start):
    """Shortest 8-connected walking distance (m) from `start` over `region`."""
    import heapq

    dist = np.full(region.shape, np.inf)
    dist[start] = 0.0
    heap = [(0.0, start)]
    steps = [(dr, dc, RES * math.hypot(dr, dc)) for dr in (-1, 0, 1) for dc in (-1, 0, 1) if dr or dc]
    while heap:
        d, (r, c) = heapq.heappop(heap)
        if d > dist[r, c]:
            continue
        for dr, dc, w in steps:
            rr, cc = r + dr, c + dc
            if 0 <= rr < region.shape[0] and 0 <= cc < region.shape[1] and region[rr, cc] and d + w < dist[rr, cc]:
                dist[rr, cc] = d + w
                heapq.heappush(heap, (d + w, (rr, cc)))
    return dist


def pick(seed, lo, hi, spots):
    band = [p for p in spots if lo <= p["from_home_m"] <= hi]
    if not band:
        raise SystemExit(f"no spot {lo}-{hi} m from #1's place")
    return dict(np.random.default_rng(seed).choice(band), seed=seed, band=[lo, hi], n_in_band=len(band))


def draw(path, free, occ, spots):
    img = np.full(free.shape + (3,), 90, np.uint8)
    img[free] = (235, 235, 235)
    img[occ] = (25, 25, 25)
    img = cv2.resize(img, None, fx=3, fy=3, interpolation=cv2.INTER_NEAREST)

    def px(x, y):
        r, c = rc(x, y)
        return c * 3 + 1, r * 3 + 1

    for name, (x, y), col in (("#1", HOME_1, (0, 0, 220)), ("#2", HOME_2, (0, 140, 0)), ("spawn", SPAWN, (200, 0, 0))):
        cv2.circle(img, px(x, y), 7, col, -1)
        cv2.putText(img, name, (px(x, y)[0] + 8, px(x, y)[1] - 6), 0, 0.6, col, 2)
    for p in spots:
        u, v = px(p["x"], p["y"])
        h = math.radians(p["heading"])
        cv2.circle(img, (u, v), 8, (0, 90, 255), -1)
        cv2.line(img, (u, v), (int(u + 22 * math.cos(h)), int(v - 22 * math.sin(h))), (0, 90, 255), 3)
        cv2.putText(img, f"seed {p['seed']}", (u + 10, v + 18), 0, 0.6, (0, 90, 255), 2)
    cv2.imwrite(str(path), img)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--seed", type=int, nargs="+", default=[0])
    ap.add_argument("--min", type=float, default=8.0, help="metres from #1's place, at least")
    ap.add_argument("--max", type=float, default=30.0, help="metres from #1's place, at most")
    ap.add_argument("--png", type=Path, help="draw the spots on the floor plan")
    ap.add_argument("--json", action="store_true", help="one JSON object per seed")
    args = ap.parse_args()
    free, occ = load_grid()
    spots = candidates(free, occ)
    chosen = [pick(s, args.min, args.max, spots) for s in args.seed]
    for p in chosen:
        if args.json:
            print(json.dumps(p))
        else:
            print(f"seed {p['seed']}: 1 {p['x']} {p['y']} {p['yaw']}   ({p['from_home_m']} m from #1's place, "
                  f"{p['walk_from_spawn_m']} m walk from spawn; {p['n_in_band']} spots in {p['band']} m)")
    if args.png:
        draw(args.png, free, occ, chosen)


if __name__ == "__main__":
    main()
