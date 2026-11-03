"""Planner: check the holes of a job, then sort them into a short drilling path.

Units: millimetres. A job is the dict read from a job file (see jobs/example.yaml):
holes are [x, y] from the sheet's corner; the sheet's corner sits at `corner` in robot coordinates.
"""

from math import dist

from scara_drill.kinematics import L1, L2, ik

EDGE_MARGIN = 5  # hole centres must be at least this far from the sheet's edges
HOME = ((L1 + L2) * 1000, 0.0)  # where the drill starts: all joints at 0, arm straight along x

# Drill cycle heights
SAFE_HEIGHT = 30      # over the sheet: move between holes up here
APPROACH_HEIGHT = 5   # over the sheet: go down fast to here, then feed slowly
DEPTH_BELOW = 2       # drill this far past the bottom of the sheet


def hole_positions(job):
    """The holes in robot coordinates: sheet corner + position on the sheet."""
    cx, cy = job['sheet']['corner']
    return [(cx + hx, cy + hy) for hx, hy in job['holes']]


def check(job):
    """Return every problem with the job's holes, as a list of messages (empty = all good)."""
    width, height = job['sheet']['width'], job['sheet']['height']
    safe_z = job['sheet']['thickness'] + SAFE_HEIGHT  # highest point of the drill cycle
    diameter = job['drill']['diameter']
    holes = job['holes']
    problems = []

    for n, ((hx, hy), (x, y)) in enumerate(zip(holes, hole_positions(job)), start=1):
        hole = f'hole {n} at [{hx}, {hy}]'

        # On the sheet, and not too close to an edge?
        if not (0 <= hx <= width and 0 <= hy <= height):
            problems.append(f'{hole}: off the {width} x {height} mm sheet')
        elif not (EDGE_MARGIN <= hx <= width - EDGE_MARGIN and EDGE_MARGIN <= hy <= height - EDGE_MARGIN):
            problems.append(f'{hole}: closer than {EDGE_MARGIN} mm to the edge')

        # Can the arm get there, up to the safe height? (ik wants metres)
        try:
            ik(x / 1000, y / 1000, safe_z / 1000)
        except ValueError as error:
            problems.append(f'{hole}: out of reach, {error}')

    # Two holes closer than one drill diameter (centre to centre) would overlap
    for i in range(len(holes)):
        for j in range(i + 1, len(holes)):
            gap = dist(holes[i], holes[j])
            if gap < diameter:
                problems.append(f'holes {i + 1} and {j + 1}: only {gap:.1f} mm apart, '
                                f'they would overlap with a {diameter} mm drill')
    return problems


def travel(holes):
    """How far the drill moves: from HOME through the holes in this order."""
    path = [HOME] + list(holes)
    return sum(dist(a, b) for a, b in zip(path, path[1:]))


def two_opt(path):
    """Shorten a path by undoing crossings. path[0] is the start and stays first.

    If two legs cross, reversing the holes between them uncrosses them and makes the path
    shorter. Keep trying every pair of legs until no reversal helps any more.
    """
    improved = True
    while improved:
        improved = False
        for i in range(1, len(path) - 1):
            for j in range(i + 1, len(path)):
                # Reversing path[i..j] only changes two legs: a-b becomes a-c, c-d becomes b-d
                a, b, c = path[i - 1], path[i], path[j]
                if j + 1 < len(path):
                    d = path[j + 1]
                    change = dist(a, c) + dist(b, d) - dist(a, b) - dist(c, d)
                else:  # c is the last hole: there is no leg after it
                    change = dist(a, c) - dist(a, b)
                if change < -1e-9:
                    path[i:j + 1] = path[i:j + 1][::-1]
                    improved = True
    return path


def sort_holes(holes):
    """Return the holes in a short drilling order, starting from HOME."""
    # 1. Nearest neighbour: from where the drill is, always go to the closest hole left
    path = [HOME]
    left = list(holes)
    while left:
        closest = min(left, key=lambda hole: dist(path[-1], hole))
        path.append(closest)
        left.remove(closest)

    # 2. 2-opt: undo the crossings nearest neighbour leaves behind
    return two_opt(path)[1:]
