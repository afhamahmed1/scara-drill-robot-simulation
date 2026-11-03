"""Unit tests for planner.py (no ROS or Gazebo needed)."""

import random

import pytest

from scara_drill.planner import HOME, check, sort_holes, travel, two_opt


def make_job(holes, width=300):
    """A 300 x 200 x 3 mm sheet at the usual place, with a 6 mm drill."""
    return {'sheet': {'width': width, 'height': 200, 'thickness': 3, 'corner': [200, -100]},
            'drill': {'diameter': 6},
            'holes': holes}


# ----- check -----

def test_good_holes_pass():
    assert check(make_job([[20, 20], [280, 180], [150, 100], [40, 160]])) == []


@pytest.mark.parametrize('holes, width, expected', [
    ([[20, 20], [310, 50]], 300, 'hole 2 at [310, 50]: off the'),
    ([[3, 100]], 300, 'hole 1 at [3, 100]: closer than 5 mm to the edge'),
    ([[100, 100], [104, 100]], 300, 'holes 1 and 2: only 4.0 mm apart'),
    ([[370, 100]], 400, 'hole 1 at [370, 100]: out of reach'),  # 570 mm from the base
])
def test_bad_hole_is_reported(holes, width, expected):
    problems = check(make_job(holes, width))
    assert len(problems) == 1
    assert problems[0].startswith(expected)


def test_every_problem_is_listed_at_once():
    problems = check(make_job([[-10, 50], [3, 100], [100, 100], [104, 100], [370, 100]], width=400))
    assert len(problems) == 4


# ----- sorting -----

def test_nearest_neighbour_starts_next_to_home():
    # HOME is at x = 550, so the hole at x = 300 comes first
    assert sort_holes([(100, 0), (300, 0), (200, 0)]) == [(300, 0), (200, 0), (100, 0)]


def test_two_opt_undoes_a_crossing():
    # From the first hole the path goes diagonally across the square and back: the legs cross
    crossing = [HOME, (400, 100), (300, -100), (300, 100), (400, -100)]
    shortest = 180.3 + 100 + 200 + 100  # around the edge of the square
    assert travel(two_opt(crossing)[1:]) == pytest.approx(shortest, abs=0.1)


def test_sorting_keeps_every_hole_and_is_never_longer():
    random.seed(0)
    holes = [(random.uniform(200, 500), random.uniform(-100, 100)) for _ in range(30)]
    order = sort_holes(holes)
    assert sorted(order) == sorted(holes)
    assert travel(order) <= travel(holes)
