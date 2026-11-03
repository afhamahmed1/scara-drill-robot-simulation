"""Drill job: read a job file, check the holes, sort them, then drill them.

    ros2 run scara_drill drill_job --ros-args -p job:=jobs/example.yaml

1. check  every problem with the holes is listed, and a bad job is refused
2. sort   nearest neighbour + 2-opt, prints the travel before and after
3. show   puts the sheet into Gazebo; RViz shows the sheet, the holes numbered in
          drilling order and the path
4. drill  each hole in order: move above it, down fast, feed slowly through the sheet, back
          up. The hole then appears in Gazebo and turns green in RViz. A summary at the end.

Publishes:   /scara/j1_cmd, /scara/j2_cmd, /scara/j3_cmd  (std_msgs/Float64) to the joint motors
             /drill_markers  (visualization_msgs/MarkerArray) for RViz
Subscribes:  /joint_states  (sensor_msgs/JointState) to see when the arm has arrived
"""

import os
import subprocess
import sys
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point
from rclpy.executors import ExternalShutdownException
from visualization_msgs.msg import Marker, MarkerArray

from scara_drill.kinematics import TIP_TOP, ik
from scara_drill.move_to import MoveTo
from scara_drill.planner import (APPROACH_HEIGHT, DEPTH_BELOW, HOME, SAFE_HEIGHT, check,
                                 hole_positions, sort_holes, travel)

WORLD = 'drill_cell'  # world name in scara_sim/worlds/drill_cell.sdf
TABLE_TOP = 0.20      # m, height of the table top in that world
FEED_SPEED = 10       # mm/s, how fast the drill goes through the sheet
FEED_STEP = 0.5       # mm, the slow feed is sent to the quill motor in steps this small

SHEET_COLOUR = (0.8, 0.8, 0.85)
HOLE_COLOUR = (0.4, 0.4, 0.4)     # RViz: grey = not drilled yet
DRILLED_COLOUR = (0.1, 0.8, 0.1)  # RViz: green = drilled
GZ_HOLE_COLOUR = (0.1, 0.1, 0.1)  # Gazebo: a drilled hole is a dark disc on the sheet
PATH_COLOUR = (1.0, 0.6, 0.0)
TEXT_COLOUR = (1.0, 1.0, 1.0)


def find_job_file(path):
    """Use the path as given, or else look in this package's installed jobs folder."""
    if os.path.exists(path):
        return path
    return os.path.join(get_package_share_directory('scara_drill'), path)


def gz_service(service, reqtype, request):
    """Call a Gazebo world service (e.g. create or remove a model). True if it worked."""
    result = subprocess.run(
        ['gz', 'service', '-s', f'/world/{WORLD}/{service}', '--reqtype', reqtype,
         '--reptype', 'gz.msgs.Boolean', '--timeout', '3000', '--req', request],
        capture_output=True, text=True)
    return 'data: true' in result.stdout


def spawn_model(name, xyz, geometry, colour):
    """Add a static model to Gazebo that is only seen, never touched. xyz in metres.

    geometry is SDF, e.g. '<box><size>0.3 0.2 0.003</size></box>'.
    """
    x, y, z = xyz
    r, g, b = colour
    sdf = (f"<sdf version='1.10'><model name='{name}'><static>true</static>"
           f"<pose>{x} {y} {z} 0 0 0</pose><link name='link'><visual name='visual'>"
           f"<geometry>{geometry}</geometry>"
           f"<material><ambient>{r} {g} {b} 1</ambient><diffuse>{r} {g} {b} 1</diffuse></material>"
           f"</visual></link></model></sdf>")
    return gz_service('create', 'gz.msgs.EntityFactory', f'sdf: "{sdf}"')


def remove_old_models():
    """Remove the sheet and the drilled holes of the previous job from Gazebo."""
    models = subprocess.run(['gz', 'model', '--list'], capture_output=True, text=True).stdout
    for name in models.split():
        if name == 'sheet' or name.startswith('hole_'):
            gz_service('remove', 'gz.msgs.Entity', f'name: "{name}" type: MODEL')


def make_marker(ns, marker_id, marker_type, xyz, scale, colour):
    """An RViz marker in the world frame. xyz and scale in metres."""
    marker = Marker(ns=ns, id=marker_id, type=marker_type, action=Marker.ADD)
    marker.header.frame_id = 'world'
    marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = xyz
    marker.pose.orientation.w = 1.0
    marker.scale.x, marker.scale.y, marker.scale.z = scale
    marker.color.r, marker.color.g, marker.color.b = colour
    marker.color.a = 1.0
    return marker


class DrillJob(MoveTo):
    """MoveTo gives us move() and send() for the joint motors."""

    def __init__(self):
        super().__init__('drill_job')
        self.job_file = self.declare_parameter('job', 'jobs/example.yaml').value
        self.marker_pub = self.create_publisher(MarkerArray, '/drill_markers', 10)
        self.hole_markers = {}  # RViz disc of each hole, by drilling number

    def run(self):
        """Check, sort, show and drill the job. False if the job is refused or stopped."""
        path = find_job_file(self.job_file)
        if not os.path.exists(path):
            print(f'Job file not found: {self.job_file}')
            return False
        with open(path) as f:
            job = yaml.safe_load(f)
        sheet, diameter = job['sheet'], job['drill']['diameter']
        print(f"Job {path}: {sheet['width']} x {sheet['height']} x {sheet['thickness']} mm sheet, "
              f"{diameter} mm drill, {len(job['holes'])} holes")

        # 1. Check
        problems = check(job)
        if problems:
            print(f'Job refused, {len(problems)} problem(s):')
            for problem in problems:
                print(f'  - {problem}')
            return False
        print('All holes OK')

        # 2. Sort
        holes = hole_positions(job)
        order = sort_holes(holes)
        print(f'Travel: as given {travel(holes):.0f} mm → sorted {travel(order):.0f} mm')
        print('Drilling order (hole numbers from the file):',
              ', '.join(str(holes.index(hole) + 1) for hole in order))

        # 3. Show
        remove_old_models()
        if not self.spawn_sheet(sheet):
            print('Could not put the sheet into Gazebo (is the simulation running?)')
        self.show_plan(sheet, diameter, order)

        # 4. Drill
        start = time.time()
        try:
            self.drill(sheet, diameter, order)
        except TimeoutError as error:
            print(f'Stopped: {error}')
            return False
        print(f'Done: {len(order)} holes drilled, travel {travel(order):.0f} mm, '
              f'{time.time() - start:.0f} s')
        return True

    def spawn_sheet(self, sheet):
        """Put the sheet on the table in Gazebo. False if Gazebo didn't answer."""
        w, h, t = sheet['width'] / 1000, sheet['height'] / 1000, sheet['thickness'] / 1000
        centre = (sheet['corner'][0] / 1000 + w / 2,  # Gazebo places a box by its centre
                  sheet['corner'][1] / 1000 + h / 2,
                  TABLE_TOP + t / 2)
        return spawn_model('sheet', centre, f'<box><size>{w} {h} {t}</size></box>', SHEET_COLOUR)

    def show_plan(self, sheet, diameter, order):
        """Publish RViz markers: the sheet, a numbered grey disc per hole, and the path."""
        t = sheet['thickness'] / 1000
        top = TABLE_TOP + t  # sheet surface (m)
        w, h = sheet['width'] / 1000, sheet['height'] / 1000
        corner_x, corner_y = sheet['corner'][0] / 1000, sheet['corner'][1] / 1000

        markers = MarkerArray()
        markers.markers.append(Marker(action=Marker.DELETEALL))  # clear the previous job
        markers.markers.append(make_marker('sheet', 0, Marker.CUBE,
                                           (corner_x + w / 2, corner_y + h / 2, TABLE_TOP + t / 2),
                                           (w, h, t), SHEET_COLOUR))

        for n, (x, y) in enumerate(order, start=1):
            x, y = x / 1000, y / 1000
            disc = make_marker('holes', n, Marker.CYLINDER, (x, y, top),
                               (diameter / 1000, diameter / 1000, 0.001), HOLE_COLOUR)
            self.hole_markers[n] = disc
            markers.markers.append(disc)
            number = make_marker('numbers', n, Marker.TEXT_VIEW_FACING, (x, y, top + 0.015),
                                 (0.0, 0.0, 0.02), TEXT_COLOUR)  # text: only scale.z (height) counts
            number.text = str(n)
            markers.markers.append(number)

        path = make_marker('path', 0, Marker.LINE_STRIP, (0.0, 0.0, 0.0), (0.002, 0.0, 0.0), PATH_COLOUR)
        path.points = [Point(x=x / 1000, y=y / 1000, z=top + 0.001) for x, y in [HOME] + order]
        markers.markers.append(path)

        # Give RViz up to 2 s to connect, so it doesn't miss the markers
        end = time.time() + 2.0
        while self.marker_pub.get_subscription_count() == 0 and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.marker_pub.publish(markers)

    def drill(self, sheet, diameter, order):
        """Drill the holes in order. Heights here are in mm above the table."""
        top = sheet['thickness']  # sheet surface
        self.tip_to(*HOME, TIP_TOP * 1000)  # start at home, where the planned path begins

        for n, (x, y) in enumerate(order, start=1):
            self.tip_to(x, y, top + SAFE_HEIGHT)                  # 1. above the hole, at safe height
            self.tip_to(x, y, top + APPROACH_HEIGHT)              # 2. fast down to just over the sheet
            self.feed(x, y, top + APPROACH_HEIGHT, -DEPTH_BELOW)  # 3. slowly through the sheet
            self.tip_to(x, y, top + SAFE_HEIGHT)                  # 4. back up
            self.mark_drilled(n, x, y, sheet, diameter)           # 5. show the hole
            print(f'Hole {n}/{len(order)} drilled at ({x:.0f}, {y:.0f}) mm')

        self.tip_to(*HOME, TIP_TOP * 1000)  # back home, out of the way

    def tip_to(self, x, y, z):
        """Move the drill tip to (x, y, z) mm and wait until it is there."""
        if not self.move(ik(x / 1000, y / 1000, z / 1000)):
            raise TimeoutError('the arm did not get there (is the simulation running?)')

    def feed(self, x, y, z_from, z_to):
        """Lower the drill slowly from z_from to z_to (mm), at FEED_SPEED.

        The quill motor always runs at its top speed, so we send it many small steps instead.
        """
        steps = round((z_from - z_to) / FEED_STEP)
        for k in range(1, steps + 1):
            z = z_from + (z_to - z_from) * k / steps
            self.send(ik(x / 1000, y / 1000, z / 1000))
            time.sleep(FEED_STEP / FEED_SPEED)
        self.tip_to(x, y, z_to)  # wait until it is really there

    def mark_drilled(self, n, x, y, sheet, diameter):
        """Hole n is done: a dark disc on the sheet in Gazebo, and its RViz marker turns green."""
        top = TABLE_TOP + sheet['thickness'] / 1000
        spawn_model(f'hole_{n}', (x / 1000, y / 1000, top + 0.0005),
                    f'<cylinder><radius>{diameter / 2000}</radius><length>0.001</length></cylinder>',
                    GZ_HOLE_COLOUR)
        disc = self.hole_markers[n]
        disc.color.r, disc.color.g, disc.color.b = DRILLED_COLOUR
        self.marker_pub.publish(MarkerArray(markers=[disc]))


def main():
    rclpy.init()
    node = DrillJob()
    try:
        ok = node.run()
    except (KeyboardInterrupt, ExternalShutdownException):
        print('Stopped with Ctrl-C')  # the arm stays where it is
        ok = False
    node.destroy_node()
    rclpy.try_shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
