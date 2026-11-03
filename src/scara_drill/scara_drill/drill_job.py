"""Drill job: read a job file, check the holes, sort them and show the plan.

    ros2 run scara_drill drill_job --ros-args -p job:=jobs/example.yaml

1. check  every problem with the holes is listed, and a bad job is refused
2. sort   nearest neighbour + 2-opt, prints the travel before and after
3. show   puts the sheet into Gazebo; RViz shows the sheet, the holes numbered in
          drilling order and the path

Publishes:  /drill_markers  (visualization_msgs/MarkerArray) for RViz
"""

import os
import subprocess
import sys
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray

from scara_drill.planner import HOME, check, hole_positions, sort_holes, travel

WORLD = 'drill_cell'  # world name in scara_sim/worlds/drill_cell.sdf
TABLE_TOP = 0.20      # m, height of the table top in that world

SHEET_COLOUR = (0.8, 0.8, 0.85)
HOLE_COLOUR = (0.4, 0.4, 0.4)  # grey = not drilled yet
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


class DrillJob(Node):

    def __init__(self):
        super().__init__('drill_job')
        self.job_file = self.declare_parameter('job', 'jobs/example.yaml').value
        self.marker_pub = self.create_publisher(MarkerArray, '/drill_markers', 10)

    def run(self):
        """Check, sort and show the job. False if the job is refused."""
        path = find_job_file(self.job_file)
        if not os.path.exists(path):
            print(f'Job file not found: {self.job_file}')
            return False
        with open(path) as f:
            job = yaml.safe_load(f)
        sheet = job['sheet']
        print(f"Job {path}: {sheet['width']} x {sheet['height']} x {sheet['thickness']} mm sheet, "
              f"{job['drill']['diameter']} mm drill, {len(job['holes'])} holes")

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
        if not self.spawn_sheet(sheet):
            print('Could not put the sheet into Gazebo (is the simulation running?)')
        self.show_plan(sheet, job['drill']['diameter'], order)
        return True

    def spawn_sheet(self, sheet):
        """Put the sheet on the table in Gazebo, replacing the sheet of the previous job."""
        w, h, t = sheet['width'] / 1000, sheet['height'] / 1000, sheet['thickness'] / 1000
        x = sheet['corner'][0] / 1000 + w / 2  # Gazebo places a box by its centre
        y = sheet['corner'][1] / 1000 + h / 2
        z = TABLE_TOP + t / 2
        r, g, b = SHEET_COLOUR
        sdf = (f"<sdf version='1.10'><model name='sheet'><static>true</static>"
               f"<pose>{x} {y} {z} 0 0 0</pose><link name='link'><visual name='visual'>"
               f"<geometry><box><size>{w} {h} {t}</size></box></geometry>"
               f"<material><diffuse>{r} {g} {b} 1</diffuse></material>"
               f"</visual></link></model></sdf>")
        gz_service('remove', 'gz.msgs.Entity', 'name: "sheet" type: MODEL')  # False if none yet
        return gz_service('create', 'gz.msgs.EntityFactory', f'sdf: "{sdf}"')

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
            markers.markers.append(make_marker('holes', n, Marker.CYLINDER, (x, y, top),
                                               (diameter / 1000, diameter / 1000, 0.001), HOLE_COLOUR))
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
        time.sleep(0.5)  # let the message go out before the node exits


def main():
    rclpy.init()
    node = DrillJob()
    ok = node.run()
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
