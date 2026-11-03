# SCARA Driller

A SCARA robot arm with a drill, simulated in Gazebo. You give it a sheet and a list of hole
positions; it sorts them into a short path and drills them. The full design is in
[PLAN.md](PLAN.md).

**All four stages are done:** the arm in Gazebo and RViz with a motor on each joint, a
`move_to` command that puts the drill tip at a point you give in millimetres, and a
`drill_job` command that checks a job file's holes, sorts them into a short path and drills them.


## 1. Build

```bash
cd ~/Robotics/Projects/scara_drill_ws
source /opt/ros/jazzy/setup.bash
colcon build
```

Run `colcon build` again after every change.

## 2. Run

```bash
source ~/Robotics/Projects/scara_drill_ws/install/setup.bash
ros2 launch scara_sim sim.launch.py
```

Gazebo shows the orange arm on its grey column next to a brown table. RViz shows the same
arm and its frames.

## 3. Move single joints

In a second terminal (run `source ~/Robotics/Projects/scara_drill_ws/install/setup.bash` first):

```bash
ros2 topic pub --once /scara/j1_cmd std_msgs/msg/Float64 "{data: 1.0}"   # shoulder, radians
ros2 topic pub --once /scara/j2_cmd std_msgs/msg/Float64 "{data: 0.5}"   # elbow, radians
ros2 topic pub --once /scara/j3_cmd std_msgs/msg/Float64 "{data: 0.10}"  # drill down, metres
```

The arm swings round and the drill goes down 10 cm. Then check where it is:

```bash
ros2 topic echo --once /joint_states        # position: 1.0, 0.5, 0.1
ros2 run tf2_ros tf2_echo world drill_tip   # Translation: [0.180, 0.502, 0.230]
```

The tip position follows from the joint values (forward kinematics):

```
x = 0.30·cos(j1) + 0.25·cos(j1 + j2)
y = 0.30·sin(j1) + 0.25·sin(j1 + j2)
z = 0.33 − j3
```

Working range: j1 ±150° (±2.62 rad), j2 ±145° (±2.53 rad), j3 0 to 0.15 m. Stay inside
it: a joint driven onto its end stop sticks there until you restart (a Gazebo quirk).

## 4. Move the drill tip to a point

```bash
ros2 run scara_drill move_to 350 0 50
```

X and Y are measured from the robot's base (x forward, y to the left), Z is the height of the
drill tip above the table, all in millimetres. It works out the joint angles, sends them to
the motors and waits until the arm is there:

```
Moving: j1 -44.4°, j2 101.5°, j3 80.0 mm
Arrived: tip at (350.0, 0.0, 50.0) mm, 0.00 mm from the target
```

Check it with TF (z is the table at 0.20 m plus 0.05 m):

```bash
ros2 run tf2_ros tf2_echo world drill_tip   # Translation: [0.350, 0.000, 0.250]
```

The arm reaches 172 to 550 mm from the base, and the tip goes from 20 mm below to 130 mm
above the table. Other points are refused:

```bash
ros2 run scara_drill move_to 600 0 50   # Refused: 600 mm from the base, but the arm reaches 172 to 550 mm
```

How the angles are found (inverse kinematics, in `kinematics.py`): base, elbow and tip make a
triangle with sides 0.30 m, 0.25 m and the distance to the tip, so the law of cosines gives
the elbow angle j2. The shoulder angle j1 is the direction to the tip minus the bend the
elbow adds. j3 is just the height. Each point has two answers (elbow bent left or right);
it always takes j2 ≥ 0, so the arm never flips between holes.

## 5. Run a drilling job

A job file gives the sheet and the holes in millimetres, measured from the sheet's corner like
on a drawing (see `src/scara_drill/jobs/example.yaml`). With the simulation running:

```bash
ros2 run scara_drill drill_job --ros-args -p job:=jobs/example.yaml
```

```
Job .../jobs/example.yaml: 300 x 200 x 3 mm sheet, 6 mm drill, 12 holes
All holes OK
Travel: as given 1908 mm → sorted 1031 mm
Drilling order (hole numbers from the file): 3, 10, 2, 8, 11, 4, 9, 12, 6, 7, 1, 5
Hole 1/12 drilled at (480, 80) mm
...
Hole 12/12 drilled at (480, -80) mm
Done: 12 holes drilled, travel 1031 mm, 46 s
```

The sheet appears on the table in Gazebo. In RViz, the **Drill plan** display shows the sheet,
a grey disc for each hole numbered in drilling order, and the orange path from the arm's home
position. As each hole is drilled, a dark disc appears on the sheet in Gazebo and its RViz
disc turns green. At the end the arm goes back home. Ctrl-C stops it where it is.

What it does:

1. **Check:** each hole must be on the sheet, with its centre at least 5 mm from the edges,
   within the arm's reach, and at least one drill diameter (centre to centre) from every other
   hole. A bad job is refused, with every problem listed:

   ```bash
   ros2 run scara_drill drill_job --ros-args -p job:=jobs/bad.yaml
   ```

   ```
   Job refused, 4 problem(s):
     - hole 2 at [-10, 50]: off the 380 x 200 mm sheet
     - hole 3 at [3, 100]: closer than 5 mm to the edge
     - hole 6 at [370, 100]: out of reach, 570 mm from the base, but the arm reaches 172 to 550 mm
     - holes 4 and 5: only 4.0 mm apart, they would overlap with a 6 mm drill
   ```

2. **Sort:** nearest neighbour (start at home, always go to the closest hole left), then
   2-opt (wherever two legs of the path cross, reverse the holes between them; repeat until
   nothing gets shorter).
3. **Show:** put the sheet into Gazebo (replacing the last job's sheet and holes) and send
   the plan to RViz.
4. **Drill:** go home, then for each hole: move above it at 30 mm over the sheet, go down fast
   to 5 mm over it, feed slowly (10 mm/s) to 2 mm through it, go back up, mark it drilled.

For your own job, give any path: `-p job:=/path/to/my_job.yaml`. Short names like
`jobs/example.yaml` are looked up in the installed package, so after editing a file in
`src/scara_drill/jobs/`, run `colcon build` again.

## 6. Unit tests

The kinematics and the planner are tested without Gazebo:

```bash
colcon test --packages-select scara_drill --event-handlers console_direct+
```

It should end with `25 passed`.

## Files

| File | What it does |
|---|---|
| `src/scara_sim/urdf/scara.urdf.xacro` | The arm: column, two links, quill with spindle and drill bit, `drill_tip` frame, plus the Gazebo joint motors |
| `src/scara_sim/worlds/drill_cell.sdf` | The world: ground, sun and the work table (top at z = 0.20 m) |
| `src/scara_sim/config/bridge.yaml` | Which topics are copied between Gazebo and ROS 2 |
| `src/scara_sim/launch/sim.launch.py` | Starts Gazebo, the arm, the bridge and RViz |
| `src/scara_sim/rviz/scara.rviz` | RViz layout: the arm, its frames and the drill plan |
| `src/scara_drill/scara_drill/kinematics.py` | IK and FK: joint angles ↔ drill tip position, and the reach checks |
| `src/scara_drill/scara_drill/move_to.py` | `move_to` node: point in mm → IK → joint motors, waits until the arm arrives |
| `src/scara_drill/scara_drill/planner.py` | Checks a job's holes and sorts them (nearest neighbour + 2-opt); drill cycle heights |
| `src/scara_drill/scara_drill/drill_job.py` | `drill_job` node: reads a job file, checks, sorts, shows the plan, then drills every hole |
| `src/scara_drill/jobs/example.yaml` | Example job: flange plate with 12 holes in a messy order |
| `src/scara_drill/jobs/bad.yaml` | A job with mistakes on purpose, to see the checks |
| `src/scara_drill/test/test_kinematics.py` | Unit tests for the kinematics |
| `src/scara_drill/test/test_planner.py` | Unit tests for the planner |
| `PLAN.md` | The design and the four stages |

## Stages (git branches)

Each stage has its own branch. `main` always holds the latest finished stage.

| Branch | What it adds | Status |
|---|---|---|
| `stage-1-arm` | SCARA arm in Gazebo + RViz, joint motors, bridge | done |
| `stage-2-ik` | Inverse kinematics + `move_to` node | done |
| `stage-3-plan` | Job file, hole checks, path sorting, sheet in Gazebo | done |
| `stage-4-drill` | Drill cycle, holes appear on the sheet, summary | done |

Open a stage, then rebuild from clean:

```bash
git switch stage-4-drill
rm -rf build install log && colcon build
```
