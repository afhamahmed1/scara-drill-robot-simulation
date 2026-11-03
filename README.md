# SCARA Driller

A SCARA robot arm with a drill, simulated in Gazebo. You give it a sheet and a list of hole
positions; it sorts them into a short path and drills them. The full design is in
[PLAN.md](PLAN.md).

**Stage 1 (this branch):** the arm in Gazebo and RViz, with a motor on each joint that you
command from ROS 2.


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

## 3. Check it works

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

## Files

| File | What it does |
|---|---|
| `src/scara_sim/urdf/scara.urdf.xacro` | The arm: column, two links, quill with spindle and drill bit, `drill_tip` frame, plus the Gazebo joint motors |
| `src/scara_sim/worlds/drill_cell.sdf` | The world: ground, sun and the work table (top at z = 0.20 m) |
| `src/scara_sim/config/bridge.yaml` | Which topics are copied between Gazebo and ROS 2 |
| `src/scara_sim/launch/sim.launch.py` | Starts Gazebo, the arm, the bridge and RViz |
| `src/scara_sim/rviz/scara.rviz` | RViz layout: the arm and its frames |
| `PLAN.md` | The design and the four stages |

## Stages (git branches)

Each stage has its own branch. `main` always holds the latest finished stage.

| Branch | What it adds | Status |
|---|---|---|
| `stage-1-arm` | SCARA arm in Gazebo + RViz, joint motors, bridge | done |
| `stage-2-ik` | Inverse kinematics + `move_to` node | planned |
| `stage-3-plan` | Job file, hole checks, path sorting, sheet in Gazebo | planned |
| `stage-4-drill` | Drill cycle, holes appear on the sheet, summary | planned |

Open a stage, then rebuild from clean:

```bash
git switch stage-1-arm
rm -rf build install log && colcon build
```
