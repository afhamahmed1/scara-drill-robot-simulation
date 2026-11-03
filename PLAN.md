# SCARA Driller — project plan

A SCARA robot arm with a drill on its end, simulated in Gazebo. You give it a sheet and a
list of hole positions; it checks them, sorts them into a short path, and drills them one
by one. Drilled holes appear on the sheet.

Stack: ROS 2 Jazzy + Gazebo Harmonic (same as Object Seeker), Python nodes.

## How it works

```
job file (sheet + holes, in mm)
   │
   ▼
drill_job node ── 1. check   holes on the sheet? reachable? not overlapping?
               ── 2. sort    nearest neighbour, then 2-opt  → short path
               ── 3. drill   for each hole: move above → plunge → retract
                      │
                      ▼  joint angles (inverse kinematics)
   /scara/j1_cmd  /scara/j2_cmd  /scara/j3_cmd ──► bridge ──► Gazebo joint motors
                                                                  │
   RViz: numbered holes + path (grey → green when drilled)   ◄────┘ /joint_states
```

## The robot

A SCARA arm moves like a human arm on a table: two horizontal "shoulder" and "elbow"
joints position the tool in X–Y, and a vertical slide moves it up and down. That is exactly
what drilling needs: the drill always points straight down.

| Joint | Type | Does | Range |
|---|---|---|---|
| `j1` shoulder | revolute, vertical axis | swings link 1 (0.30 m) | ±150° |
| `j2` elbow | revolute, vertical axis | swings link 2 (0.25 m) | ±145° |
| `j3` quill | prismatic, down | lowers the drill | 0–0.15 m |

- Base column 0.40 m tall; the arm works over a table whose top is at z = 0.20 m.
- Reach: 0.17–0.55 m from the base axis (j2's ±145° limit stops the arm folding closer).
- The drill bit is visual only (no collision shape): Gazebo can't cut, so the bit must not
  collide with the sheet.
- Each joint gets a Gazebo `JointPositionController` ("motor that goes to the position you
  send it"), with `use_velocity_commands` so the vertical slide isn't fighting gravity.

## The job file — how you give coordinates

Everything in **millimetres**, holes measured from the sheet's corner, like on a drawing:

```yaml
sheet:
  width: 300            # mm, along the robot's x axis
  height: 200           # mm, along y
  thickness: 3          # mm
  corner: [200, -100]   # where the sheet's (0, 0) corner sits, in robot coordinates (mm)
drill:
  diameter: 6           # mm
holes:                  # [x, y] from the sheet corner, any order
  - [20, 20]
  - [280, 180]
  - [150, 100]
  - [40, 160]
```

Run it with:

```bash
ros2 run scara_drill drill_job --ros-args -p job:=jobs/example.yaml
```

The node spawns the sheet in Gazebo at that size and position, so each job can use a
different sheet.

## Checking the holes

The job is refused (with every problem listed at once) if a hole is:
- off the sheet, or closer than 5 mm to an edge,
- outside the arm's reach,
- closer than one drill diameter to another hole.

## Sorting the holes

The drill should travel as little as possible between holes. This is the "travelling
salesman problem"; two simple steps get close to the best answer:

1. **Nearest neighbour:** start at the arm's home position and always go to the closest
   hole not yet drilled.
2. **2-opt:** if two legs of the path cross, reverse the part between them; repeat until no
   swap makes the path shorter.

It prints the travel distance as given vs. sorted, e.g. `as given 1840 mm → sorted 910 mm`.

## The drill cycle (per hole)

1. Move above the hole at safe height (30 mm over the sheet).
2. Move down quickly to 5 mm over the sheet.
3. Feed down slowly to 2 mm through the sheet.
4. Retract to safe height.
5. Mark the hole as drilled: a dark disc appears on the sheet in Gazebo, and its RViz
   marker turns green.

At the end it prints a summary: holes drilled, distance travelled, time taken.

## Inverse kinematics (IK)

To put the drill at (x, y), the arm needs the angles j1 and j2. For a two-link arm this is
one triangle, solved with the law of cosines. There are two answers (elbow left or elbow
right); always use the same one so the arm doesn't flip between holes. The height is simply
j3. IK and FK (angles → position, to double-check) live in a plain Python file with
pytest unit tests, so they can be tested without Gazebo.

## Packages

| Package | Contents |
|---|---|
| `scara_sim` | URDF, world (ground + table), bridge config, launch file, RViz layout |
| `scara_drill` | `kinematics.py`, `planner.py` (check + sort), nodes `move_to` and `drill_job`, `jobs/*.yaml`, tests |

## Stages (one git branch each, `main` = latest finished stage)

| Branch | Adds | Done when |
|---|---|---|
| `stage-1-arm` | SCARA model in Gazebo + RViz, joint motors, bridge | `ros2 topic pub --once /scara/j1_cmd std_msgs/msg/Float64 "{data: 1.0}"` swings the arm |
| `stage-2-ik` | IK/FK + `move_to` node: `ros2 run scara_drill move_to 350 0 50` (mm) | drill tip lands within 1 mm of the target (checked with TF); unreachable points refused |
| `stage-3-plan` | job file, checks, sorting, sheet spawned, RViz shows numbered path | bad holes are reported; distance before/after sorting printed |
| `stage-4-drill` | drill cycle, holes appear, summary | every hole drilled in sorted order, all markers green |

## Later ideas

- Send holes live with a ROS service instead of a file.
- Import holes from a CSV or DXF drawing.
- Switch the joint motors to `ros2_control` + `joint_trajectory_controller` (already
  installed), then MoveIt.
- Peck drilling for thick sheets (drill a bit, retract, repeat).
