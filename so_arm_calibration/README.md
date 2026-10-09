# so_arm_calibration

Joint zero calibration for the SO-ARM101: how far each joint's reading is from the angle the
description means, measured with the arm itself and a pen, and folded into the URDF.

LeRobot's calibration sets each servo's zero where a person judges the middle of its travel to
be. A joint then reads a degree or a few off its description's angle, and the tip lands
millimetres from where the description puts it: about 5 mm for each degree at the shoulder, at
full reach.

## How it works

The arm's torque is off. The closed jaw tips are rested by hand on a few dots marked on the table,
each dot in several arm shapes. Every touch of one dot puts the tips on the same point, whatever
the joints read. So the zero offsets are the ones that make the description agree with itself
about where each dot is.

- The dots need not be measured: their positions are fitted too.
- They share one height, the table's, level with the arm's base. That is what fixes the
  shoulder lift's zero.
- The closed tips' offset from the description's tip frame is fitted as well.

What it finds: `shoulder_lift_joint`, `elbow_flex_joint`, `wrist_flex_joint`.

What it cannot find, held at their LeRobot zeros:

- `shoulder_pan_joint`: turning the base turns every dot with it and fits as well.
- `wrist_roll_joint`: the closed tips are too near the roll axis to show its turn.

With three dots in five shapes each (15 touches), the tests recover offsets of up to 3 degrees to
within 0.3 degrees, the tips settling 0.5 mm from each dot's centre.

## Calibrating

Stop anything using the servo bus, clamp the arm to a flat table, and mark three dots within reach
with a pen: one near the base, one far out in front, one to the side.

```bash
ros2 run so_arm_calibration calibrate_zeros --port /dev/ttyACM0 --release --out ~/so101_zeros
```

- **`--release`** turns the servos' torque off after asking you to hold the arm. It is the only
  thing the tool writes to the servos.
- **For each dot, five shapes:** the tool straight down, then leaning about 40 degrees towards
  the base, away from it, and to either side. Rest the closed tips in the dot, hold still, and
  press Enter. The tool says how the tool leans in each touch, so you can check the shape.
- **The result** is each offset with its uncertainty, and how far the tips were from their dots.
  A touch more than 2 mm off is named, so you can redo it.

It writes, in `--out`:

- **`joint_zeros.yaml`:** each offset in radians (add it to the joint's reading for the
  description's angle), its uncertainty, the tips' rms distance from their dots, and the servos'
  homing offsets at the time.
- **`so_arm101_calibrated.urdf`:** the description with each offset folded in. Each joint's
  origin is turned by its offset about the joint's axis, and its limits shifted so they bound the
  same physical travel.
- **`touches.json`:** every touch, to refit later with `--refit` and no arm.

The offsets hold for the homing offsets they were measured with. After a new LeRobot calibration,
run it again.

To fold an existing `joint_zeros.yaml` into another description (one generated with your own
xacro arguments, say):

```bash
ros2 run so_arm_calibration apply_zeros --urdf so_arm101.urdf --zeros ~/so101_zeros/joint_zeros.yaml --out calibrated.urdf
```

Use the corrected URDF wherever the description is loaded (`robot_state_publisher`'s
`robot_description`, a planner, a simulator), so forward and inverse kinematics see the arm as
it is.
