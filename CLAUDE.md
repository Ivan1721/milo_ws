# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

Workspace for **Milo** (`andesrobot`), a mobile manipulator: hoverboard differential base, RPLIDAR C1,
IMU, a vertical lift on a column, a 6-DOF arm and a two-finger gripper. ROS 2 **Humble**, Gazebo
**Classic 11**. Everything normally runs inside Docker (`docker/`, image `milo:humble`, container
`milo`); `ros2_ws/` is bind-mounted into the container as `/ros2_ws`, so code is edited on the host
and built/run in the container.

The user's own work is **the arm only** (`andesrobot_arm` + the arm parts of `andesrobot_description`).
Don't add or propose base-navigation features as next steps unless asked.

Repo docs are written in Spanish for students, with very dense explanatory comments (every launch,
xacro, yaml and script explains each concept). Match that: Spanish comments and log messages, same
comment density. `README.md` is step-by-step for non-experts; per-package details live in
`docs/*.md` (`andesrobot_description/docs/ROBOT.md`, `andesrobot_arm/docs/BRAZO.md`).

Repo-root `docs/` holds non-code material: `docs/informe/` (offline copy of the technical report,
versioned) and `docs/hardware/` (vendor manuals, datasheets, CAD for the EB300 arm, EBG-20 gripper,
motors and Orbbec camera; third-party, so only its `README.md` index is versioned). Keep
`ros2_ws/src/` for ROS packages only. `docs/escena_web/README.md` is a guide (documentation only, on purpose: the user builds the Vite +
three.js app themselves, so don't add app code there) describing the robot, scene, conventions and
how IK/camera work, with reference values. Its data pack (`milo.urdf`, meshes, `robot.json`,
`escena.json`) is generated into `ros2_ws/escena_web/` (gitignored) by
`andesrobot_arm/scripts/exportar_escena_web.py`; rerun it after geometry or world changes.

## Commands

All from `~/milo_ws` on the host:

```bash
./sim.sh                 # mapping sim: Gazebo + Milo (arm locked) + safety filter + slam_toolbox + RViz
./sim.sh brazo           # arm sim: Gazebo + Milo with arm/lift/gripper controlled + IK + RViz marker
./sim.sh teleop          # drive (keyboard -> /cmd_vel_teleop)
./sim.sh build           # colcon build --symlink-install inside the container
./sim.sh shell           # bash inside the container (ROS sourced)
./sim.sh stop            # docker compose down
./sim.sh mapa [nombre]   # save the slam map
./sim.sh cpu|software|sin-gazebo|gpu   # no NVIDIA / CPU rendering (black windows) / no gzclient / GPU check
./robot.sh ...           # same commands on the real robot's laptop (USB lidar + hoverboard);
                         # extra: rviz, puertos; ports via LIDAR=/dev/ttyUSB0 HOVER=... ./robot.sh
```

Docker plumbing shared by both scripts lives in `docker/lib.sh` (`milo_init`, `milo_up`, ...).
External drivers (today `sllidar_ros2`) are pinned in `docker/milo.repos` and fetched at image build
time with `vcs import` into `/opt/milo_drivers` (outside the workspace). To add or bump one, edit that
file; it lives in `docker/` because that is the image build context. Own drivers (hoverboard) stay
in `ros2_ws/src/drivers/`.

`build`, `shell` and `teleop` need the container already up (started by `./sim.sh` or
`./sim.sh brazo` in another terminal). `--symlink-install`: edits to existing `.py`/`.xacro`/`.yaml`
take effect on relaunch; new files need `./sim.sh build`. `milo_up` only builds automatically if
`install/` doesn't exist yet (`./sim.sh brazo` also builds if `andesrobot_arm` isn't installed).

Tests (pytest, no ROS graph needed; run inside the container after a build):

```bash
./sim.sh shell
source /ros2_ws/install/setup.bash
cd /ros2_ws/src/andesrobot_arm && python3 -m pytest -q test                      # all
cd /ros2_ws/src/andesrobot_arm && python3 -m pytest -q test -k ik_con_lift       # one test
cd /ros2_ws/src/andesrobot_safety && python3 -m pytest -q test
```

There are no ament_lint tests; keep Python within 99 columns (flake8) to match the code.

**Never run `colcon build` natively inside `~/milo_ws/ros2_ws`.** The container reuses
`ros2_ws/install/`, and host-built paths break it. For native (non-Docker) runs, build elsewhere:
`colcon build --symlink-install --build-base /tmp/milo_build --install-base /tmp/milo_install`
and `source /tmp/milo_install/setup.bash`. Natively, VS Code terminals auto-activate Anaconda
(`(base)` prompt) and ROS Python fails with `No module named 'rclpy._rclpy_pybind11'`:
`conda deactivate` first.

## Architecture

Packages (`ros2_ws/src/`):

| Package | Role |
|---|---|
| `andesrobot_description` | URDF/xacro, meshes, `rsp.launch.py`, `display.launch.py`. Single source of truth for geometry. |
| `andesrobot_gazebo` | `sim.launch.py` (world + rsp + spawn, arm **locked**), `andesrobot_arena.world` |
| `andesrobot_bringup` | top-level launches: `sim_mapping` (what `./sim.sh` runs), `robot`/`robot_mapping` (real), `rviz_mapping`; `milo_controllers.yaml` (diff drive, real robot) |
| `andesrobot_safety` | `safety_filter`: `/cmd_vel_teleop` + `/scan` → stops before obstacles → `cmd_vel_out`. Math in `logic.py` (unit-tested), ROS in `safety_filter.py` |
| `andesrobot_slam` | `laser_filters` (removes the column from the scan) + slam_toolbox online async |
| `andesrobot_arm` | arm kinematics/IK, `arm_ik_node`, `arm_marker_node`, `arm_controllers.yaml`, `arm_sim.launch.py` |
| `drivers/hoverboard_hardware_interface` | C++ ros2_control hardware plugin for the real wheels (serial) |

**Sim vs real parity for the base:** same topics in both. Sim uses Gazebo plugins from
`andesrobot.sim.xacro` (`libgazebo_ros_diff_drive` publishes `/odom` + TF `odom→base_footprint`,
ray sensor → `/scan`, IMU, `libgazebo_ros_joint_state_publisher`). Real uses ros2_control
(`andesrobot.ros2_control.xacro`, only with `use_hardware:=true`) + `diff_drive_controller`; the
safety filter's output is remapped to `/diff_drive_controller/cmd_vel_unstamped` there.

**URDF switches** (`andesrobot.urdf.xacro` args, documented in `docs/ROBOT.md`): `lock_arm` (arm, lift,
fingers become `fixed`; used by the mapping sim because the unactuated arm would collapse),
`simple_collision` (primitive boxes on `base_link`, including a fixed "brazo" box; arm links then have
**no** collision), `sim_lidar`/`sim_imu`, `use_hardware`/`hoverboard_port`, `arm_control` +
`arm_controllers_file`. Frames: `base_footprint` on the ground under the wheel axle; `base_link` at the
axle (z = 0.08255); `laser` is the C1 optical frame. The CAD was modeled rotated 180° about Z, fixed
with `rpy="0 0 π"` at base-level joints/visuals; a new child of `base_link` likely needs the same.

**Arm in simulation** (`./sim.sh brazo` → `andesrobot_arm/launch/arm_sim.launch.py`):
- Processes the xacro itself with `lock_arm:=false arm_control:=true` instead of using
  `rsp.launch.py`, because `gazebo_ros2_control` (Humble) re-passes `robot_description` as a CLI
  `--param` and fails to parse it when the URDF has XML comments (the controller manager never
  starts). The launch strips comments; keep that if you change how the URDF is produced.
- `arm_control:=true` includes `andesrobot.arm_control.xacro` (`GazeboSystem`, **position** command
  interfaces, `left_finger_joint` as `mimic` → appears as `left_finger_joint_mimic`) and removes the
  arm joints from the Gazebo joint_state plugin so `joint_state_broadcaster` is their only publisher.
- One `arm_controller` holds `vertical_lift_joint` + `joint_1..6` (so 7-DOF IK solutions move in
  sync; `allow_partial_joints_goal: true`); `gripper_controller` holds `right_finger_joint`.
- Controllers are spawned only if `spawn_entity` exits 0.
- Data flow: `arm_marker_node` (interactive marker at `gripper_tcp`) publishes `PoseStamped` on
  `/arm_target_pose` → `arm_ik_node` (TF into the IK chain's base frame, IK seeded from `/joint_states`) →
  `FollowJointTrajectory` on `/arm_controller/follow_joint_trajectory`. Anything that publishes
  `/arm_target_pose` can drive the arm (params: `use_lift`, `position_only`, `lift_weight`, ...).
- Uses mesh collisions by default (`simple_collision:=false`) because the simple boxes leave the arm
  without collision.
- `gripper_camera:=true` (on by default in `arm_sim`, arg `camara`) adds the Orbbec Gemini Plus on
  top of the gripper (`andesrobot.gripper_camera.xacro`, mesh `orbbec_gemini_plus.stl`): Gazebo
  color, depth (+points) and IR sensors publishing `/gripper_camera/{color,depth,ir}/image_raw`,
  `/gripper_camera/depth/points`, same names as the real `OrbbecSDK_ROS2` driver. Plugin topics are
  renamed with `<remapping>`. `ros2 run andesrobot_arm capturar_camara` saves one image per channel to
  `ros2_ws/capturas/` (gitignored). `arm_sim` also spawns `worlds/mesa_prueba.sdf` (arg `mesa`); its
  objects are links of one model, so it needs `<self_collide>true` or they fall through the table.
- Arm masses: gripper scaled from the CAD's steel to ABS (`gripper_mass_scale`); stepper motors and
  the gripper servo are fixed `motor_joint_N`/`gripper_servo_link` inertial-only links on the link
  before each joint. Sources in `docs/hardware/README.md` at the repo root (the docs there are
  gitignored third-party files).
- Gazebo tests leave orphan nodes (old `robot_state_publisher`s publishing another
  `robot_description`) that make the next `arm_sim` crash: kill every ROS process before relaunching.
- There is no arm driver for the real robot yet: arm control is sim-only.

**Arm kinematics** (`andesrobot_arm/kinematics.py`, pure numpy, details in `docs/BRAZO.md`):
`ArmKinematics` walks the processed URDF from tip to base, so it always follows the xacro (no DH
table). Chains: `arm_base_link_1 → gripper_tcp` (6-DOF) or `base_footprint → gripper_tcp` (7-DOF
with the lift, `base=LIFT_BASE`). `from_xacro()` reads the **installed** `andesrobot_description`
xacro with `lock_arm:=false` (with the arm locked every arm joint is `fixed`). IK is weighted damped
least squares with random restarts within URDF limits; `arm_ik_node` gives the lift weight 10 so
it prefers arm joints. Full-turn revolute ranges (the provisional ±π) wrap instead of clipping.
The wrist isn't spherical (joint_4 ∥ joint_6), so there's no closed-form IK.

`gripper_tcp` (0.102 m along −X of `link_6_1`, between the finger pads) is the IK tip and the
marker's frame. `andesrobot_arm/scripts/figuras.py` regenerates `docs/figuras/` (renders, dimension
drawings, workspace and `medidas.json`) from the URDF + STL meshes; rerun it after geometry changes. Arm limits are **provisional** (`arm_lower/arm_upper/arm_effort/arm_velocity`
properties: ±π, 20 N·m, 1 rad/s); don't use them for real hardware.

Known sim behaviors: IK has no collision checking, so poses that pass through the chassis/column/table
get physically blocked in Gazebo; `arm_controller` has goal tolerances (`arm_controllers.yaml`:
0.02 rad, lift 5 mm, `goal_time` 1 s, path tolerance off) so a blocked move aborts with
GOAL_TOLERANCE_VIOLATED and `arm_ik_node` logs it. `gripper_controller` has no tolerances on purpose
(fingers stall on a grasped object). With the provisional ±π limits a joint can't cross ±π, so a
target "on the other side" makes a near-full turn (measured: 6.08 rad / 12.2 s for a 0.2 rad change).
A Ctrl+C can leave an orphan `gzserver` holding port 11345 (next launch dies with exit 255):
`killall gzserver gzclient` or `./sim.sh stop`.

## Conventions worth keeping

- `sim.sh`/`robot.sh` print their header comment as help with `sed -n '2,Np'`: when adding an option,
  add its help line and update both the range and the comment that mentions it.
- Saving from RViz (File > Save) rewrites the `.rviz` file and deletes its comments.
- Robot docs use REP-103/105 (+X forward, +Y left, +Z up).
