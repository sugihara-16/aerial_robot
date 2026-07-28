# Ninja / MuJoCo development guide

## Scope

This guide covers only the ROS workspace rooted at `ros_ws/` (mounted as
`/ws` in the development container).  Inspect, build, test, and commit each
Git repository under `ros_ws/src` independently.

The main repositories touched by this work are:

- `src/aerial_robot`: Ninja/Beetle navigation, controller, launch files, and
  the aerial-robot-specific MuJoCo hardware interface.
- `src/aerial_robot_3rdparty`: generic `mujoco_ros_control` model generation,
  multi-robot execution, docking, and constraint-wrench extraction.
- `src/kalman_filter`: Kalman plugins used by the estimator during startup.

Do not assume that `ros_ws/src` itself is a Git repository.  Before editing or
committing, run `git status` in every repository listed above and preserve
unrelated user changes.

## Build environment

Build from the container workspace, not from an individual package:

```bash
cd /ws
source /opt/ros/noetic/setup.bash
catkin build
source /ws/devel/setup.bash
```

For a controller-only iteration, the usual shorter build is:

```bash
cd /ws
source /opt/ros/noetic/setup.bash
catkin build beetle ninja --no-deps
source /ws/devel/setup.bash
```

Changes in `mujoco_ros_control`, `aerial_robot_simulation`, message packages,
or `kalman_filter` require rebuilding their affected dependency chain; use a
full `catkin build` when uncertain.  A stale `devel/` is a plausible cause if
runtime behavior does not match source, so verify the executable/script path
with `rosrun --prefix 'readlink -f' ...` or rebuild before changing code.

Generated MJCF files live under the Ninja package's `mujoco/` output paths.
They are build products produced from URDF and YAML; edit the URDF or
`config/mujoco_model.yaml`, not the generated XML.

## Architecture and important files

### Open-chain wrench control

- `robots/ninja/src/ninja_navigation.cpp` maintains each module's desired
  pitch/yaw joint state and calculates forward-kinematic transforms.
- `robots/ninja/include/ninja/ninja_navigation.h` defines the full 6-D spatial
  wrench transform.  Its translation block is essential because moving the
  wrench reference point changes torque by the force moment arm.
- `robots/ninja/src/control/ninja_controller.cpp` estimates open-chain contact
  wrenches and propagates their compensation from a zero-compensation leader.
- `robots/beetle/src/control/beetle_controller.cpp` owns the common desired
  interaction-wrench input, synchronization, ordering, and accepted-target
  publication.

The implemented model intentionally targets an open serial chain:

- For `N` modules there are exactly `N-1` physical contact-wrench unknowns.
- Contact `i` is the wrench exerted by module `i` on module `i+1`, expressed at
  module `i`'s `yaw_connect_point` (`D_i`).
- Per-module estimated external wrenches and the common disturbance are
  transformed into consistent frames before a QR solve.
- There is no virtual end contact and no KKT constraint in this path.
- Joint-dependent transforms use `des_joint_pos_`.  This is deliberate for the
  current controller and must remain consistent throughout estimation and
  compensation.

Do not reduce the wrench transform to a 3x3 rotation.  Any change involving a
joint angle or connector frame must preserve both rotation and reference-point
translation for all six components.

### Desired interaction wrench

`ff_inter_wrench` is the desired internal/contact wrench setpoint, despite its
historical name.  It is incorporated by adjusting the pose controller's
integral compensation.  This prevents the ordinary pose integral term from
slowly cancelling a separately added feed-forward wrench.  Do not replace this
with a naive additive FF term without re-deriving and experimentally validating
the coupled integral behavior.

At a zero desired contact wrench, a remaining measured contact produces the
opposite-signed error and actively adjusts/unwinds the integral compensation;
zero is not merely “stop adding”.

Use the ordered helper for all six axes:

```bash
rosrun ninja ff_inter_wrench_cmd.py 1 Fx Fy Fz Tx Ty Tz
rosrun ninja ff_inter_wrench_cmd.py 1 0 0 0 0 0 0
```

Why the helper matters:

- Separate `rostopic pub -1` processes can overlap; TCPROS provides no global
  ordering between different publishers.
- The helper records a wall-clock ordering key before waiting for connections,
  repeats the same stamped command, and waits for both module controllers.
- The controller rejects stale, conflicting, non-finite, unstamped (when
  required), and invalid-contact commands under a mutex.
- A desired wrench persists by default.  `controller/ff_inter_wrench_timeout`
  is disabled when zero and may be enabled explicitly if watchdog semantics
  are wanted.

Observe what the controllers actually accepted on:

```bash
rostopic echo /ninja1/des_inter_wrench
rostopic echo /ninja2/des_inter_wrench
```

The misspelled `des_inter_wnrech` topic remains only for compatibility.  The
accepted desired wrench is not the same quantity as the physical constraint
wrench or `/ninja1/internal_wrench`.

### MuJoCo integration

Single Ninja launch:

```bash
roslaunch ninja bringup.launch \
  real_machine:=false simulation:=true mujoco:=true headless:=true
```

Two-module launch and docking:

```bash
roslaunch ninja two_module_mujoco.launch headless:=true
rosrun ninja mujoco_dock_two_ninja.py
roslaunch ninja switch_cmd.launch \
  left_edge_id:=1 right_edge_id:=2 switch_type:=1 real_machine:=false
```

Then arm/take off both modules using the normal Ninja command tool.  The order
is important: start the shared world, dock and verify the weld, publish the
assembly state, then fly.

The two-module launch uses one MuJoCo physics process and two namespaced
controller managers.  The model prefixes `ninja1/` and `ninja2/` identify each
robot inside the shared MJCF.  Do not run two independent MuJoCo worlds for a
physical weld between the modules.

Docking is performed through `/mujoco/docking/align_and_attach`.  The backend
waits for a MuJoCo update, atomically aligns the sites, zeros stale velocity and
solver warm-start state, writes the MuJoCo 2.3.x weld-data layout, activates the
weld, and verifies connector position/orientation.  The script also refuses to
teleport bodies that are still moving.  Keep these safety checks when extending
the docking workflow.

MuJoCo publishes weld-only constraint wrenches at:

```text
/ninja1/mujoco/contact_wrench
/ninja2/mujoco/contact_wrench
```

They are expressed in their respective connector-site frames.  The two sides
are an action/reaction pair, but direct numeric comparison still requires the
correct frame transform and sign convention.  Do not compare six raw numbers
from differently oriented frames.

The launch defaults to headless mode.  Only set `headless:=false` for visual
inspection.  `halt` should disarm the rotors and produce free fall in MuJoCo;
`land` remains a controlled descent.

## Validation checklist

For controller or docking changes, verify at least:

1. `git diff --check` in each changed repository.
2. A clean full `catkin build` result for the affected packages.
3. Repeat the launch → dock → switch → takeoff sequence several times.  A
   single successful flight is insufficient for startup-race fixes.
4. Confirm both accepted desired-wrench topics receive the same ordering stamp
   and six-axis value.
5. Send a nonzero command followed immediately by zero; keep an older publisher
   alive long enough to prove the stale command is rejected.
6. Exercise force and torque axes independently at approximately 0° and 90°
   joint configurations.
7. During a slow `/assembly/target_joint_pos` trajectory (roughly 90° over 10
   seconds), compare the estimated contact wrench with the transformed MuJoCo
   weld wrench, allowing only the expected action/reaction sign.
8. Check that no NaN/Inf actuator command, rank-deficient solve, large residual,
   or docking-verification error appears in logs.

When collecting quantitative results, record the exact launch arguments,
command stamps, frame IDs, joint target, time interval, sign/frame transform,
and calculation script.  Do not report an RMSE without retaining enough data to
reproduce it.

## Runtime cleanup

Use Ctrl-C on the owning `roslaunch` process and allow required child nodes to
exit.  Before starting another trial, check for leftovers:

```bash
rosnode list
pgrep -af 'roslaunch|rosmaster|roscore|mujoco_ros_control'
```

Stale ROS masters, controller spawners, or MuJoCo processes can create repeated
“waiting for” logs and make the next experiment nondeterministic.  Stop only
the identified processes; do not use broad destructive process commands.

## Editing and Git discipline

- Keep generic MuJoCo changes in `aerial_robot_3rdparty` and Ninja-specific
  behavior in `aerial_robot`.
- Treat `aerial_robot`, `aerial_robot_3rdparty`, and `kalman_filter` as separate
  histories.  Inspect and commit each one explicitly.
- Follow the repository's existing commit style (`[Ninja][Controller] ...`,
  `[Mujoco] ...`, or a short imperative Kalman message).
- Do not commit generated MJCF, catkin build products, ROS logs, or experiment
  bags unless the task explicitly requires them.
- Keep backward-compatible topics or parameters documented when renaming them.
- Preserve deliberate concurrency protection around callbacks and the control
  loop; ROS callback order is not a synchronization guarantee.
