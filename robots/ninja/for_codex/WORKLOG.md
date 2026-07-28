# Ninja open-chain / MuJoCo work log

Last updated: 2026-07-28

This log covers work performed in the ROS workspace and records design choices,
implemented changes, tests, and remaining caveats for future sessions.

## 1. Branch comparison and controller design review

- Read the Beetle and Ninja packages from the two original implementations and
  compared their open-chain interaction-wrench approaches.  Closed-loop chains
  and free-joint behavior were explicitly left out of scope.
- Found that the older open-chain propagation was structurally simple but did
  not transform wrenches correctly when modules had relative joint rotation.
  Rotation alone was also insufficient because changing a wrench reference
  point changes torque through the force moment arm.
- Reviewed the alternative KKT formulation.  Its underlying constrained method
  is viable, but the examined implementation mixed physical and virtual
  contacts and contained inconsistent boundary assumptions.  For an open chain,
  a KKT solve was not needed.
- Selected the open-chain design with correct joint-dependent spatial transforms
  based on `des_joint_pos_`.

## 2. Open-chain wrench estimation and compensation

Implemented in the Ninja navigation/controller and common Beetle controller:

- Added full 6x6 spatial wrench transforms including rotation and moment-arm
  translation.
- Calculated module COG, pitch connector, yaw connector, and assembly COM frame
  transforms from desired pitch/yaw joint positions.
- Defined contact `i` as the wrench exerted by module `i` on module `i+1`, at
  module `i`'s yaw/right connector.
- Formulated an `N`-module open chain with exactly `N-1` physical contact
  unknowns and solved the stacked module balance equations by rank-checked QR.
- Removed reliance on a virtual terminal contact and KKT boundary constraint.
- Estimated the common disturbance in a consistent assembly frame, transformed
  each module residual, and added finite/rank/residual checks.
- Anchored the leader's compensation wrench at zero and propagated desired-minus-
  estimated contact errors toward both ends using the same spatial transforms.
- Corrected COM/COG target pose and twist conversion paths and added a smooth
  trapezoidal desired-joint trajectory option for controlled morphing tests.
- Added mutex-protected snapshots for estimated and desired wrenches so ROS
  callbacks cannot race the control update.

The existing method of injecting the desired interaction wrench through the
pose integral compensation was retained intentionally.  A plain additive FF
would be counteracted over time by the pose I controller.  When the desired
contact wrench returns to zero while contact remains, the error changes sign and
the integral compensation actively readjusts; zero does not freeze the stored
term.

## 3. Single-Ninja MuJoCo support

- Extended URDF-to-MJCF conversion so Ninja can generate its MuJoCo model during
  the catkin build.
- Added `mujoco:=true` support to `bringup.launch`, a dedicated MuJoCo controller
  configuration, sensor-launch gating, direct servo relay, and headless launch.
- Preserved joint effort limits and fixed-link inertials, added configurable
  joints/sites/collisions, and decimated oversized STL collision meshes to stay
  below MuJoCo's mesh-face limit.
- Corrected servo target conversion so MuJoCo receives the actual signed/offset
  target used by the servo model.
- Added model-prefix-aware actuator, joint, site, sensor, mass, and rotor lookup.
- Added finite actuator-command checks and callback/control-loop locking.
- Published ground-truth pose plus world-frame linear and local-frame angular
  velocity, and connected ground-truth attitude mode to the controller.
- Tuned MuJoCo-specific roll, pitch, translation, joint, and damping parameters.
- Disabled the inappropriate global fluid model that produced excessive drag
  from dummy-link inertias.
- Initialized flight-control flags that could otherwise make `halt` enter a
  force-landing-like path.  MuJoCo `halt` now removes rotor force/free-falls,
  while `land` remains controlled descent.

## 4. Shared two-Ninja world and docking

- Added `two_module_mujoco.launch` with one shared physics process, two Ninja
  namespaces/controller managers, and prefixed model names.
- Extended model generation to instantiate multiple robots, add connector sites,
  and define an initially inactive weld equality.
- Added `mujoco_dock_two_ninja.py` to wait until both modules are stationary,
  align `ninja2/pitch_connect_point` with `ninja1/yaw_connect_point`, activate the
  weld atomically, wait for estimator settling, and verify the attachment.
- Moved alignment and weld activation into the MuJoCo update thread to eliminate
  a race with fast headless stepping.
- Cleared root velocity, acceleration, and solver warm-start data when
  teleporting/attaching so stale dynamics do not create a rebound impulse.
- Corrected the MuJoCo 2.3.x runtime `eq_data` weld layout: body-2 anchor,
  body-1 anchor, relative quaternion, and torque scale.  The former layout was
  the direct cause of modules jumping apart immediately after docking.
- Added adaptive damping for docking joints and excluded those joints from the
  direct-position relay so the weld sees physical actuator loads.
- Extracted only the selected equality rows from MuJoCo's constraint solver,
  mapped them through the site Jacobian, and published six-axis connector-frame
  wrenches for both sides on `/ninja1/mujoco/contact_wrench` and
  `/ninja2/mujoco/contact_wrench`.
- Made headless mode the normal path and made owning simulation/spawner nodes
  required so launch shutdown propagates cleanly.

## 5. Startup nondeterminism fixes

Repeated launch/dock/switch/takeoff trials sometimes succeeded and sometimes
diverged violently.  Several independent timing and initialization issues were
fixed:

- Delayed IMU `ACTIVE` publication until all Kalman dimensions and covariance
  initialization had completed.
- Serialized Kalman covariance/flag/dimension access and made repeated
  acceleration-bias enabling idempotent.
- Initialized previously indeterminate navigation and flight-control flags.
- Made duplicate arm and takeoff commands harmless outside their valid source
  states; stale teleoperation messages no longer restart transitions.
- Corrected `switch_cmd_pub.py` parameter lookup and switch semantics.
- Treated assembly state as state rather than a one-shot edge: publishers are
  latched, wait for expected subscribers, and repeat briefly so both controllers
  enter the same assembly mode.
- Rejected non-finite direct actuator inputs and synchronized direct pose/joint
  callbacks with the simulation update.

After these fixes, the full launch → dock → switch → takeoff sequence was
successfully repeated, and docking no longer caused an immediate launch impulse.

## 6. Desired interaction-wrench command ordering

Observed symptom: after sending `/ff_inter_wrench` repeatedly, a zero target
could be replaced later by an older nonzero command, especially during repeated
torque tests.

Root cause:

- Several short-lived `rostopic pub -1` processes were separate TCPROS
  publishers.  ROS preserves order per connection, but provides no total order
  across publishers.
- An older process could spend longer establishing connections and deliver
  after a newer process.  The controller ignored message timestamps and accepted
  whichever callback ran last.
- Shared desired/estimated wrench maps were also read and written from different
  threads without synchronization.

Fix:

- Added ordering metadata per contact and rejection of stale, equal-key
  conflicting, non-finite, invalid-ID, and (when configured) unstamped commands.
- Added `ff_inter_wrench_cmd.py`.  It captures wall time before connection setup,
  requires both module subscribers, and repeats one six-axis target using the
  same stamp/order key.
- Enabled stamped-command enforcement in the Ninja MuJoCo config.
- Published the effective accepted targets as latched
  `/ninja1/des_inter_wrench` and `/ninja2/des_inter_wrench`; retained the legacy
  misspelling `des_inter_wnrech` for compatibility.
- Added an optional wall-time command timeout, disabled by default so a target
  remains a persistent setpoint.

Validation performed:

- Both controllers accepted identical six-axis values and ordering stamps.
- A nonzero command followed by zero remained zero while an intentionally older
  publisher continued sending for several seconds; stale messages were rejected.
- The same ordering behavior was checked before and after docking/assembly mode.
- Unstamped raw commands were rejected under the MuJoCo configuration.

This issue was not caused by MuJoCo physics and was not caused by the deliberate
I-term compensation scheme.  MuJoCo made it easier to reproduce through rapid,
repeatable command experiments.

## 7. Wrench and moving-joint validation

- Compared MuJoCo's leader/follower weld wrenches as an action/reaction pair and
  checked the controller estimate using the required frame transforms and sign
  convention.
- Exercised desired force and torque components across all six axes.
- Tested desired interaction wrench behavior near 0° and approximately 90° joint
  configurations.
- Sent gradual `/assembly/target_joint_pos` changes (approximately 90° over 10
  seconds) and checked that the estimator continued using joint-dependent
  transforms during motion.
- Confirmed that zero desired wrench can reduce a remaining contact-wrench error
  through the compensation integrator rather than merely stopping accumulation.

No standalone numerical RMSE is recorded here.  Future quantitative claims must
retain the exact bag/time interval, frame IDs, action/reaction sign, transform,
joint state, and calculation procedure so the result is reproducible.

## 8. Current operating procedure

Build and source:

```bash
cd /ws
source /opt/ros/noetic/setup.bash
catkin build
source /ws/devel/setup.bash
```

Start, dock, and enter assembly mode:

```bash
roslaunch ninja two_module_mujoco.launch headless:=true
rosrun ninja mujoco_dock_two_ninja.py
roslaunch ninja switch_cmd.launch \
  left_edge_id:=1 right_edge_id:=2 switch_type:=1 real_machine:=false
```

Set or clear one contact target:

```bash
rosrun ninja ff_inter_wrench_cmd.py 1 Fx Fy Fz Tx Ty Tz
rosrun ninja ff_inter_wrench_cmd.py 1 0 0 0 0 0 0
```

Relevant observations:

```text
/ninja1/des_inter_wrench
/ninja2/des_inter_wrench
/ninja1/internal_wrench
/ninja1/mujoco/contact_wrench
/ninja2/mujoco/contact_wrench
```

`internal_wrench` is produced by the assembled controller path and therefore is
not expected before the assembly switch.  Accepted desired-wrench topics are
available for checking command delivery independently of physical response.

## 9. Remaining scope and caveats

- The implemented wrench solver is for an open serial chain.  Closed loops and
  free joints remain intentionally unaddressed.
- Transform inputs use desired rather than measured joint angles.  This was the
  selected design for the current work; large servo tracking error will appear
  as a model/estimation error.
- MuJoCo and controller contact wrenches use connector-local frames and opposite
  action/reaction conventions.  Always transform and sign-align before judging
  agreement.
- Retest repeated startup and all six axes after changes to launch ordering,
  callback threading, connector geometry, servo signs, or integral-compensation
  logic.
