# TEMRover Navigation — Research Log

Owner: Zeyu (Zephyr) Ren — Avenue 3, autonomous navigation.

What each file does and how they fit together: `architecture.zh.md`.

## Target spec (from Charter / Phase-1 docs)

| Quantity | Value | Source |
|---|---|---|
| Survey grid | 50 x 50 m, 5 m line spacing, 11 lines | Minutes 12/08 |
| Line tracking | within 10% of line spacing = **+/-0.5 m** | Charter objective |
| Survey speed | 1.5 m/s (6 m TEM resolution) | Minutes 19/08 |
| Scout 2.0 CAN limit | linear +/-1500 mm/s, angular +/-1500 mrad/s, 20 ms frame | Scout manual 3.3.3 |
| Tx geometry tolerance | dz 12 cm, dr 10 cm, tilt 10 deg | Phase-1 sensitivity study |

## Plan

- **RP-1** Closed-loop kinematic sim to replace the Phase-1 open-loop GPS simulator. Without it no controller claim is testable.
- **RP-2** Boustrophedon path generator from an area spec (no hard-coded waypoint lists), with forward-only turns respecting a minimum turn radius.
- **RP-3** Compare Stanley vs pure-pursuit on cross-track error against the +/-0.5 m spec.
- **RP-4** Sweep GNSS noise grade (standalone / SBAS / RTK) and heading noise to find the sensor requirement that meets spec. Decides whether NEO-M9N alone is sufficient.
- **RP-5** EKF fusion (GNSS + IMU yaw + wheel odometry), then ROS 2 node + `/cmd_vel` -> CAN bridge.

---

## 2026-08-21 — RP-1/2/3/4

### Rebuilt the stack

Phase-1 nodes (`gps_waypoint_navigator`, `lidar_obstacle_detector`, `integrated_navigator`,
`grid_turn_manager`, both simulators) removed. They had no heading feedback, two nodes writing
`/cmd_vel`, hard-coded waypoints and an open-loop GPS simulator, so none of their results were
testable. Replaced by:

| Module | Role |
|---|---|
| `geodesy` | ellipsoidal WGS84 <-> local ENU, replaces per-tick haversine |
| `survey_grid` | boustrophedon generator from an area spec, Pi-turn and teardrop turn |
| `path_tracking` | Stanley and pure-pursuit laws, curvature-limited speed policy |
| `rover_model` | unicycle with Scout command, acceleration and skid-slip limits |
| `pose_estimator` | EKF over east/north/yaw from wheel speed, IMU yaw rate, GNSS position and course |
| `obstacle_monitor` | nearest return in the forward sector |
| `survey_navigator` | sole `/cmd_vel` owner, state machine, digital e-stop |
| `scout_can_bridge` | `/cmd_vel` <-> SocketCAN 0x111 at 20 ms, republishes 0x221 feedback |
| `rover_simulator` | closed-loop stand-in, ray-cast scans against obstacle discs |

### Turn geometry (RP-2)

Hypothesis: a semicircle U-turn is enough. **False** — it only closes when the line spacing is at
least 2R. With 5 m spacing that caps the turn radius at 2.5 m, which the Tx/Rx train will not hold.
Added the teardrop turn, which swings away by `arccos(spacing / 2R)` first; solved in closed form,
no root finder. Verified: max curvature equals 1/R exactly, line spacing exact to 1e-3 m.
Cost of a larger radius: path length 623 m at R = 2 m against 810 m at R = 4 m, **+30% survey time**.

### Controller and receiver grade (RP-3, RP-4)

50 x 50 m, 5 m spacing, R = 2 m, cruise 1.5 m/s, 20 Hz control, gains selected per case by
lowest p95. Error is scored on survey lines only, against true position.

| Controller | GNSS | rms | p95 | max | yaw rate rms | duration |
|---|---|---|---|---|---|---|
| stanley | RTK | 0.018 | 0.031 | 0.105 | 0.246 | 460 s |
| stanley | SBAS | 0.283 | 0.495 | 0.764 | 0.258 | 466 s |
| stanley | M9N standalone | 0.683 | 1.194 | 1.702 | 0.352 | 468 s |
| pure pursuit | RTK | 0.018 | 0.032 | **0.101** | **0.202** | 457 s |
| pure pursuit | SBAS | 0.277 | 0.473 | 0.719 | 0.288 | 460 s |
| pure pursuit | M9N standalone | 0.684 | 1.128 | 1.454 | 0.211 | 458 s |

- **Receiver grade decides the spec, not the controller.** RTK passes +/-0.5 m with 5x margin;
  SBAS sits on the limit and busts it in the turns; the purchased standalone NEO-M9N misses by 3x.
  **Action: the navigation module needs an RTK correction source.**
- Tuned Stanley and pure pursuit are indistinguishable on error. Pure pursuit needs 18% less yaw
  rate for the same accuracy, which matters directly for cart tilt and vibration noise, so it is
  the default in `survey_navigator`.
- An untuned comparison first showed pure pursuit 3x ahead; that was a gain artefact, not a result.
- Residual error concentrates at turn exits, never on the straights — see `results/tracking_*.png`.

### Obstacle hold

First attempt used a fixed 1.5 m danger distance: the rover drove straight through the obstacle,
because stopping from 1.5 m/s at 0.5 m/s^2 needs 2.25 m. Second attempt derived the threshold from
the instantaneous command, which chattered every cycle once the command was zeroed. Kept: threshold
is `standoff + braking distance of the planned speed`, independent of the hold state. Rover now
stops 0.95 m short and holds.

### Next

- RP-5: replace the commanded-speed prediction with the CAN 0x221 feedback on hardware, bench the
  bridge against `vcan0`, then a real Scout with the transmitter in command mode.
- Obstacle response is hold-only. Re-planning around an obstacle is not implemented.
- Terrain is flat and slip is a single constant; the Phase-1 Simscape work suggests both matter.

## 2026-08-21 — Gazebo Fortress

### Choice of simulator

Gazebo Classic was ruled out: Humble has no ROS 2 GPS sensor plugin for it (nothing in apt,
`hector_gazebo_plugins` is ROS 1 only), and a GPS grid survey cannot be simulated without one.
Fortress ships `NavSat`, `Imu` and `GpuLidar` as systems and is the officially paired release for
Humble. Installed `ignition-fortress`, `ros-humble-ros-gz`, `ros-humble-robot-localization`.

AgileX publishes the Scout V2 model on the `humble` branch of `ugv_gazebo_sim`, with real geometry
(track 0.583 m, wheel radius 0.1646 m, mass 40 kg). Vendored as `scout_description` and its Gazebo
Classic block replaced with Fortress systems. **Their upstream `diff_drive` had
`wheel_diameter 0.08` and `wheel_separation 0.451` against the model's own 0.329 and 0.583** — a
4x odometry scale error that fails silently. Corrected.

### Terrain

`experiments/make_terrain.py` builds the heightmap from the Phase-1 terrain spec: swells from the
stated wavelength and amplitude envelopes plus an ISO 8608 Class D roughness field, band-limited to
wavelengths above 1 m because a 0.33 m wheel cannot follow shorter ones. Result: 1.38 m
peak-to-peak, 2.6 deg slope rms, 18.4 deg worst case.
Fortress heightmaps must be **8-bit** — a 16-bit PNG produced 260k `Image: Coordinates out of range`
errors and no terrain. Quantisation at an 8-bit, 2 m scale is 1.7 mm rms, negligible against 45 mm
roughness.

### 2D LiDAR reads the ground as an obstacle

First full run: the rover stopped 14 m in and never resumed, flipping between SURVEYING and HOLDING
every few cycles. The scan showed a uniform 3.0 m return across the whole forward arc — not an
object, but the ground. A LiDAR fixed 0.49 m above the wheels sees terrain at about 3 m as soon as
the rover pitches ~9 deg, and this terrain reaches 18 deg. **Flat-ground simulation cannot surface
this; it is a design problem for the obstacle-detection Must-have.**

Fixed by projecting each beam with the IMU roll and pitch and discarding returns whose endpoint
falls below `minimum_obstacle_height` above the sensor plane. After the fix the rover drove to the
planted obstacle at x = 18 m and held 3.48 m short of it, matching the standoff plus braking
distance, with cross-track error 0.23 m on real terrain. Two state changes over the run instead of
continuous chatter.

Open: mounting the LiDAR level does not solve this on its own — a nose-down pitch on a downslope
still puts the beam into the ground. Worth comparing against tilting the unit up a few degrees.

### Interface change

Wheel feedback moved from a bespoke `TwistStamped wheel_velocity` to `nav_msgs/Odometry` on `odom`,
which is what Gazebo's DiffDrive publishes and what `robot_localization` expects. `scout_can_bridge`
now dead-reckons the 0x221 feedback into the same message.

### Dropped the lightweight simulator

`rover_simulator` and `ros_smoke_test` removed on request: two simulation paths of different
fidelity is one too many to keep honest, and Gazebo covers everything the fake sensors did.
What remains is one simulated path (Gazebo Fortress) and one real path (`scout_can_bridge`).
`launch/scout_navigation.launch.py` split into `gazebo.launch.py` and `hardware.launch.py`.
The offline harness in `experiments/` is kept — it is not a third simulator but the parameter
sweep behind the RTK decision, and it runs a full survey in seconds where Gazebo needs 8 minutes.

## 2026-08-21 — Obstacle detour (not achieved)

Added `worlds/flat_field.sdf`; the heightmap world became `worlds/rough_field.sdf` and the launch
file takes a `world` argument, **flat by default**, so the detour could be developed without the
terrain perception problem in the way.

**A 2D LiDAR cannot separate a rising slope from a wall.** On the rough world the rover stopped
3.8 m in. Diagnosis: pitch +8.9 deg nose-up while climbing, and the range profile across the sector
was a smooth 2.40 -> 2.08 -> 2.34 with a **maximum step of 0.081 m between adjacent beams**. That is
ground. The IMU-attitude ground filter cannot remove it: with the nose up the beam points upward and
the return genuinely sits above the sensor plane. Added a discontinuity classifier
(`standing_objects`) — terrain spans the sector smoothly, an object subtends a limited angle.
The first version of the rule ("bounded by a step on both sides") discarded the only object in view
on flat ground, so the criterion became angular width.

**World-file bug:** the obstacle cylinders were posed at z = 1.5 with length 1.2, floating between
0.9 m and 2.1 m while the LiDAR sits at 0.39 m — the rover drove underneath them. They only worked
on the rough world because the terrain lifted them into the scan plane. Now grounded.

**The reactive lateral detour does not converge.** Six iterations, each fixing a real defect and
exposing the next: target collapsing when the obstacle left the sector (fixed by latching);
deadlock once stopped, since a non-holonomic rover cannot translate sideways (added an in-place
recovery turn); runaway to y = -13.7 m; a double-counted offset from using the commanded rather than
the measured cross-track; detection at 8 m leaving only 3 s to move; runaway to y = -4.09 m.

The problem is structural, not parametric: a latched lateral target, a stop-and-turn recovery, and
pure pursuit chasing an offset line form a hybrid system with no single well-defined path. The right
design is to **plan a detour path into the survey path** so the tracker always follows one defined
path and no recovery mode is needed.

Kept the code, disabled by default (`maximum_detour: 0.0`, `recovery_turn_rate: 0.0`). Verified on
the flat world: **maximum cross-track error 0.001 m along the line, stopping 3.35 m short of the
obstacle**, matching standoff 1.0 + braking 2.25.

## 2026-08-28 — CAN physical layer verified on the rover

The half the loopback test could not reach. CANable `1d50:606f` enumerated, `can0` up at 500 kbit/s,
**421 frames in 5 s with 0 errors and 0 drops**. Battery 25.8 V off `0x0102`, no faults.

**Control mode is what actually gates command authority.** Mapped empirically from `0x211` byte 1:
`0x00` standby, `0x01` CAN command, `0x02` serial, `0x03` remote control. The transmitter outranks
CAN entirely — while it holds the chassis, `0x421` is ignored and no `0x111` frame has any effect.
S1 to the top hands control over, and the mode then stays at `0x01`.

Rewrote `scout_can_bridge.py` around this: from standby it asks for command mode, in command mode it
sends motion frames, and in any other mode it sends nothing. It requests authority but never fights
the transmitter for it.

**Verified with zero commands first: 271 `0x111` frames of `0000000000000000` accepted, `0x221`
stayed all zeros — the rover did not move.** The command path was proven up to the motion boundary
without the vehicle moving at all.

## 2026-08-28 — First motion on the real rover

**The chassis ignored a 0.1 m/s command.** Control mode read back as CAN command and the bridge ran,
but `0x221` stayed all zeros and only **10 odometry samples** arrived in 3.5 s. Two causes stacked:
the test published `cmd_vel` on every executor iteration, starving the bridge's 20 ms send timer, so
only a handful of `0x111` frames reached the bus; and 100 mm/s is 6.7 % of the `+/-1500` full scale,
inside the motor controller deadband. Fixed by publishing the command once — the bridge already
caches it and reruns it on its own timer — and by driving at the real survey speed from
`config/survey.yaml` rather than a token value.

**Result: the rover drove.** Commanded `cruise_speed` 1.5 m/s, bounded by dead-reckoned distance:
**2.009 m travelled, peak 1.500 m/s, 183 feedback samples**. `/odom` is decoded from the chassis's
own `0x221` frames, not from the command, so this is the vehicle's own report. The full chain
`/cmd_vel -> scout_can_bridge -> 0x111 -> Scout 2.0 -> 0x221 -> /odom` is now verified on hardware.
Kept as `experiments/rover_nudge.py`.

**Open:** heading and cross-track were never exercised — this was a straight run with no GNSS. The
mean of 0.794 m/s is the acceleration ramp, not a tracking error.

## 2026-08-28 — 50 m line and two turns on the real rover

**No GNSS and no IMU are fitted**, so the navigator could not run at all: `has_fix` never became true,
and `yaw_rate` came only from `on_imu`, leaving it pinned at zero. A zero yaw rate means the estimator
never integrates heading, so `rotation_settled` can never fire — the rover would have spun in place
until something killed it. The Scout already reports yaw rate in `0x221`, which the bridge publishes on
`/odom`, so a `dead_reckoning` parameter now declares the fix present and routes odometry through
`on_chassis_odometry`, which takes the yaw rate from chassis feedback. Not a fallback — the source is
chosen once at construction.

**Result: the mission executed on hardware.** 8 m alignment run, heading solved at +0.1 deg, 41 steps
generated, then 50 m line, 90 deg turn, 5 m cross link, 90 deg turn — stopped at step 4 by design in
72 s. Final dead-reckoned pose east +49.94 m, north +5.82 m, heading +3.112 rad against an expected
50 / 5 / pi. Both turns entered and terminated on their own; the 150 s bound was never reached.

**These numbers are the rover's own estimate, not ground truth** — with no GNSS there is nothing to
check them against, and skid-steer wheel odometry understates turn slip. This run validates the state
machine and the command path on hardware, not tracking accuracy. Kept as `experiments/rover_mission.py`.

**Simulation comparison did not run:** `ign gazebo` exited shortly after spawn while the navigator
stayed alive publishing into nothing, so the recorder logged a stationary rover and reported a
meaningless `cross-track rms 0.000 m`. Unresolved.

## 2026-09-04 — Obstacle detour planned into the path (simulation)

**Hypothesis.** The earlier reactive detour never converged because it had no single path to track.
Planning the detour *into the mission* — sideways ramp, hold beside the object, ramp back, then the
rest of the line, spliced in as ordinary drive steps — lets the existing tracker follow one defined
path and needs no recovery mode. Object size and shape are not assumed: the along- and cross-track
extent of the LiDAR returns in the line frame is what gets planned around.

**What was built.** `obstacle_monitor.py` now uses the full 360 deg scan, filters ground by IMU
attitude as before, and returns world-frame points. `detour.py` plans the three-part detour and
picks the side. The navigator checks a *swept corridor* (rover half width + clearance) along every
remaining drive step, holds if it is blocked within standoff + braking distance, replans if blocked
within a 10 m lookahead, and during a spot turn holds when anything lies inside swing radius +
clearance. Consecutive drive steps flow through without stopping and decelerate towards the end of
the whole run, not each 1 m segment. Old sector-only monitor and the disabled reactive detour removed.

**Failure 1 — the rover flip-flopped sides.** Around the cylinder it planned right (-1.56 m), then
left (+1.27 m), then right (-1.67 m), swinging across the line with the yaw command pinned at the
1.5 rad/s cap. It still cleared the object by 0.57 m, but the S-curve is unacceptable for towed
carts. Scan latency was measured at 13 ms and ruled out. Working back from the three offsets showed
the returns had rotated ~8 deg clockwise about the rover between plans.

**Cause — GNSS lever arm.** The antenna sits 0.30 m behind base_link. When the rover yaws at
omega, the antenna carries a sideways velocity 0.30 omega, so GNSS course over ground deviates from
heading by atan(0.3 omega / v): 8.2 deg at the 0.72 rad/s a cosine ramp reaches at cruise. The EKF
trusted course to ~2 deg at that speed and dragged the heading against every turn. Offline: 5.7 deg
heading error during a 0.5 rad/s turn without compensation, 0.00 deg with it.

**Change.** `PoseEstimator` now models the antenna: position update predicts the antenna location
from base_link + lever arm (Jacobian includes the heading term), course update removes the swing
component omega x arm first. `gnss_offset: -0.30` from the URDF. Side choice also gained hysteresis:
of the two clearing offsets, take the one nearer the rover's *current* lateral position, so a detour
in progress keeps its side. Ramp length derived from a lateral acceleration limit (1.0 m/s^2) rather
than a fixed 4 m. Navigator publishes `estimated_pose` so the recorder can measure heading error
against truth.

**Result.** Single plan around the cylinder, body clearance 0.71 m (was 0.57). Corner test: a thin
pole 0.95 m beside the first corner, outside the drive corridor but inside the swing radius — the
rover completed the line, entered HOLDING before turning, and resumed the instant the pole was
removed (the recorder plays the bystander and removes it after 3 s). Box 2.0 x 0.6 m centred on
line 3 planned at +2.07 m from the first 12 returns at the lookahead edge, widened to +2.59 m on the
same side once 45 returns showed the full face; clearance 1.00 m.

**Failure 2 — a single +/-1.5 rad/s double spike beside the box.** Blamed first on a heading kink
at the replan splice; replacing the cosine ramp with a cubic Hermite that starts at the rover's
current slope cut the offline splice kink from 12.6 deg to 0.6 deg but the spike stayed.
Instrumenting the command showed the real cause: the ramp-*in* was built with the same length as the
ramp-*out*, and on a replan the ramp-out shift is tiny because the rover is already near the new
offset — so a 2.59 m return to the line was squeezed into ~1 m, a segment heading of -67 deg, and
the rover swung to -88 deg and 1.31 m below the line recovering. Each ramp is now sized from its own
shift; offline the return is 5.91 m and the steepest segment 32.5 deg, the cubic's own peak slope.

**Pending.** The full simulation run confirming the fixed ramp-in was aborted when a second
simulation was started on the same ROS domain mid-run (two navigators on `/cmd_vel`, clock reset
sent the first back to WAITING_FOR_FIX). To be rerun. Numbers to beat: yaw rate while driving
max 1.50 / rms 0.20 rad/s; expected max well under 1.0.

**Lessons.** Test-fixture geometry matters: any static object close enough to trigger the swing
hold is also inside the arrival corridor unless it is thin. A launch piped through `grep` without
`--line-buffered` hides log lines for minutes. `standing_objects` angular limit raised to 1.57 rad —
a wall fills a quarter turn up close, a slope fills half — but the flat world does not exercise it.
Towed carts will sit inside the swing radius behind the rover; the turn check must exclude them
before `carts:=true` runs again.

## 2026-09-09 — South Lawn survey points, bare rover and towed payload

**Site coordinates.** `resource/SOUTHLAWN2020.txt` gives 14 grid points plus an RTCM base, in
GDA2020 / MGA zone 55 (easting, northing). Converted to geographic with a Transverse Mercator inverse
on GRS80 (`resource/south_lawn_geographic.csv`). The grid is 30 x 30 m on a 5 m spacing, start corner
GS0013 at -37.79873428, 144.96013018; the lines run along MGA grid east, which is 1.25 deg clockwise
of true east here, so `grid_bearing = -0.02182 rad`.

**Config-driven site, no hard-coding.** New `config/south_lawn.yaml` layers origin, bearing, size and
`cruise_speed 0.95` over `survey.yaml`. Added `anchor_to_origin`: the mission is laid on the surveyed
corners rather than wherever the rover starts. The world files became xacro templates
(`flat_field.sdf.xacro`) taking the GPS origin as arguments, so the simulated fix lands where the
config expects. The navigator gained `estimated_pose` output and a `towed_length` filter so the LiDAR
ignores the rover's own carts.

**Bare rover: pass.** 7 lines, 6 spot turns, reached COMPLETE. Cross-track rms 0.053 m, max 0.256 m
against the 0.5 m spec — the same clean result as the earlier 50 m run, now on the real site geometry.
`results/south_lawn_rover.png`.

**Towed payload: the spot turn is the problem.** With the Tx and Rx carts (Rx rear wheels 7.76 m
behind base_link), two failures compounded. First, the 8 m train drags the rover sideways on the
straight, and the alignment run mis-solved heading by +12.5 deg; adding gyro damping to the alignment
drive straightened it. Second and unresolved: the in-place 90 deg turn is violently underdamped. The
instrumented turn showed heading swinging -27 / +38 / +8 / +57 deg with the yaw rate reversing
+1.5 / -0.7 / +0.4 rad/s over ~30 s before converging; an earlier GUI run did not complete the first
turn in 11 minutes. `rotate_command`'s PD gains are tuned for the bare rover's inertia — the train
multiplies rotational inertia and stores then releases energy, so the same gains ring. The carts get
whipped through tens of degrees, which the sensor payload cannot tolerate.

**Direction, not yet built.** Do not spot-turn with the payload. Use a wide-radius arc turn so the
train follows a curve rather than pivoting — the arc turns already exist in `survey_grid`
(`teardrop_turn`, `pi_turn`, `turn_radius`), but `generate_survey_mission` and the executor currently
only do spot turns. A payload survey needs the arc path wired into the executor, or at least retuned
damping and a lower `rotation_rate` for the carts. `results/south_lawn_payload.png`.

## 2026-09-09 — Multi-obstacle verification of the detour planner

**Layout moved to config.** `config/obstacles.yaml` now holds the test set — name, model, pose,
footprint half-size and a `bystander` flag. Both `gazebo.launch.py` and `experiments/detour_run.py`
read it, so the world and the scoring can no longer disagree. Seven obstacles over three lines:
three in sequence on line 1 (cylinder at +0.3, cylinder at -0.4, 2 m box at +0.6), a 2 m box centred
on line 2, a cylinder offset and a 2 m box centred on line 3, plus the thin corner pole as bystander.

**Side selection matches theory.** Required clearing offsets computed from the footprints against the
planned offsets:

| Obstacle | left needs | right needs | planned | chose |
|---|---|---|---|---|
| line1_near_centre | +2.05 | -1.45 | -1.42 | nearer side |
| line1_second | +1.35 | -2.15 | +1.38 | nearer side |
| line1_wide | +2.95 | -1.75 | +3.13 | left, since the rover was still at ~+1.0 from the previous detour |
| line2_centre | +2.35 | -2.35 | -2.16 | symmetric, broken by current offset |
| line3_wide | +2.35 | -2.35 | -2.19 | same |

**Result: all seven cleared.** Minimum body clearance, swing radius included:

| Obstacle | clearance |
|---|---|
| line1_near_centre | +1.10 m |
| line1_second | +0.77 m |
| line1_wide | +0.86 m |
| corner_swing | +0.31 m (hold, scored only while it stood) |
| line2_centre | +0.51 m |
| line3_offset | +0.50 m |
| line3_wide | +0.54 m |

**The ramp-in fix is confirmed.** Yaw rate while driving max **0.98 rad/s**, rms 0.21, against
**1.50 saturated** before each ramp was sized from its own shift. The steering trace no longer touches
the CAN cap. This was the run aborted by a domain collision on 2026-09-04; it now passes.

**Two scoring bugs found in the recorder, both mine.** A stale hardcoded `OBSTACLES` dict sat *below*
the new config-driven one and shadowed it, so the first run scored the rover against three phantom
obstacles at last week's positions and reported -0.40 m "collisions" where the world was empty.
Second, the removed bystander kept being scored after removal, turning the rover's legitimate pass
through that space into another false negative. Fixed: one definition, and the bystander is scored
only over samples taken before it was removed. **Neither bug touched the navigator — the avoidance
was correct in both runs; only the measurement lied.**

## 2026-09-15 — Smaller margins, side commitment, and a second stack on Nav2

**Complaint.** Detours were too wide (3.13 m for a 2 m box) and consecutive obstacles compounded.
Decomposed: 1.0 m of that was margin (clearance 0.5 + hysteresis 0.5), the rest was the side rule
"nearer to the rover's current lateral position" dragging the second detour to the far side because
the rover had not yet returned from the first.

**Change 1 — margins.** `obstacle_clearance` 0.5 → 0.3, `obstacle_hysteresis` 0.5 → 0.2, so the
planning margin is 0.5 m and the corridor half width 0.65 m. Predicted offsets halved.

**Change 2 — side rule, three iterations, each fixed by data.**
(a) "Smaller offset from the line, no memory": the box centred on line 2 flip-flopped every cycle
(−1.49 / +1.50 / −1.49 …) because a symmetric object is a tie and the measured extent jitters.
(b) "Keep the committed side on a near-tie (< margin)": still flipped, the difference sat at 0.48
one cycle and 0.50 the next. Also the committed side was inferred from the widest planned point,
which is the rover's own position mid-return, not the plan's intent — replaced with an explicit
`detour_offset` reset at every run boundary.
(c) "Switch only if it saves more than margin + rover width (1.2 m)": the far side of any object is
partly occluded so it always looks shorter (measured −1.19..+0.64 for a −1.0..+1.0 box); a switch
threshold below the occlusion error chases the unseen side. A body crossing is the physical cost of
switching, so that is the threshold. Kept.

**Result (7 obstacles, headless).** One side per object, widened monotonically as returns came in,
no flips. Max lateral offset 1.98 m (was 3.13), body clearance +0.26..+0.46 m across all seven,
yaw rate while driving max 0.63 rad/s (was 0.98; 1.36 during the flip-flop runs), rms 0.17.
The corner pole at 0.95 m no longer triggers the swing hold (radius now 0.88 m); to demo the hold,
move it to ≤ 0.85 m. `results/multi_obstacle_small_margin.png`.

**Second stack, kept separate.** Nav2 1.1.20 installed. `launch/nav2.launch.py` starts the same
world with `navigator:=false`, a `controller_server` running MPPI, a local costmap fed by `/scan`
whose inflation radius (0.65 m) *is* the safety margin, and `nav2_line_follower`: it owns the mission
and the spot turns, publishes odom→base_link, and hands each straight run to `FollowPath`. Config in
`config/nav2.yaml`, PathAlign weighted high so the rover hugs the line. Nothing of the existing
stack was removed; `gazebo.launch.py` gained a `navigator` argument. Not yet validated end to end.

## 2026-09-15 — Nav2 stack brought up and measured against the same seven obstacles

**Four wiring faults, each found from the log, none a design issue.**
1. Five stale `parameter_bridge` processes from earlier runs were still publishing `/clock`; the TF
   buffer logged 13,148 "jump back in time" clears in 70 s. Our own stack never used TF, so this had
   gone unnoticed. Killed; the cleanup pattern now includes the bridge.
2. Gazebo stamps scans with frame `temrover/base_link/lidar`; the URDF frame is `lidar_link`. The
   costmap dropped every scan. Fixed with `<ignition_frame_id>` on the sensor.
3. MPPI alone aborts with "Resulting plan has 0 poses" when the given path runs through an obstacle
   — it is a tracker and expects a collision-free path. Added `planner_server` (NavFn) on a global
   costmap and a two-node behaviour tree (replan at 1 Hz, follow), driven through `NavigateToPose`.
4. Bringup raced Gazebo (configure timed out), the default through-poses tree needed a recovery
   server we do not run, and a 70 m rolling global costmap put the 50 m line end off the map.
   Fixed with a 10 s start delay, `navigators: [navigate_to_pose]`, and a fixed 70 × 70 m window.

**Result.** Lines 1–2 and both cross links reached (`status 4`), all seven clearances positive
(+0.47 .. +3.05 m), yaw rate while driving max 0.28 rad/s, rms 0.05 — far smoother than our planner.

**But it does not survey.** After the first cylinder the path never returns to the line: it holds
−1.7 m for the remaining 35 m of line 1, sits at +7.3 m on line 2, and heads for y = 17 m on line 3.
A point-goal planner minimises distance to the line *end*; from an offset position the shortest
route is a diagonal, and nothing in the cost encodes "stay on the line". Our planner exists to
encode exactly that, and returns to the line within one ramp length.

**Kept, not adopted.** `nav2.launch.py` / `config/nav2.yaml` / `nav2_line_follower.py` stay as the
second stack. Making it survey-grade needs a lane-preference costmap layer (cost rising with
distance from the current line) or path-constrained local replanning — a real piece of work, not a
parameter. Decision for the team: our planner meets the survey objective now; Nav2 offers smoother
control and a maintained codebase but needs that layer before it can replace ours.

## 2026-09-16 — Leave the line late, return early

**Complaint.** Even with the smaller margins the rover left the line as soon as it *saw* an object
(10 m ahead) and only rejoined a full ramp after it: about 15 m off-line for a 0.8 m cylinder.
The ramp-out began at the rover's current position (`min(object − margin, now + L)`).

**Change.** The ramp now begins at `max(now, object − margin − L)`: the rover stays on the line
until the ramp has to start. Start slope is the rover's current heading only when the ramp starts
immediately, zero otherwise. Nothing else changed. Geometry: 14.9 m → 9.0 m off-line per cylinder.

**Result (7 obstacles).** All clearances +0.41 .. +0.63 m, yaw while driving max 0.71 rad/s
(rms 0.19), one plan side per object, the track shows a compact bump around each obstacle with
straight line either side. `results/multi_obstacle_small_margin.png`. The recorder now scores
off-line distance on the lines only (cross links excluded) and saves the raw track as `.npz`, so
future changes can be compared offline without a rerun.

**Nav2, one more finding.** The Route Server exists on Humble (`ros-humble-nav2-route` 1.1.20) —
it follows a predefined line network, which is closer to a survey than point goals, but an object
*on* an edge is still handled by stop or free-space replanning, not by a return-to-line detour.

## 2026-09-16 — Return to the line between obstacles; Nav2 measured and set aside

**Decision.** Our planner stays the field solution; Nav2 is kept in the repo but not pursued. The
overlay (figure deleted in the results clean-up) shows why: on the same seven obstacles Nav2 (NavFn + MPPI)
was off the line for 82.6 % of its line distance, holding −1.2 m / −2.0 m / +2.0 m after each object
because a point-goal planner has no term for "stay on the line". Ours: 28.6 %.

**Then five revisions to ours, each driven by the overlay or the log.**
1. Ramp-out starts at `object − margin − L` instead of at the rover: no more leaving the line the
   moment an object is seen. Per cylinder 14.9 → 9.0 m off-line by geometry.
2. `detour_speed` 1.0 m/s while off the line: ramps scale with speed (3.6 → 2.4 m). Yaw peaks rise
   to ~1.1 rad/s because lateral acceleration is held constant (ω = a/v) — a real trade, still under
   the 1.5 cap.
3. Between consecutive objects the path now returns to the line first (ramp back, straight, ramp
   out) when there is room; before, it held the current offset across the gap.
4. Planning only around the *nearest* return group (cut at the first along-track gap wider than
   2·margin). A pole 9 m past a box had been merged into the box's extent and produced a −0.08 m
   plan that drove the rover into a hold in front of the box.
5. Replan before the hold decision, and let ramp-less replans extend the hold along the line
   instead of being skipped: the LiDAR sees only the front face, the ramp-in was planned into the
   unseen rear corner, and the hold chattered 6× beside each box. Down to one hold in the run.
   Stop hysteresis separated from the planning margin (`hold_hysteresis` 0.5).

**Result (7 obstacles).** All clearances +0.29 .. +0.74 m, all three lines completed, off-line
distance 63.9 → 59.4 m on 213 m of line, one hold. (figure superseded by `results/avoidance_before_after.png`).
The residual off-line distance is now set by physics: two ramps of L = √(6·D·v²/a_lat) plus the
object and 2 × 0.5 m. The only lever left is `detour_lateral_acceleration`, which the towed carts
must decide on the real machine.

## 2026-09-16 — Nav2 removed; two rules deleted

Nav2 files, launch switch and entry point deleted (apt packages left installed). Audit of the
avoidance logic: 210 lines, nine rules. Two were tuning debt rather than physics and went:
the "switch sides if it saves more than a body crossing" clause (replaced by: first plan picks the
smaller side, that side is kept for the run) and the off-line speed cap `detour_speed` (its ramp
saving was 7 % of off-line distance for a yaw peak of 1.1 rad/s). Seven rules remain, five of them
definitions (corridor, nearest group, ramp geometry, return-between-objects, replan-before-hold) and
two two-line guards.

**Result (7 obstacles).** Zero holds, one side per line, yaw max 0.73 / rms 0.19 rad/s, all
clearances +0.36 .. +0.64 m, three lines plus part of the fourth in the window (238 m on lines).
Off-line 65.0 m (27.3 %) — the same fraction as before the speed cap, as predicted, with the
steering back where the carts can follow. (figure superseded by `results/avoidance_before_after.png`).

## 2026-09-16 — results/ clean-up and before/after comparison

Deleted the intermediate runs, single-run videos and superseded overlays from `results/`; what
remains is one `avoidance_before` and one `avoidance_after` run (`.npz` track, `.png` figure), the
side-by-side video and the path comparison figure.

**Caveat.** The pre-revision code no longer exists, so "before" is the current code run with the
old wide planning margin (`obstacle_clearance` 0.5, `obstacle_hysteresis` 0.5 → 1.0 m corridor
margin), which reproduces the old footprint but not the old side-switch rule. "After" is the
simplified planner with its shipped parameters (0.5 m margin, late ramp-out, return between
objects, one side per line).

| run | off-line distance | on lines | fraction | yaw peak |
|---|---|---|---|---|
| before | 76.2 m | 216.7 m | 35.2 % | 1.50 rad/s |
| after | 63.4 m | 223.1 m | 28.4 % | 0.72 rad/s |

`results/avoidance_before_after.png` (track + cross-track offset, both runs on the seven obstacles)
and `results/avoidance_before_after.mp4` (overhead camera, 226 s, both runs side by side, made with
`experiments/compose_video.py`).

## 2026-09-16 — Operator dashboard

**Choice.** Three candidates were weighed for live monitoring of the track and a camera: Foxglove
(desktop app, generic panels, needs foxglove_bridge and a layout file), Vizanti (browser, hosted on
the robot, rviz-style 2D view with mission tools for Nav2 goals) and a purpose-built page over
rosbridge. The survey has exactly four things an operator watches — the lines, where the rover is
against them, what the camera sees, and whether it is stopped — so a single 250-line page beats
configuring a generic tool, and it runs from the rover with no internet (`roslib.min.js` vendored).

**Pieces.** `launch/dashboard.launch.py` starts rosbridge (9090), web_video_server (8080, MJPEG)
and the page (8000); `usb_camera:=true` adds `usb_cam` on `/dev/video0` for the real rover. The
navigator now publishes `survey_status` (JSON: state, line n/N, speed, heading, cross-track,
detour offset, distance to the first blocking return) each control step. The simulated rover got a
forward camera (`front_camera/image`, 640×360, 15 Hz).

**Page.** Map: planned lines (including spliced detours, from `survey_path`), driven track
coloured on/off line at 0.2 m, lidar returns in world frame, rover marker with heading. Camera
panel 16:9. Status cells, GNSS/rosbridge dots, and an emergency-stop button that publishes
`/emergency_stop` — the same topic the navigator already honoured. Verified against the simulation
with headless Chrome captures.

**Addendum — the reference view.** The LinkedIn demo turned out to be plain rviz2: Nav2 costmap
(purple obstacles, cyan inflation), lidar, the coverage path as arrows, robot model. Reproduced the
useful part of that with two more publishers in the navigator (map→base_link TF and the filtered
obstacle returns as a `PointCloud2`), an rviz config (`config/survey.rviz`, obstacle cloud with a
60 s decay so it accumulates into a map) and a persistent "obstacles seen" layer on the web page
(0.1 m cells). No costmap: the navigator never had one, and the accumulated returns show the same
thing for an operator.

**Redesign (second reference).** The second reference was a control panel: state banner, mode
buttons, e-stop, manual drive, telemetry, range bars, and a plot with axes. Rebuilt the page that
way. The map is now a plain plot — grey field, 1 m grid, ticked axes in metres, legend — showing the
survey lines as generated (teal, orange direction arrows, red connectors), the driven route (dark
blue), obstacle cells (purple, 0.1 m) with the planner's keep-out radius (cyan, `corridor` from the
status message), the live lidar (green) and a rover drawn at true size. The detour path is no
longer published or drawn: the lines are the reference, the route shows what happened.

The navigator gained a mode: `survey_mode` idle / manual / autonomous (`start_mode` parameter).
Manual hands `cmd_vel` to the page (0.5 m/s, 0.5 rad/s, sent at 10 Hz while a key or button is
held), idle holds the rover, autonomous runs the survey; the e-stop is separate and overrides all
three. Checked against the simulation: manual stops the navigator's `cmd_vel`, idle publishes zeros,
autonomous resumes.

Lesson from the day: a `pgrep -f name$` pattern misses ROS nodes because their command line
continues with `--ros-args`; three stale navigators were driving one Gazebo for two captures.

**Two follow-ups from the mode test.** (1) Switching to manual left the simulated rover coasting
at its last command (Gazebo's diff drive holds the last Twist; the CAN bridge re-sends it at
50 Hz), and it ran off the line into LOST. The navigator now halts once on every mode change, and
the CAN bridge zeroes its command after 0.5 s without `cmd_vel` (`command_timeout`), so a wifi drop
while driving from the page stops the rover. (2) The page only accumulated the route while it was
open, so a page opened mid-survey showed nothing; the navigator now latches the whole route
(`driven_route`, one point per 0.25 m, republished every 2 s) and the page draws that plus a live
tail to the rover. Re-tested: manual → odom speed 0 within 2 s, idle → zero commands, autonomous
→ surveying resumes on the line; a reloaded page shows the full route.

**Simplified.** Feedback: too busy, not good-looking. Cut the page to the four things an operator
uses — the plot, the camera, four numbers (line, speed, cross-track, heading) and two buttons
(start/pause, emergency stop) — on a light theme. Mode buttons, the manual-drive pad, the telemetry
block and the range fan went; `manual` mode stays in the navigator for a teleop node. The plot is
now white with a light grid, steel-blue survey lines with small direction arrows and dashed
connectors, the driven route in one strong blue, obstacle cells in charcoal with a single soft
keep-out halo (one path, so overlaps do not darken), and the legend beside the plot. Fixed the line
counter, which counted both spot turns of each line change and read 5 / 21 on line 3 of 11. All
four button actions verified through the page against the simulation (stop → STOPPED, resume →
SURVEYING, pause → idle, start → autonomous).

**True-size models, zoom and pan.** The rover is now drawn from the URDF (0.925 × 0.38 m chassis,
wheels 0.329 × 0.117 m at ±0.249 / ±0.292 m, lidar at +0.34, antenna at −0.30) and the Tx/Rx carts
from `temrover_carts.xacro` (1.0 × 0.6 m bodies, rods 2.0 and 3.3 m, tow points −0.47 on the rover
and −0.923 on Tx). The carts have no sensor, so the page places them with a rigid-drawbar trailer
model: each cart yaws about its tow point by (step / axle) · sin(tow direction − cart heading).
Checked against Gazebo's link poses on a straight run: Tx position error 0.20 m mean; the Rx figure
(2.0 m) is not a clean test because the run itself snaked (below). The map got wheel zoom about
the cursor, drag pan and double-click refit, with tick spacing chosen from the zoom.

**The towed train breaks the navigation as tuned — every avoidance run so far was `carts:=false`.**
Runs with the carts attached, no obstacles, anchored grid:
- Alignment: the 8 m run drifts +13.7° with the train (gyro damping only holds yaw rate, not
  heading); with `anchor_to_origin: false` the whole grid would inherit that.
- Line tracking oscillates: at 1.5 m/s the rover left line 1 by 5 m and went LOST; at 0.95 m/s it
  still swung ±40° in heading with ±0.9 m cross-track. Without carts the same loop holds ±0.05 m.
- Spot turns drag the train: ~100° in 50 s, wheels skidding. During those turns the heading
  estimate lagged truth by up to 87° (heading_check.py against the model pose). Straight, the
  error is 0.2–1.7°. Working hypothesis: the wheel-odometry speed is wrong while skidding, the
  EKF turns the resulting position innovation partly into yaw through the antenna lever-arm
  Jacobian, and nothing corrects yaw in the sim (`gps/fix_velocity` is not bridged).
- With obstacles as well, the train's own returns leave the towed-train mask as soon as the train
  swings, get planned around, the detour offset creeps +0.8 → +2.7 m chasing them, and the Tx hitch
  jams at its 69° limit. The rover then sits with the train jackknifed.
The simulation runs at real-time factor 0.86; the control loop keeps 20 Hz in sim time, so loop
timing is not the cause. Next: arc turns for the train (Fields2Cover was deferred for this),
gains retuned with the carts on, a train mask from the same trailer model the page uses, and a
predict step that does not trust wheel speed during a blocked turn.

**0.9 m/s recording (results/dashboard_demo.mp4).** Two things surfaced when recording the page
at the payload speed instead of 1.5 m/s:
- With the halved planning margin (0.25 m) the rover deadlocked in front of the line-2 box: the
  ramp at 0.9 m/s is 2.8 m long for a 1.58 m shift (30° kink), the P tracker lags it, the rover
  arrived beside the box 0.5 m short of the planned offset, held at 0.5 m, and with no room for a
  ramp the replans only extended the hold. With the 0.5 m margin (`obstacle_clearance` 0.3,
  `obstacle_hysteresis` 0.2) the same run passed every obstacle with no hold. The halved margin
  is therefore not usable at the real speed; the parameters in `survey.yaml` remain halved until
  the user decides.
- The corner "bystander" pole in `obstacles.yaml` is meant to be removed mid-run by
  `detour_run.py`; in a plain launch it stays and the rover holds at the end of line 1 as designed.
  Removed it by service call for the recording.
Open item: a hold with no room ahead has no exit — the planner needs a from-standstill ramp or a
spot-turn recovery, and the tracker a curvature feed-forward so short ramps are followed.
