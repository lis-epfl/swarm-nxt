# vision_pose_ros2

`vision_pose_node` gives PX4 its pose. It is the only writer of
`/fmu/in/vehicle_visual_odometry` and replaces `mocap_to_vision_pose_ros2`.

- With the mocap only it does what the old node did: every valid mocap sample
  becomes one `px4_msgs/VehicleOdometry` message (NED position, FRD attitude).
- With an estimator pose (`vio_topic`) it also places the VIO in the room while
  the drone stands still, publishes the placed VIO (`odometry_topic`) and a
  status (`status_topic`, 10 Hz and at every change of source).
- With `use_vio` it feeds PX4 the placed VIO from the ground on, and goes back
  to the mocap for good when PX4 goes too far from the mocap, when the two
  can no longer be compared, or when the VIO stops.

The logic (buffers, placement, state machine, conversions) is in
`VisionPoseCore` (`vision_pose_core.hpp`), which has no ROS dependency. The
node class `VisionPose` only wires topics, parameters and one timer.

## What PX4 is fed

1. NONE until the first valid mocap sample, then MOCAP: each valid sample
   (`tracking_valid`, finite, stamp above zero) is forwarded, others dropped.
2. `use_vio` false or `vio_topic` empty: MOCAP for ever.
3. The VIO is settled when its poses have come for `vio_settle_s` without a gap
   above 0.2 s and all lie within `vio_still_m` of their mean.
4. Every VIO pose is paired with the mocap at its stamp (between two valid
   samples at most 50 ms apart). If the mocap has not reached that stamp when
   the pose arrives, the pose waits (up to 64 of them) and is paired when the
   mocap gets there. A pose with no valid mocap around its stamp is never
   paired.
5. While disarmed and settled the VIO is placed at every pose from the pairs
   of the last second (heading with the mount yaw, pin, body tilt). The
   placement is frozen from the arming edge on.
6. MOCAP -> VIO only while disarmed, with `use_vio`, the VIO ready, a mount, a
   vehicle status received within the last 2 s, PX4's own position being
   compared with the mocap (so that rule 9a is running) and no fallback
   latched; `reset_counter` + 1. Without a vehicle status for 2 s the arming
   state is not known: the VIO is neither placed nor switched to (reason "no
   vehicle status"). Without PX4's position the reason is "no vehicle
   odometry".
7. Armed before that: MOCAP for this flight.
8. On the VIO each pose gives one PX4 message; mocap samples are not forwarded.
   When the VIO is placed again while PX4 is on it, the pose sent steps by
   what the VIO drifted, and `reset_counter` is raised by one with that step.
   This is the first placement after a stretch without one: after a flight
   (once disarmed and settled), after the drone was carried (it has to stand
   for `vio_settle_s` again), after the mocap or the vehicle status was away.
   The placement at every pose that follows while the drone keeps standing
   does not raise it.
9. VIO -> MOCAP, `reset_counter` + 1, latched until the node restarts, when
   one of these holds:
   - a) PX4's position (`vehicle_odometry_topic`) is further than
     `fallback_dist_m` from the mocap at the same stamp for `fallback_hold_s`.
     The hold needs every comparison in it above the limit; it goes on over
     up to 0.5 s without comparison (slow odometry, a hole in the mocap) and
     starts again after more.
   - b) no VIO pose arrives for `vio_timeout_s`. The default, 0.33 s, is
     held between two bounds. On the recorded flight of June the estimator
     gave a pose every 133 ms in flight (173 ms at most), and with one pose
     lost its neighbours are up to 325 ms apart: 0.33 s rides through one
     lost pose there and falls back on two. And EKF2 stops using vision when
     two samples are more than 400 ms apart in their stamps; at this
     fallback the stamps step by the timeout, the age of the poses when they
     arrive (58 ms in the tests), up to 10 ms until the timer runs and up to
     4 ms until the next mocap sample, less the age of that sample (7 ms in
     the tests), which leaves about 334 ms. Set the value again from the
     pose age and the arrival jitter measured on the flight computer.
   - c) PX4's position has not been compared with the mocap for 1.0 s (its
     odometry stopped, or its stamps no longer meet the mocap's) although
     valid mocap samples have been arriving for 1.0 s without a hole of more
     than 50 ms. Rule a) cannot run then. A mocap that is away, or whose
     samples cannot be compared with (a quaternion that is not a rotation),
     does not trigger this: PX4 stays on the VIO, and the 1.0 s counts from
     the mocap's return.
   - d) the stamps of the VIO jump forward by more than 1.0 s while its poses
     keep arriving (see rule 12).
   The VIO is not judged against the mocap by itself. A VIO that jumps is
   sent to PX4 as it comes, and rule a) ends it once PX4's own position has
   followed the jump.
   Rule c) applies only to a VIO placed against the mocap. Rule a)
   runs whenever valid mocap samples are around the stamp of PX4's position,
   also for a VIO placed at the start pose.
10. Without a valid mocap at that moment: NONE, until a valid sample arrives.
11. A node restarted in flight is on MOCAP and stays there.
12. VIO stamps that jump forward by more than 1.0 s mean that the estimator
    has started again, in another world: the placement is dropped. On the
    ground the VIO is placed again once it has stood for `vio_settle_s`; in
    flight it stays unplaced until the landing. A jump that only comes from
    the node itself having been held up (it did not read the poses in
    between) is not taken for a restart.
13. VIO poses whose stamp goes back are dropped. Only on the ground, while PX4
    is not fed the VIO, a step back of more than 5 s is taken as a new stream
    (a recording played again) and placed again. On the VIO, such a stream
    therefore ends in the timeout of rule 9b.
14. After a fallback with `correct_mocap_attitude`, the rotation `C` in the
    mocap feed is the one PX4 had on the VIO; a later placement does not
    change it.
15. The rules that go by time (9b, 9c) are checked 100 times a second. When
    two checks are more than 50 ms apart the node itself was held up, and
    what arrived meanwhile has not been read yet: these rules are then
    skipped for 0.2 s. Only two late checks get this, until no check has
    been late for 1 s: hold-ups that keep coming cannot switch the rules
    off. The other side of it: a third hold-up within a second of the one
    before gets no pause. If the node has then read no pose for
    `vio_timeout_s`, it gives the VIO up although the VIO never stopped.

## Log lines

- `pose source: <OLD> -> <NEW> (<reason>)` at every change of source.
- `VIO ready: ...` when the VIO is placed, `VIO not ready any more: ...` when
  it moved or paused on the ground.
- `VIO placed again: step <m> m, heading <deg> deg, reset_counter <n>` when
  rule 8 raises the counter.
- `VIO stream restarted (stamps jumped <s> s): placement dropped` (rule 12).
- `VIO not used: <reason>` whenever the reason changes while `use_vio` is set
  and PX4 is not fed the VIO (the reasons of the status message: "no vehicle
  status", "vio not ready", "no mount", "no vehicle odometry", "vio not ready
  before arming", "vio timeout", "distance limit").
- `armed: ...` at the arming edge.

## Parameters

Numbers may be written with or without a decimal point (`3.0` or `3`, and
`[1, -2, 0]` for `start_position`). Any other type where a number is expected
(a text, true/false), and a value that cannot work, stop the node at start
with a FATAL line. All parameters are read once at start.

| name | default | meaning |
| --- | --- | --- |
| `mocap_topic` | `/optitrack_multiplexer_node/rigid_body/mavros` | rigid body of the drone |
| `pos_var`, `att_var` | 1e-6 | variances sent with the mocap (m^2, rad^2) |
| `vio_topic` | `""` | estimator pose (`PoseWithCovarianceStamped`); empty: no VIO handling |
| `use_vio` | false | PX4 may be fed the VIO; false with a `vio_topic`: place and log only |
| `has_mount`, `mount_yaw_deg` | false, 0.0 | mount yaw of the calibration (`mount.json`) |
| `vio_lever_arm` | [0, 0, 0] | position of the IMU from the origin of the rigid body, in the IMU frame (m), from the calibration (`mount.json`, `lever_arm_imu_m`): the VIO is moved back by it onto the point the mocap reports, at any attitude |
| `world_from_start_pose` | false | without any mocap sample, place the VIO at the start pose (outdoors) |
| `start_position`, `start_yaw_deg` | [0, 0, 0], 0.0 | that start pose (m, deg; IMU body) |
| `fallback_dist_m`, `fallback_hold_s` | 0.15, 0.10 | distance rule on PX4's position (rule 9a) |
| `vio_settle_s`, `vio_still_m` | 7.0, 0.03 | standstill needed before the VIO is placed |
| `vio_timeout_s` | 0.33 | silence of the VIO that triggers the fallback (arrival time, rule 9b). Above the 325 ms of one lost pose on the recorded flight, below about 334 ms: the 400 ms of EKF2 minus the age of the poses (58 ms in the tests) and about 8 ms that the timer and the mocap add (rule 9b); to be set again from the pose age and the arrival jitter measured on the flight computer |
| `vio_pos_var`, `vio_att_var` | 1e-6 | variances sent with the VIO |
| `correct_mocap_attitude` | false | after a fallback send the attitude of the IMU body (`R_m C^T`) |
| `heading_trim_rate_dps` | 0.0 | slew rate of the heading trim; 0 = off |
| `heading_trim_start_m`, `heading_trim_window_s`, `heading_trim_max_deg` | 2.0, 20.0, 2.0 | where it starts, how long it runs, its bound |
| `vehicle_status_topic` | `/fmu/out/vehicle_status_v1` | arming state |
| `vehicle_odometry_topic` | `/fmu/out/vehicle_odometry` | PX4's own position |
| `odometry_topic`, `status_topic` | `vio/odometry`, `vio/status` | placed VIO (`nav_msgs/Odometry`, frame `world`), `swarmnxt_msgs/VisionPoseStatus` |

## Build and run

```
colcon build --packages-select swarmnxt_msgs vision_pose_ros2
ros2 run vision_pose_ros2 vision_pose_node --ros-args -p mocap_topic:=/optitrack_multiplexer_node/rigid_body/nxt3
```

## Checks

Offline check of the logic on synthetic data (no ROS graph, 0.1 s):

```
colcon test --packages-select vision_pose_ros2 --event-handlers console_direct+
```

Acceptance test of the mocap path against a recorded flight (the node must
reproduce what the old node sent). On a ROS domain of its own, with
`ROS_LOCALHOST_ONLY=1`:

```
# 1. the node on the mocap only
ros2 run vision_pose_ros2 vision_pose_node --ros-args -p mocap_topic:=/optitrack_multiplexer_node/rigid_body/nxt3
# 2. record what it sends
ros2 bag record -o replay_out /fmu/in/vehicle_visual_odometry
# 3. replay the mocap topic only (the bag also holds the old node's output)
ros2 bag play <flight>/bag --topics /optitrack_multiplexer_node/rigid_body/nxt3
```

Then compare `replay_out` with `/fmu/in/vehicle_visual_odometry` of the flight
bag on the pairs of equal `timestamp_sample`: stamp, position and constant
fields equal, `q` within 2.4e-7 per component, one message per valid mocap
sample and none for the invalid ones.
