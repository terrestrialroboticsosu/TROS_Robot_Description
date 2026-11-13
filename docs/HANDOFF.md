# Autonomy handoff note

Written 2026-11-12 at the end of a Claude Code session on Windows, where ROS and
Gazebo were not available. **Nothing described here has been run in Gazebo.**
Files were only checked for syntax. Start on Ubuntu by working through
"Check first in Gazebo" below.

## Hardware this is planned around

Jetson Orin Nano, Raspberry Pi 5, Velodyne VLP-16, one OV2311 (mono, global
shutter), one colour global-shutter camera, a standard GPS (RTK possible later),
a BNO055 IMU (possibly replaced later).

## URC 2027 Autonomy Mission (rules section 1.e)

Rules: <https://urc.marssociety.org/home/requirements-guidelines> (PDF linked
there). The Q&A page under it is binding and changes through the year.

40 minutes, two halves of 50 points, in either order. This is **not** the older
"GNSS waypoints, ArUco posts, find a mallet and bottle" format.

**Astronaut Assistance (50)**

| Task | Points | Requirement |
|---|---|---|
| EVA suit with camera and microphone | 5 | Monitored from the command station |
| Drive to astronaut's GNSS location | 5 | Stop within 3 m |
| Follow | up to 15 | Follow, stop within 3 m when they stop |
| Stay | 5 | Hold still while astronaut walks >20 m away |
| Fetch | 10 | Autonomously pick up a rock pick hammer |
| Come | 5 | Drive to astronaut, stop within 3 m |
| Give | 5 | Put down or drop the hammer on command |

Follow points depend on the command method: 5 for a carried device or a sign
(may be an AR tag), 10 for a spoken word, 15 for a silent beckoning gesture.

**Route-Finding (50, 25 per target)**

- Teleoperate to a start point on flat ground, then navigate autonomously.
- Two ArUco posts in hilly terrain. Stop within 1 m, measured from the post to
  the closest point of the rover.
- One is on a hill where only some approaches are drivable; the other is behind
  the hill, out of radio line of sight.
- Post coordinates are exact (Q&A). Organisers aim to provide a DEM at 1 m or
  better.
- Tags: 4x4_50, 20 cm faces, 2.5 cm cells (15 cm black marker), same tag on
  three sides, 0.5 to 1.5 m off the ground.

**Other rules that shape the design**

- LED on the back of the rover: red autonomous, blue teleoperation, flashing
  green on arrival.
- Commanded stop or autonomous return: 20% off that task. Teleoperating back:
  50%. If the rover stops itself there is no penalty and it may be reprogrammed.
- RTK needs your own base station; public correction networks are not allowed.

## Sensor plan

| Sensor | Job |
|---|---|
| GPS | Global position; heading from course-over-ground while moving |
| BNO055 | Yaw rate and roll/pitch only. Do not trust its magnetometer heading |
| Wheel encoders | Forward velocity. Skid-steer yaw is poor on loose soil |
| VLP-16 | Traversability, range to posts and to the astronaut |
| OV2311 | ArUco detection; also suits a wrist camera for the hammer grasp |
| Colour camera | Astronaut tracking, hammer detection, gesture recognition |

Key points:

- **Final approach uses the tag plus lidar, not GPS.** Standard GPS error is
  bigger than the 1 m stop radius, and RTK corrections drop out at the target
  behind the hill.
- With the current lens (fx about 909 px at 1600 wide) the 15 cm marker reads to
  roughly 5 to 6 m. That is enough because coordinates are exact.
- **Terrain planning is the core problem.** Plan the global route over a slope
  cost map from the DEM before the run; use lidar elevation mapping locally.
  Do not flatten the VLP-16 to a 2D scan.
- Mount the lidar high and consider a 10 to 15 degree forward tilt; the ground
  within about 3 m is otherwise unseen.
- The rover must be self-contained at the hidden target: arrive, signal, and
  retrace its recorded path with no radio link.
- Compute split: Orin Nano for lidar, cameras, detection, Nav2, mission logic.
  Pi 5 for motor control, IMU and GPS drivers, LED. Sync clocks with chrony.
- Best upgrade: RTK-capable GNSS (two receivers in moving-base mode also give
  heading). A better IMU matters mainly if lidar-inertial odometry is added.

Suggested order of work:

1. GPS navigation with tag-and-lidar final approach.
2. DEM route planning and lidar traversability.
3. Person following with a sign command, plus Stay.
4. Autonomous hammer pickup and drop.
5. Gesture recognition.

Open question for the organisers: may the rover follow a beacon carried by the
astronaut instead of tracking them by sight? The rules only grade how the
command is given.

## What changed in the sim

Commit "Fix bugs, add covariance":

- `navsat_transform_node` name now matches the key in `config/ekf.yaml`
  (parameters were previously not applied).
- Lidar publishes in `lidar_link`; the identity static transform was removed.
- Sim IMU reports orientation in world ENU (yaw 0 = east); the global EKF fuses
  absolute yaw; `imu_link` is parented to `base_link` so its axes match the
  robot's.
- GPS noise about 1.1 m, gyro noise about 0.1 deg/s with a small bias.
- `tros_sldworks_pkg/gps_covariance.py` republishes `/gps/fix_raw` as
  `/gps/fix` with a covariance, because Gazebo publishes none.

This commit:

- `models/aruco_post_0`, `models/aruco_post_1`: three-sided posts, faces at
  1.0 to 1.2 m, tags 4x4_50 id 0 and 1 with white border.
- `tools/make_aruco_posts.py`: regenerates posts, no third-party packages.
  Ids 0 to 4 are embedded: `python3 tools/make_aruco_posts.py 2 3`.
- `models/rock_hammer`: simple stand-in for the Fetch hammer (grasp testing
  only, not detector training).
- `world/rover_world.sdf`: posts at (12, 5) and (30, -14), hammer at (5, -3),
  metres east/north of the origin. The old flat `aruco_marker_0` was removed
  from the world; its model folder remains.
- `config/mission_2027.yaml`: the same points as lat/lon, stop radii, LED
  colours. Nothing reads it yet.

## Check first in Gazebo

1. **It builds and launches.** `colcon build`, then
   `ros2 launch tros_sldworks_pkg view_gazebo.launch.py`.
2. **GPS noise units.** The horizontal noise is `0.00001` on the belief that
   Gazebo applies it in degrees. With the rover still, echo `/gps/fix_raw`. If
   the position does not wander by about a metre, the unit is metres and the
   value should be about `1.0` (in `urdf/robot_description.urdf.xacro`).
3. **Lidar cloud in RViz** sits at the right height and orientation. If it does
   not appear, try `<frame_id>` instead of `<gz_frame_id>` on the lidar sensor.
4. **Heading.** Face the rover north; `/odometry/global` yaw should be near
   +90 degrees.
5. **Tags are not mirrored.** Compare a post face in the camera view with
   `models/aruco_post_0/materials/textures/aruco_4x4_50_0.png`. If it is
   flipped, swap the `vt` rows in `write_head_obj` and regenerate.

## Still open

- **Terrain.** The hilly terrain was made on another machine and was not in
  this checkout; the world here is still a flat plane. After merging it, set
  each prop's z to the ground height (post poles extend 0.3 m below origin) and
  move the posts to match the rules: one up a hill, one behind it.
- No astronaut model; the mission file only has a GNSS point for them.
- The sim IMU's absolute yaw is exact and GPS noise is white, so heading and
  position look better than they will on hardware.
- EKFs run with `two_d_mode: true`; revisit once driving on slopes.
- `config/ov2311_config.yaml` decodes MJPEG to RGB at 50 fps for a mono
  sensor; mono at 15 to 20 fps with short exposure would be cheaper.
