# Autonomy handoff note

Last updated 2026-11-12, after a Claude Code session on Ubuntu 24.04 with ROS 2
Jazzy and Gazebo. Unlike the first version of this note, what is described
here **has been run in Gazebo**; "Test results" says exactly what was measured
and "Still open" says what was not.

## Where things stand

- The sim builds, launches, and the rover drives on real terrain from the
  competition area (1024 m square, from a lidar DEM, with an aerial photo).
- Localisation works: dual EKF with wheel odometry, IMU and GPS.
- **Stage 1 navigation works:** Nav2 drives the rover to both route-finding
  posts using odometry and GPS only. It is blind (no lidar, no camera), plans a
  straight line, and deliberately stops 2.5 m short of each post.
- Not started: obstacle avoidance, tag detection and final approach, terrain-
  aware planning, anything for the Astronaut Assistance half.

## How to run it

```
colcon build
source install/setup.bash
ros2 launch tros_sldworks_pkg view_gazebo.launch.py    # sim, EKFs, RViz
ros2 launch tros_sldworks_pkg nav2.launch.py           # second terminal
ros2 run tros_sldworks_pkg gps_mission                 # third terminal
```

The mission takes about six minutes. It prints the distance to each post on
arrival, but that number is the rover's own estimate, not the truth.

## Hardware this is planned around

Jetson Orin Nano, Raspberry Pi 5, Velodyne VLP-16, one OV2311 (mono, global
shutter), one colour global-shutter camera, a standard GPS (RTK possible later),
a BNO055 IMU (possibly replaced later).

The rover's left and right sides pivot independently so it can conform to
rocks. With the current weight distribution it rests nose-down against the
pivot stops, about 4 to 5 degrees. The sim reproduces this; the angle is set
by `rotatable_link_limit` in the xacro (5 degrees). It is not a bug. Anything
using the lidar or camera must take roll and pitch from the IMU, not assume a
level chassis.

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

- **Final approach uses the tag plus lidar, not GPS.** A standalone GPS is
  consistent from fix to fix but offset from the truth by a metre or more, and
  the offset cannot be filtered out. Post coordinates are true coordinates, so
  the rover ends up offset by the same amount. RTK would fix this, but its
  corrections drop out at the target behind the hill.
- With the current lens the 15 cm marker reads to roughly 5 to 6 m. That is
  enough because coordinates are exact.
- **Terrain planning is the core problem.** Plan the global route over a slope
  cost map from the DEM before the run; use lidar elevation mapping locally.
  Do not flatten the VLP-16 to a 2D scan. The test runs below show why: the
  straight line to post 1 took the rover to 44 degrees of roll.
- The sim lidar sits about 0.56 m above the ground and 0.4 m behind
  `base_link`. Mount it high; the ground close in front is otherwise unseen.
- The rover must be self-contained at the hidden target: arrive, signal, and
  retrace its recorded path with no radio link.
- Compute split: Orin Nano for lidar, cameras, detection, Nav2, mission logic.
  Pi 5 for motor control, IMU and GPS drivers, LED. Sync clocks with chrony.
- Best upgrade: RTK-capable GNSS (two receivers in moving-base mode also give
  heading). A better IMU matters mainly if lidar-inertial odometry is added.

Order of work:

1. ~~GPS navigation~~ done (stage 1). Next: tag-and-lidar final approach.
2. DEM route planning and lidar traversability.
3. Person following with a sign command, plus Stay.
4. Autonomous hammer pickup and drop.
5. Gesture recognition.

Open question for the organisers: may the rover follow a beacon carried by the
astronaut instead of tracking them by sight? The rules only grade how the
command is given.

## What is in the sim

**Terrain** (`models/urc_terrain`, built by `tools/make_terrain.py`)

- 1024 m square cut from the lidar DEM (`~/Downloads/output.tin.tif`, 0.22 m
  grid, not in the repo), sampled at 1 m. World origin is the tile centre,
  38.4166050 N, -110.7761587 E.
- World z = 0 is the lowest point of the tile, so the ground at the origin is
  at z = 31.47. The launch file reads `terrain.yaml` to spawn the rover at the
  right height. Use `python3 tools/make_terrain.py --height-at X Y` to get the
  ground height for placing a prop.
- Ground photo: ArcGIS World Imagery, 0.25 m per pixel, read from a `.tpkx`
  tile package with `--imagery`. Without that option the tool downloads USGS
  NAIP (public domain, 0.6 m). **The repo is public and the World Imagery
  texture is Esri's; nobody has checked that their terms allow it to be
  there.**
- 8% of the tile had no DEM data and was filled by interpolation.

**Props** (`world/rover_world.sdf`, mirrored in `config/mission_2027.yaml`)

- Start at the origin. Post 0 at (110, -100) on a knoll that is steep on its
  north and east sides. Post 1 at (164, -100), behind the knoll.
- Hammer stand-in at (5, -3). No astronaut model.

**Rover model changes** (`urdf/robot_description.urdf.xacro`)

- Wheel collisions are cylinders, not the 38,000-triangle meshes, which sank
  into the heightmap.
- One link had all-zero inertia; it now has a placeholder. Replace with CAD
  values.
- Lidar publishes in `lidar_link`. IMU reports orientation in world ENU
  (yaw 0 = east) and is parented to `base_link`.
- Camera 15 Hz, GPS 1 Hz, physics step 5 ms.

**GPS error model** (`tros_sldworks_pkg/gps_covariance.py`)

Gazebo publishes the true position on `/gps/fix_raw`. The node adds the error
and a covariance and republishes on `/gps/fix`. The error is a slowly drifting
offset, not independent noise on each fix:

| Part | Size per horizontal axis | Changes over |
|---|---|---|
| Slow offset (orbits, clocks, atmosphere) | 1.1 m | about 10 minutes |
| Fast offset (reflections) | 0.3 m | about 30 seconds |
| Fix-to-fix noise | 0.05 m | every fix |

Vertical is 1.7 times horizontal. Overall that is 1.14 m per axis (1.35 m CEP)
with only 0.12 m of change between consecutive fixes. These are typical figures
for a standalone single-band receiver, **not measurements of the rover's
receiver**. All are node parameters. To set them properly, log the real GPS
sitting still for an hour.

**Localisation** (`config/ekf.yaml`)

- Local EKF (odom to base_link): wheel odometry and IMU.
- Global EKF (map to odom): wheel odometry, IMU including absolute yaw, GPS.
- `diff_cont` takes speed from wheel velocity; differencing wheel position
  read high and noisy in Gazebo.

**Navigation, stage 1** (`config/nav2_params.yaml`, `launch/nav2.launch.py`)

- Nav2 servers only: controller (Regulated Pure Pursuit, 0.8 m/s), planner,
  behaviours, velocity smoother, BT navigator, waypoint follower.
- Both costmaps are empty, so the plan is a straight line. The global costmap
  is a 600 m window that follows the rover; a goal must be within 300 m.
- Velocity goes controller -> velocity smoother -> `/diff_cont/cmd_vel` as
  `TwistStamped`, which is the only type the drive controller accepts.
- `gps_mission` reads the targets from `mission_2027.yaml` and sends each to
  the GPS waypoint follower, aimed `standoff_m` (default 2.5 m) short of the
  post. The standoff is the only thing stopping the blind rover from hitting
  the post, and it means stage 1 cannot score the 1 m requirement.

## Test results (2026-11-12)

**Sensor checks**, done on the earlier flat world, before the terrain went in:

| Check | Result |
|---|---|
| Lidar cloud | Ground at z = 0.00 +/- 0.03 m in the world frame; a post seen within 5 cm of its true position |
| Heading | Facing north reads 90.2 degrees (true 90.3); facing east reads 0.0 |
| Tags | Id 0 detected by OpenCV from 3.5 m, and not when the image is mirrored. Only the south face of post 0 was checked |

These were not repeated on the terrain.

**GPS mission on the terrain**, start to post 0 to post 1, one run each:

| | Old GPS (independent 1 m noise) | New GPS model |
|---|---|---|
| Targets reached | 2 of 2 | 2 of 2 |
| Time | 224 s + 140 s | 228 s + 118 s |
| True stop distance, post 0 | about 3.2 m | 2.4 m |
| True stop distance, post 1 | 2.8 m | 2.1 m |
| Rover's own estimate | 2.80 m, 2.95 m | 2.97 m, 2.85 m |
| Largest roll / pitch | 27 / 29 degrees | 44 / 38 degrees |

Reading these:

- Stopping 2 to 3 m out is the 2.5 m standoff plus the 0.5 m goal tolerance
  plus GPS error. It is by design, not a miss.
- With the new GPS model the rover's estimate and the truth disagreed by up to
  0.8 m. That gap is the GPS offset, and the rover cannot see it.
- The roll and pitch are the result of driving a straight line over the knoll.
  44 degrees is not a safe angle for the real rover. The two runs differ
  because the GPS offset shifted the line slightly; it is one run each, so do
  not read more into the difference than that.
- Nav2 reported no aborts or recovery behaviours in the second run.

## Still open

- **Blind driving.** Add a lidar obstacle layer to the costmaps. The cloud has
  to be height-filtered in a gravity-aligned frame because of the resting
  pitch, or the ground ahead reads as an obstacle.
- **Final approach.** No ArUco detector node exists and no ArUco ROS package is
  installed. Needs a detector and an approach step that takes over from the
  waypoint follower near the post.
- **Route planning.** Slope cost map from the DEM; both EKFs still run with
  `two_d_mode: true`, which should be revisited now the rover is on slopes.
- **Sim realism.** The IMU's absolute yaw is exact, so heading is better than
  it will be on hardware. Wheel slip on loose soil is not modelled.
- **Astronaut Assistance.** Nothing built. No astronaut model; the mission
  file only has a GNSS point for them.
- `mission_2027.yaml` has LED colours and stop radii that nothing reads yet.
- `config/ov2311_config.yaml` decodes MJPEG to RGB at 50 fps for a mono
  sensor; mono at 15 to 20 fps with short exposure would be cheaper.
- The `models/aruco_marker_0` folder is unused.
