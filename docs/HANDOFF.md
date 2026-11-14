# Autonomy handoff note

Last updated 2026-11-14, after a Claude Code session on Ubuntu 24.04 with ROS 2
Jazzy and Gazebo. Unlike the first version of this note, what is described
here **has been run in Gazebo**; "Test results" says exactly what was measured
and "Still open" says what was not.

## Where things stand

- The sim builds, launches, and the rover drives on real terrain from the
  competition area (1024 m square, from a lidar DEM, with an aerial photo).
- Localisation works: dual EKF with wheel odometry, IMU and GPS.
- **The Route-Finding mission runs end to end in the sim.** A route planned
  from the DEM on the base station is sent to the rover as one small packet;
  the rover drives it on GPS, locks onto the post's ArUco tag when it comes
  into view, and drives in to stop with its closest point 0.6 to 0.7 m from
  each post (rule: within 1 m).
- Experimental: a lidar terrain cost layer (off by default).
- Not started: the radio link itself, the LED, returning or retracing,
  anything for the Astronaut Assistance half.

## How to run it

```
colcon build
source install/setup.bash
ros2 launch tros_sldworks_pkg view_gazebo.launch.py    # sim, EKFs, RViz
ros2 launch tros_sldworks_pkg nav2.launch.py           # Nav2 + ArUco detector
ros2 run tros_sldworks_pkg gps_mission --ros-args \
    -p route_file:=$PWD/config/route_2027.yaml         # planned route + tag approach
```

The mission takes about seven minutes. Without `route_file` the rover drives
straight lines to the posts instead. The distances it prints on arrival are
its own estimates, not the truth.

To plan a new route (base station; needs the DEM, which is not in the repo):

```
ros2 run tros_sldworks_pkg plan_route ~/Downloads/output.tin.tif --out route.yaml
```

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

- **Final approach uses the tag, not GPS.** A standalone GPS is
  consistent from fix to fix but offset from the truth by a metre or more, and
  the offset cannot be filtered out. Post coordinates are true coordinates, so
  the rover ends up offset by the same amount. RTK would fix this, but its
  corrections drop out at the target behind the hill.
- With the current lens the 15 cm marker reads to roughly 5 to 6 m (7 m in
  the sim, whose camera is narrower). That is enough because coordinates are
  exact.
- **The camera cannot see the tag from closer than about 2.5 m:** it is
  mounted low and level, and the rover's nose-down rest pitch tilts it down
  further. The final approach copes by locking the post position while the
  tag is visible and finishing on wheel odometry. Tilting the camera up 15 to
  20 degrees, or mounting it higher, would still be better.
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

1. ~~GPS navigation~~, ~~DEM route planning~~, ~~tag final approach~~: working
   in the sim. Next: the radio link, LED, and testing on hardware.
2. Lidar traversability (started, experimental).
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

**Navigation** (`config/nav2_params.yaml`, `launch/nav2.launch.py`)

- Nav2 servers only: controller (Regulated Pure Pursuit, 0.8 m/s), planner,
  behaviours, velocity smoother, BT navigator, waypoint follower.
- Both costmaps are empty by default; the terrain knowledge comes from the
  DEM route instead. The global costmap is a 600 m window that follows the
  rover; a goal must be within 300 m.
- Velocity goes controller -> velocity smoother -> `/diff_cont/cmd_vel` as
  `TwistStamped`, which is the only type the drive controller accepts.
- `max_angular_accel` is 10 on purpose. The rover is skid-steer and turns at
  well under half the commanded rate; with the old value of 1.5 the controller
  never built up a turn command and the rover crept round at about 2 deg/s.

**DEM route** (`tros_sldworks_pkg/dem_route.py`, run as `plan_route`)

- Runs on the base station. Resamples the DEM to 1 m and, for every move
  between neighbouring cells, works out the rover's pitch (along the move) and
  roll (across it). Moves over `--max-pitch` (20) or `--max-roll` (15) are
  forbidden; ground steeper than both is avoided by `--clearance` (2 m) to
  allow for GPS error. **These limits are placeholders, not the rover's real
  ones.**
- Not a Nav2 planner on purpose: a costmap holds one value per cell and
  cannot tell climbing a slope from driving across it.
- The cheapest path is reduced to waypoints and packed into a packet of about
  5 bytes per waypoint with a CRC: 68 bytes for the current two-leg route
  (`config/route_2027.yaml`, picture in `docs/route_2027_preview.png`). The
  whole route goes to the rover in one packet before the run, so the rover
  does not need the radio once it has started.
- Each leg after the first starts from the previous post, because the tag
  approach leaves the rover there.

**Mission** (`tros_sldworks_pkg/gps_mission.py`)

- Reads the targets from `mission_2027.yaml`. With `route_file` it decodes the
  packet and gives each leg to Nav2's controller as one continuous path;
  without it, it drives straight to a point `standoff_m` short of each post.
- **Final approach:** every sighting of the target's tag within 7 m becomes a
  post-centre position in the odom frame (which does not jump with GPS). Five
  agreeing sightings lock it; GPS navigation is cancelled and the rover turns
  to the post and drives in, stopping when its front (`front_reach_m`, 0.45 m
  ahead of `base_link`) is `stop_gap_m` (0.5 m) from the post centre. If GPS
  navigation ends without a lock it turns on the spot, and if that fails backs
  up 2 m and turns again.

**ArUco detector** (`tros_sldworks_pkg/aruco_detector.py`)

- Ported from Team Mountaineers' `ros2_aruco` (MIT licence, notice in the
  file). Their version crashes on OpenCV 5, publishes positions with two axes
  swapped (about 1.5 m wrong after a TF transform), and gives every marker in
  an image the first marker's id. All three are fixed, and it works with
  OpenCV 4.6 (Ubuntu's) and 5.
- Publishes `vision_msgs/Detection3DArray` on `/aruco/detections` (class_id is
  the marker id) and a PoseArray on `/aruco/poses` for RViz.

**Lidar terrain** (`tros_sldworks_pkg/lidar_terrain.py`, experimental)

- Builds an elevation map from the VLP-16 (levelled with IMU roll and pitch)
  and scores slope and steps per cell into both costmaps. Turn on with
  `nav2.launch.py use_lidar_terrain:=true`.
- Gave no false obstacles on gentle ground and scored the knoll's rim as
  steep, but it has not completed a mission. Both times it stalled at a crest
  the controller was also hitting the turning problem fixed above, so it
  needs testing again.

## Test results

### 2026-11-14: full mission, planned route and tag approach

Two runs, start to post 0 to post 1, realistic GPS model. Distances are from
the post centre to the closest point of the rover's body and wheels, from the
sim's ground truth.

| | Run 1 | Run 2 (next leg starts at the post) |
|---|---|---|
| Posts reached | 2 of 2 | 2 of 2 |
| Closest point to post 0 / post 1 | 0.56 m / 0.59 m | 0.60 m / 0.71 m |
| Tag approach started from | 4.9 m / 3.4 m | 3.6 m / 4.3 m |
| Largest roll / pitch, leg to post 0 | 6 / 12 deg | 8 / 11 deg |
| Largest roll / pitch, leg to post 1 | 25 / 27 deg | 15 / 25 deg |
| Largest roll / pitch, tag approaches | under 5 / 8 deg | under 6 / 7 deg |

- In run 1 the leg to post 1 started from the end of the previous leg's
  waypoints, 2.5 m short of post 0, so the stretch from where the rover
  actually stopped was unplanned and crossed steep ground. Run 2 is with that
  fixed.
- Pitch on the leg to post 1 is still over the planner's 20 degree limit. The
  measured pitch includes the 5 degree rest pitch, and the rover strays up to
  about 2 m from the planned line.
- In run 2 the rover reached the end of the leg to post 1 without having seen
  the tag; the search turn found it within 2 seconds.

**ArUco detector accuracy** (rover placed straight in front of post 0):

| Distance | 3 m | 4 m | 5 m | 6 m | 7 m | 8 m+ |
|---|---|---|---|---|---|---|
| Position error | 0.05 m | 0.12 m | 0.10 m | 0.17 m | 0.27 m | not detected |

The error is mostly range, which reads about 4% long; the cause has not been
checked. From 2 m the tag is above the top of the camera image.

### 2026-11-12

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

- **Radio link.** Nothing yet sends the route packet over the RFD900x or
  receives it on the rover. The rover should save it to disk and echo its CRC
  back, so the operator knows it arrived intact before starting.
- **Losing the radio.** The route and tag approach need no link, but nothing
  yet lets the rover come back on its own (no return leg in the packet,
  nothing retraces the driven path).
- **Rover limits.** The planner's 20 / 15 degree pitch and roll limits are
  placeholders. The real rover's limits are needed, and the rover strays up
  to 2 m from the planned line, which the limits should allow for.
- **Blind driving.** Nothing stops the rover for a rock the DEM cannot show.
  The lidar terrain layer is the start of this.
- **LED** (red / blue / flashing green) is not driven.
- **EKFs** still run with `two_d_mode: true`, worth revisiting on slopes.
- **Sim realism.** The IMU's absolute yaw is exact, so heading is better than
  it will be on hardware. Wheel slip on loose soil is not modelled.
- **Astronaut Assistance.** Nothing built. No astronaut model; the mission
  file only has a GNSS point for them.
- `mission_2027.yaml` has LED colours and stop radii that nothing reads yet.
- `config/ov2311_config.yaml` decodes MJPEG to RGB at 50 fps for a mono
  sensor; mono at 15 to 20 fps with short exposure would be cheaper.
- The `models/aruco_marker_0` folder is unused.
