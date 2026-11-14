"""Drive to the route-finding targets in mission_2027.yaml.

Each target is reached in two stages: GPS navigation to near the post, then a
final approach on the post's ArUco tag. No lidar.

Getting near the post, two ways:

- Straight line (default). The rover is sent to a point `standoff_m` short of
  the post, on the line it approaches from, through Nav2's GPS waypoint
  follower.
- Planned route (`route_file`). The rover follows the waypoints in a route
  written by `plan_route`, which picks ground within the rover's pitch and
  roll limits from the DEM. The waypoints are read from the file's
  `packet_hex`, i.e. exactly the bytes that would arrive over the radio. Each
  leg is joined into one continuous path and given straight to Nav2's
  controller, so the rover does not stop at, or have to hit, each waypoint.

Final approach (needs the aruco_detector node; `final_approach` turns it off).
The rule is to stop within 1 m of the post, and GPS alone is out by a metre or
two, so the last metres are driven on the tag:

1. Lock. While driving, every sighting of the target's tag within
   `lock_range_m` is turned into a post-centre position in the odom frame. Once
   `lock_count` sightings agree to within `lock_spread_m`, GPS navigation is
   cancelled. The odom frame is used because it does not jump when GPS does.
2. Search. If GPS navigation finishes with no lock, the rover turns on the
   spot for up to one turn; if that fails it backs up 2 m (the camera cannot
   see the tag from closer than about 2.5 m) and turns again.
3. Approach. The rover turns to the post and drives at it, steering on the
   locked position, which keeps being refined while the tag is in view. It
   stops when its front is `stop_gap_m` from the post centre. Once the tag
   leaves the top of the frame the rover finishes on wheel odometry.

Commands go to cmd_vel_nav, i.e. through Nav2's velocity smoother, so the
same speed and acceleration limits apply.

    ros2 run tros_sldworks_pkg gps_mission
    ros2 run tros_sldworks_pkg gps_mission --ros-args -p standoff_m:=1.5
    ros2 run tros_sldworks_pkg gps_mission --ros-args -p route_file:=route.yaml
"""
import math
import os

from ament_index_python.packages import get_package_share_directory
from geographic_msgs.msg import GeoPose
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry, Path
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.time import Time
from robot_localization.srv import FromLL
from scipy.spatial.transform import Rotation
import tf2_ros
from vision_msgs.msg import Detection3DArray
import yaml

from tros_sldworks_pkg.dem_route import decode_packet

EARTH_RADIUS_M = 6378137.0


def offset_m(lat, lon, ref_lat, ref_lon):
    """East/north metres from (ref_lat, ref_lon) to (lat, lon)."""
    east = math.radians(lon - ref_lon) * EARTH_RADIUS_M * math.cos(math.radians(ref_lat))
    north = math.radians(lat - ref_lat) * EARTH_RADIUS_M
    return east, north


def standoff_point(target, came_from, standoff):
    """The point `standoff` metres before `target` on the line from `came_from`."""
    east, north = offset_m(target[0], target[1], came_from[0], came_from[1])
    dist = math.hypot(east, north)
    if dist <= standoff:
        return came_from
    keep = (dist - standoff) / dist
    return (came_from[0] + (target[0] - came_from[0]) * keep,
            came_from[1] + (target[1] - came_from[1]) * keep)


class Mission(BasicNavigator):

    def __init__(self):
        super().__init__('gps_mission')
        default_file = os.path.join(
            get_package_share_directory('tros_sldworks_pkg'), 'config', 'mission_2027.yaml')
        self.declare_parameter('mission_file', default_file)
        self.declare_parameter('standoff_m', 2.5)
        self.declare_parameter('route_file', '')
        # Final approach on the ArUco tag.
        self.declare_parameter('final_approach', True)
        self.declare_parameter('lock_range_m', 7.0)      # use sightings closer than this
        self.declare_parameter('lock_count', 5)
        self.declare_parameter('lock_spread_m', 0.3)
        self.declare_parameter('post_centre_offset_m', 0.058)  # tag face to post axis
        self.declare_parameter('front_reach_m', 0.45)    # base_link to the rover's front
        self.declare_parameter('stop_gap_m', 0.5)        # front to post centre at the stop
        self.declare_parameter('approach_speed', 0.35)
        self.declare_parameter('odom_frame', 'odom')

        self.odom = None
        self.local_pose = None
        self.create_subscription(Odometry, '/odometry/global', self._on_odom, 10)
        self.create_subscription(Odometry, '/odometry/local', self._on_local, 10)
        self.from_ll = self.create_client(FromLL, '/fromLL')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.wanted_id = None
        self.sightings = []          # (x, y) post-centre estimates in odom
        self.create_subscription(Detection3DArray, '/aruco/detections', self._on_tags, 10)
        self.cmd_pub = self.create_publisher(TwistStamped, 'cmd_vel_nav', 10)

    def _on_odom(self, msg):
        self.odom = msg.pose.pose.position

    def _on_local(self, msg):
        p = msg.pose.pose
        q = p.orientation
        self.local_pose = (p.position.x, p.position.y,
                           math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))

    # ------------------------------------------------------- tag sightings
    def _on_tags(self, msg):
        if self.wanted_id is None:
            return
        for det in msg.detections:
            if not det.results or det.results[0].hypothesis.class_id != str(self.wanted_id):
                continue
            pose = det.results[0].pose.pose
            tag = np.array([pose.position.x, pose.position.y, pose.position.z])
            if np.linalg.norm(tag) > self.get_parameter('lock_range_m').value:
                continue
            # The tag's z axis points out of its face, towards the camera, so
            # the post's axis is a little way behind the tag along -z.
            q = pose.orientation
            out = Rotation.from_quat([q.x, q.y, q.z, q.w]).apply([0.0, 0.0, 1.0])
            centre = tag - out * self.get_parameter('post_centre_offset_m').value
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.get_parameter('odom_frame').value, msg.header.frame_id,
                    Time.from_msg(msg.header.stamp), timeout=Duration(seconds=0.0))
            except tf2_ros.TransformException:
                try:
                    tf = self.tf_buffer.lookup_transform(
                        self.get_parameter('odom_frame').value, msg.header.frame_id, Time())
                except tf2_ros.TransformException:
                    continue
            t, r = tf.transform.translation, tf.transform.rotation
            world = Rotation.from_quat([r.x, r.y, r.z, r.w]).apply(centre) + [t.x, t.y, t.z]
            self.sightings.append((float(world[0]), float(world[1])))
            del self.sightings[:-15]

    def post_estimate(self):
        """Median post position in odom, and whether enough sightings agree."""
        if not self.sightings:
            return None, False
        pts = np.array(self.sightings[-self.get_parameter('lock_count').value * 2:])
        centre = np.median(pts, axis=0)
        close = np.hypot(*(pts - centre).T) <= self.get_parameter('lock_spread_m').value
        locked = close.sum() >= self.get_parameter('lock_count').value
        return (centre if locked else None), locked

    # ---------------------------------------------------------- motion
    def drive(self, v, w):
        msg = TwistStamped()     # stamp left at zero: the smoother uses it on arrival
        msg.twist.linear.x = float(v)
        msg.twist.angular.z = float(w)
        self.cmd_pub.publish(msg)

    def wait(self, seconds):
        end = self.get_clock().now() + Duration(seconds=seconds)
        while rclpy.ok() and self.get_clock().now() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def turn_search(self, max_turn=2 * math.pi):
        """Turn on the spot until the tag is locked or `max_turn` is done."""
        turned, last = 0.0, None
        while rclpy.ok() and turned < max_turn:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.post_estimate()[1]:
                self.drive(0, 0)
                return True
            if self.local_pose is not None:
                if last is not None:
                    turned += abs(math.remainder(self.local_pose[2] - last, 2 * math.pi))
                last = self.local_pose[2]
            self.drive(0.0, 0.5)
        self.drive(0, 0)
        return False

    def back_up(self, metres):
        start = self.local_pose
        while rclpy.ok() and start is not None:
            rclpy.spin_once(self, timeout_sec=0.05)
            if math.hypot(self.local_pose[0] - start[0], self.local_pose[1] - start[1]) >= metres:
                break
            self.drive(-0.25, 0.0)
        self.drive(0, 0)
        self.wait(1.0)

    def approach(self, name, timeout_s=90.0):
        """Drive at the locked post until the front is stop_gap_m from it."""
        stop_at = (self.get_parameter('front_reach_m').value
                   + self.get_parameter('stop_gap_m').value)
        speed = self.get_parameter('approach_speed').value
        target, _ = self.post_estimate()
        end = self.get_clock().now() + Duration(seconds=timeout_s)
        while rclpy.ok() and self.get_clock().now() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            fresh, locked = self.post_estimate()
            if locked:
                target = fresh
            x, y, yaw = self.local_pose
            dx, dy = target[0] - x, target[1] - y
            dist = math.hypot(dx, dy)
            if dist <= stop_at:
                self.drive(0, 0)
                self.wait(1.0)
                return dist - self.get_parameter('front_reach_m').value
            error = math.remainder(math.atan2(dy, dx) - yaw, 2 * math.pi)
            if abs(error) > 0.35:
                self.drive(0.0, 0.6 if error > 0 else -0.6)    # face the post first
            else:
                v = max(0.1, min(speed, 0.5 * (dist - stop_at)))
                self.drive(v, 1.5 * error)
        self.drive(0, 0)
        self.error(f'{name}: approach timed out')
        return None

    def final_approach(self, name, tag_id):
        """Search if needed, then approach. Returns the estimated gap or None."""
        if not self.post_estimate()[1]:
            self.info(f'{name}: tag {tag_id} not locked yet; turning to look for it')
            if not self.turn_search():
                self.info(f'{name}: not found; backing up 2 m and looking again')
                self.back_up(2.0)
                if not self.turn_search():
                    return None
        target, _ = self.post_estimate()
        x, y, _ = self.local_pose
        self.info(f'{name}: tag {tag_id} locked, post {math.hypot(target[0] - x, target[1] - y):.2f} m '
                  'away; approaching')
        return self.approach(name)

    def to_map(self, lat, lon):
        """Convert lat/lon to map-frame x, y with navsat_transform."""
        request = FromLL.Request()
        request.ll_point.latitude = lat
        request.ll_point.longitude = lon
        future = self.from_ll.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if future.result() is None:
            return None
        return future.result().map_point.x, future.result().map_point.y

    def leg_path(self, waypoints, spacing=0.25):
        """One map-frame path from the rover through a leg's lat/lon waypoints."""
        rclpy.spin_once(self, timeout_sec=0.5)
        points = [self.to_map(lat, lon) for lat, lon in waypoints]
        if self.odom is None or any(p is None for p in points):
            return None
        points = [(self.odom.x, self.odom.y)] + points
        path = Path()
        # Stamp left at zero: "use the newest transform". This node's clock
        # is not the sim clock, and a wall-clock stamp is rejected as too new.
        path.header.frame_id = 'map'
        last = len(points) - 2
        for i, ((x0, y0), (x1, y1)) in enumerate(zip(points[:-1], points[1:])):
            length = math.hypot(x1 - x0, y1 - y0)
            yaw = math.atan2(y1 - y0, x1 - x0)
            steps = max(1, int(length / spacing))
            for k in range(steps + (1 if i == last else 0)):
                pose = PoseStamped()
                pose.header = path.header
                pose.pose.position.x = x0 + (x1 - x0) * k / steps
                pose.pose.position.y = y0 + (y1 - y0) * k / steps
                pose.pose.orientation.z = math.sin(yaw / 2)
                pose.pose.orientation.w = math.cos(yaw / 2)
                path.poses.append(pose)
        return path

    def estimated_distance_to(self, lat, lon):
        """Distance from the rover's estimated position to a lat/lon point."""
        point = self.to_map(lat, lon)
        if point is None or self.odom is None:
            return None
        return math.hypot(point[0] - self.odom.x, point[1] - self.odom.y)

    def run(self):
        with open(self.get_parameter('mission_file').value) as f:
            route = yaml.safe_load(f)['route_finding']
        standoff = self.get_parameter('standoff_m').value

        self.info('Waiting for Nav2 and navsat_transform...')
        self.waitUntilNav2Active(localizer='robot_localization')
        self.from_ll.wait_for_service()

        legs = None
        route_file = self.get_parameter('route_file').value
        if route_file:
            with open(route_file) as f:
                packet = bytes.fromhex(yaml.safe_load(f)['packet_hex'])
            legs = decode_packet(packet)
            if len(legs) != len(route['targets']):
                raise RuntimeError('the route has a different number of legs than the '
                                   'mission has targets')
            self.info(f'Using planned route from {route_file}: {len(packet)} byte packet, '
                      f'{sum(len(leg) for leg in legs)} waypoints')

        came_from = (route['start']['lat'], route['start']['lon'])
        reached = 0
        for number, target in enumerate(route['targets']):
            name = target['model']
            post = (target['lat'], target['lon'])
            if legs is not None:
                path = self.leg_path(legs[number])
                if path is None:
                    self.error(f'{name}: could not convert the route to map coordinates.')
                    break
                goal_lat, goal_lon = legs[number][-1]
                self.info(f'{name}: post at {post[0]:.7f}, {post[1]:.7f}; '
                          f'following a planned path through {len(legs[number])} waypoints')
                self.followPath(path)
            else:
                goal_lat, goal_lon = standoff_point(post, came_from, standoff)

                goal = GeoPose()
                goal.position.latitude = goal_lat
                goal.position.longitude = goal_lon
                goal.orientation.w = 1.0   # heading at the goal is not checked

                self.info(f'{name}: post at {post[0]:.7f}, {post[1]:.7f}; '
                          f'driving to {goal_lat:.7f}, {goal_lon:.7f} '
                          f'({standoff:.1f} m short)')
                self.followGpsWaypoints([goal])
            use_tag = self.get_parameter('final_approach').value
            self.wanted_id = target['aruco_id'] if use_tag else None
            self.sightings = []
            locked_early = False
            while not self.isTaskComplete():
                rclpy.spin_once(self, timeout_sec=0.05)
                if use_tag and self.post_estimate()[1]:
                    self.cancelTask()
                    locked_early = True
                    self.info(f'{name}: tag {target["aruco_id"]} seen and locked; '
                              'stopping GPS navigation')
                    break

            result = TaskResult.SUCCEEDED if locked_early else self.getResult()
            rclpy.spin_once(self, timeout_sec=0.5)
            if result != TaskResult.SUCCEEDED:
                dist = self.estimated_distance_to(*post)
                dist_text = 'unknown' if dist is None else f'{dist:.2f} m'
                self.error(f'{name}: navigation ended with {result.name}. '
                           f'Estimated distance to post: {dist_text}. Stopping mission.')
                break
            if use_tag:
                gap = self.final_approach(name, target['aruco_id'])
                if gap is None:
                    self.error(f'{name}: could not find or reach tag {target["aruco_id"]}; '
                               'stopped near the GPS point instead')
                else:
                    reached += 1
                    self.info(f'{name}: ARRIVED. Front of rover {gap:.2f} m from the post centre '
                              '(estimated from the tag and odometry)')
            else:
                dist = self.estimated_distance_to(*post)
                dist_text = 'unknown' if dist is None else f'{dist:.2f} m'
                reached += 1
                self.info(f'{name}: arrived. Estimated distance to post: {dist_text} '
                          f'(GPS estimate, not ground truth)')
            self.wanted_id = None
            came_from = (goal_lat, goal_lon)

        self.info(f'Mission finished: {reached} of {len(route["targets"])} targets reached.')


def main():
    rclpy.init()
    mission = Mission()
    try:
        mission.run()
    except KeyboardInterrupt:
        mission.cancelTask()
    finally:
        mission.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
