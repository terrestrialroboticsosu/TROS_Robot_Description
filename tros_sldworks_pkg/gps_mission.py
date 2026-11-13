"""Drive to the route-finding targets in mission_2027.yaml using GPS only.

Stage 1 mission: no lidar, no camera. For each target the rover is sent to a
point `standoff_m` short of the post, on the line it approaches from, through
Nav2's GPS waypoint follower. It cannot see the post, so the standoff is the
only thing keeping it from driving into it.

    ros2 run tros_sldworks_pkg gps_mission
    ros2 run tros_sldworks_pkg gps_mission --ros-args -p standoff_m:=1.5
"""
import math
import os

from ament_index_python.packages import get_package_share_directory
from geographic_msgs.msg import GeoPose
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import Odometry
import rclpy
from robot_localization.srv import FromLL
import yaml

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

        self.odom = None
        self.create_subscription(Odometry, '/odometry/global', self._on_odom, 10)
        self.from_ll = self.create_client(FromLL, '/fromLL')

    def _on_odom(self, msg):
        self.odom = msg.pose.pose.position

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

        came_from = (route['start']['lat'], route['start']['lon'])
        reached = 0
        for target in route['targets']:
            name = target['model']
            post = (target['lat'], target['lon'])
            goal_lat, goal_lon = standoff_point(post, came_from, standoff)

            goal = GeoPose()
            goal.position.latitude = goal_lat
            goal.position.longitude = goal_lon
            goal.orientation.w = 1.0   # heading at the goal is not checked

            self.info(f'{name}: post at {post[0]:.7f}, {post[1]:.7f}; '
                      f'driving to {goal_lat:.7f}, {goal_lon:.7f} '
                      f'({standoff:.1f} m short)')
            self.followGpsWaypoints([goal])
            while not self.isTaskComplete():
                rclpy.spin_once(self, timeout_sec=0.5)

            result = self.getResult()
            rclpy.spin_once(self, timeout_sec=0.5)
            dist = self.estimated_distance_to(*post)
            dist_text = 'unknown' if dist is None else f'{dist:.2f} m'
            if result == TaskResult.SUCCEEDED:
                reached += 1
                self.info(f'{name}: arrived. Estimated distance to post: {dist_text} '
                          f'(GPS estimate, not ground truth)')
            else:
                self.error(f'{name}: navigation ended with {result.name}. '
                           f'Estimated distance to post: {dist_text}. Stopping mission.')
                break
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
