"""Build a terrain cost map from the lidar for Nav2.

Nav2's stock obstacle layers call anything above a flat ground plane an
obstacle, so a hillside reads as a wall. This node instead keeps a small
elevation map around the rover and scores each cell by how steep the ground is
and whether something sticks up out of it. The result is published as an
OccupancyGrid that both costmaps load through a static layer.

Per lidar scan:

1. Level the cloud. Points go lidar -> base_link (static TF), are rotated by
   the IMU's roll and pitch so z is truly up, then placed in the odom frame
   with the rover's x, y and yaw. The EKFs run in 2D mode, so the odom
   transform carries no roll, pitch or height; that is why the IMU is used
   directly.
2. Find the rover's height. Odometry gives none, so each scan is slid up or
   down until it agrees with the map built so far (median difference over the
   cells they share).
3. Fuse the scan into the elevation map: a running mean height and the highest
   point seen, per cell.
4. Fill unseen pockets. From the top of a rise the lidar cannot see the face
   that drops away in front of it, so that face is a patch of unseen cells
   with seen ground all round. Each such cell gets a height interpolated from
   the seen ground on either side (along its row and its column). A pocket
   between a high rim and low ground beyond therefore comes out steep; a gap
   between two lidar rings on level ground comes out level.
5. Around the rover, fit a plane to each cell's neighbourhood. Its tilt is the
   slope; a point well above it is a step (rock, post).
6. Cost: 0 below slope_free_deg, rising to 98 at slope_lethal_deg, 100 (lethal)
   at or above it and for steps. Ground the rover has driven over is free.
   Unseen cells that are not enclosed by seen ground stay unknown, which the
   costmaps treat as free; without that the rover could never plan beyond
   lidar range.

Limits worth knowing: a filled pocket only gets the average slope across it,
so a short cliff inside a wide pocket is understated; a drop with nothing
visible beyond it is not enclosed and is not caught; and one value per cell
cannot say that driving up a slope and driving across it are different.
"""
from array import array
import math

from nav_msgs.msg import OccupancyGrid, Odometry
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data, ReliabilityPolicy
from rclpy.time import Time
from scipy.ndimage import binary_fill_holes, uniform_filter
from sensor_msgs.msg import Imu, PointCloud2
from sensor_msgs_py import point_cloud2
import tf2_ros


def rotation(q):
    """3x3 rotation matrix from a quaternion message."""
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class LidarTerrain(Node):

    def __init__(self):
        super().__init__('lidar_terrain')
        p = self.declare_parameter
        p('cell_m', 0.25)             # map cell size
        p('map_size_m', 160.0)        # side of the square map kept in memory
        p('max_range_m', 15.0)        # ignore returns beyond this
        p('body_half_length_m', 0.9)  # returns inside this box are the rover itself
        p('body_half_width_m', 0.7)
        p('max_height_m', 2.0)        # ignore returns this far above the lidar
        p('fit_window_cells', 5)      # plane-fit neighbourhood (5 cells = 1.25 m)
        p('slope_free_deg', 8.0)
        p('slope_lethal_deg', 30.0)
        p('step_lethal_m', 0.30)
        p('analysis_radius_m', 20.0)  # re-score this far around the rover per scan
        p('driven_radius_m', 1.3)     # ground this close to the rover's track counts as seen
        p('publish_period_s', 0.5)
        p('odom_frame', 'odom')
        p('base_frame', 'base_link')

        g = lambda name: self.get_parameter(name).value   # noqa: E731
        self.cell = g('cell_m')
        self.n = int(round(g('map_size_m') / self.cell))
        self.odom_frame = g('odom_frame')
        self.base_frame = g('base_frame')

        # Map arrays, indexed [row (y), column (x)]. origin = world x, y of the
        # corner of cell [0, 0]; it is set on the first scan and moves in whole
        # cells when the rover nears an edge.
        self.origin = None
        self.count = np.zeros((self.n, self.n), np.uint16)
        self.height = np.zeros((self.n, self.n), np.float32)
        self.top = np.full((self.n, self.n), -np.inf, np.float32)
        self.cost = np.full((self.n, self.n), -1, np.int8)
        self.slope_deg = np.full((self.n, self.n), np.nan, np.float32)
        self.driven = np.zeros((self.n, self.n), bool)
        self.rover_z = 0.0

        self.imu = None
        self.odom = None
        self.lidar_to_base = None
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_subscription(Imu, 'imu/data', self._on_imu, qos_profile_sensor_data)
        self.create_subscription(Odometry, 'odometry/local', self._on_odom, 10)
        self.create_subscription(PointCloud2, 'velodyne_points', self._on_cloud,
                                 qos_profile_sensor_data)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.cost_pub = self.create_publisher(OccupancyGrid, 'terrain/cost', latched)
        self.create_timer(g('publish_period_s'), self._publish)

    def _on_imu(self, msg):
        self.imu = msg

    def _on_odom(self, msg):
        self.odom = msg

    # ------------------------------------------------------------------ scan
    def _on_cloud(self, msg):
        if self.imu is None or self.odom is None:
            return
        if self.lidar_to_base is None:
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.base_frame, msg.header.frame_id, Time(),
                    timeout=Duration(seconds=0.0))
            except tf2_ros.TransformException:
                return
            t = tf.transform.translation
            self.lidar_to_base = (rotation(tf.transform.rotation), np.array([t.x, t.y, t.z]))

        g = lambda name: self.get_parameter(name).value   # noqa: E731
        pts = point_cloud2.read_points_numpy(msg, field_names=('x', 'y', 'z'))
        pts = pts[np.isfinite(pts).all(axis=1)].astype(np.float64)
        if len(pts) == 0:
            return
        rot, trans = self.lidar_to_base
        pts = pts @ rot.T + trans                      # now in base_link

        keep = ((np.hypot(pts[:, 0], pts[:, 1]) < g('max_range_m'))
                & ~((np.abs(pts[:, 0]) < g('body_half_length_m'))
                    & (np.abs(pts[:, 1]) < g('body_half_width_m'))))
        pts = pts[keep]

        # Level with the IMU's roll and pitch, then turn and shift into odom.
        q = self.imu.orientation
        roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))
        pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))
        cr, sr, cp, sp = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch)
        level = (np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
                 @ np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]]))
        pose = self.odom.pose.pose
        yaw = yaw_of(pose.orientation)
        cy, sy = math.cos(yaw), math.sin(yaw)
        pts = pts @ level.T
        pts = pts[pts[:, 2] < trans[2] + g('max_height_m')]
        x = pose.position.x + cy * pts[:, 0] - sy * pts[:, 1]
        y = pose.position.y + sy * pts[:, 0] + cy * pts[:, 1]
        z = pts[:, 2]

        self._recentre(pose.position.x, pose.position.y)
        col = np.floor((x - self.origin[0]) / self.cell).astype(np.int64)
        row = np.floor((y - self.origin[1]) / self.cell).astype(np.int64)
        inside = (col >= 0) & (col < self.n) & (row >= 0) & (row < self.n)
        flat = row[inside] * self.n + col[inside]
        z = z[inside]
        if len(flat) == 0:
            return

        # One mean and one max per cell for this scan.
        cells, inverse, hits = np.unique(flat, return_inverse=True, return_counts=True)
        scan_mean = np.bincount(inverse, weights=z) / hits
        scan_top = np.full(len(cells), -np.inf)
        np.maximum.at(scan_top, inverse, z)

        # Slide the scan vertically onto the existing map.
        count = self.count.ravel()
        height = self.height.ravel()
        top = self.top.ravel()
        seen = count[cells] > 0
        if seen.sum() >= 30:
            shift = float(np.median(height[cells][seen] - scan_mean[seen]))
            self.rover_z += float(np.clip(shift - self.rover_z, -0.1, 0.1))
        scan_mean += self.rover_z
        scan_top += self.rover_z

        new_count = np.minimum(count[cells] + 1, 20)
        height[cells] += (scan_mean - height[cells]) / new_count
        count[cells] = new_count
        top[cells] = np.maximum(top[cells], scan_top)

        self._score(pose.position.x, pose.position.y)

    def _recentre(self, x, y):
        """Keep the rover in the middle half of the map, moving it by whole cells."""
        span = self.n * self.cell
        if self.origin is None:
            self.origin = [math.floor((x - span / 2) / self.cell) * self.cell,
                           math.floor((y - span / 2) / self.cell) * self.cell]
            return
        for axis, value in ((0, x), (1, y)):
            offset = value - (self.origin[axis] + span / 2)
            if abs(offset) < span / 4:
                continue
            cells = int(round(offset / self.cell))
            self.origin[axis] += cells * self.cell
            for grid, empty in ((self.count, 0), (self.height, 0), (self.top, -np.inf),
                                (self.cost, -1), (self.slope_deg, np.nan),
                                (self.driven, False)):
                grid[:] = np.roll(grid, -cells, axis=1 - axis)
                edge = [slice(None), slice(None)]
                edge[1 - axis] = slice(-cells, None) if cells > 0 else slice(0, -cells)
                grid[tuple(edge)] = empty

    # ----------------------------------------------------------------- score
    def _score(self, x, y):
        """Plane-fit slope and step height for the cells around the rover."""
        g = lambda name: self.get_parameter(name).value   # noqa: E731
        reach = int(g('analysis_radius_m') / self.cell)
        c0 = int((x - self.origin[0]) / self.cell)
        r0 = int((y - self.origin[1]) / self.cell)
        rows = slice(max(0, r0 - reach), min(self.n, r0 + reach))
        cols = slice(max(0, c0 - reach), min(self.n, c0 + reach))

        seen = self.count[rows, cols] > 0
        h = self.height[rows, cols].astype(np.float64) - self.rover_z
        filled = self._fill_pockets(seen, h)           # heights for enclosed unseen cells
        pocket = np.isfinite(filled) & ~seen
        known = (seen | pocket).astype(np.float64)
        h = np.where(seen, h, np.where(pocket, filled, 0.0))
        yy, xx = np.mgrid[0:known.shape[0], 0:known.shape[1]].astype(np.float64) * self.cell

        window = int(g('fit_window_cells'))
        mean = lambda a: uniform_filter(a, window, mode='constant')   # noqa: E731
        n = mean(known)
        ok = n * window * window >= 6
        n_safe = np.where(ok, n, 1.0)
        mx, my, mz = mean(xx * known) / n_safe, mean(yy * known) / n_safe, mean(h) / n_safe
        cxx = mean(xx * xx * known) / n_safe - mx * mx
        cyy = mean(yy * yy * known) / n_safe - my * my
        cxy = mean(xx * yy * known) / n_safe - mx * my
        cxz = mean(xx * h) / n_safe - mx * mz
        cyz = mean(yy * h) / n_safe - my * mz
        det = cxx * cyy - cxy * cxy
        # Smaller eigenvalue of the x/y spread: tiny when the known cells lie
        # along a single line (one lidar ring), where a plane is not defined.
        spread = (cxx + cyy) / 2 - np.sqrt(np.maximum(((cxx - cyy) / 2) ** 2 + cxy * cxy, 0))
        ok &= spread > 0.03
        det = np.where(ok, det, 1.0)
        a = (cyy * cxz - cxy * cyz) / det
        b = (cxx * cyz - cxy * cxz) / det
        slope = np.degrees(np.arctan(np.hypot(a, b)))
        plane = mz + a * (xx - mx) + b * (yy - my)     # fitted height at each cell

        top = self.top[rows, cols].astype(np.float64) - self.rover_z
        ground = np.where(ok, plane, h)
        step = seen & (top - ground > g('step_lethal_m'))

        free, lethal = g('slope_free_deg'), g('slope_lethal_deg')
        cost = np.clip((slope - free) / (lethal - free), 0.0, 1.0) * 98.0
        cost = np.where(slope >= lethal, 100.0, cost)
        cost = np.where(ok, cost, -1.0)
        cost = np.where(step, 100.0, cost)
        cost = np.where(known > 0, cost, -1.0)
        # A cell seen but with too few neighbours for a plane keeps its old score.
        old = self.cost[rows, cols]
        undecided = (known > 0) & ~ok & ~step
        cost = np.where(undecided, old, cost.astype(np.int8))
        self.slope_deg[rows, cols] = np.where(ok, slope, self.slope_deg[rows, cols])

        # Ground under and right around the rover has been driven on.
        near = np.hypot(xx - (x - self.origin[0] - cols.start * self.cell),
                        yy - (y - self.origin[1] - rows.start * self.cell))
        self.driven[rows, cols] |= near < g('driven_radius_m')
        driven = self.driven[rows, cols]

        cost = np.where(driven & ~step, 0, cost)
        self.cost[rows, cols] = cost.astype(np.int8)

    @staticmethod
    def _fill_pockets(seen, h):
        """Heights for unseen cells enclosed by seen ones; NaN everywhere else.

        Each pocket cell takes the mean of two straight-line interpolations
        between the nearest seen cells: one along its row, one along its column.
        """
        pocket = binary_fill_holes(seen) & ~seen
        out = np.full(h.shape, np.nan)
        if not pocket.any():
            return out
        total = np.zeros(h.shape)
        weight = np.zeros(h.shape)
        for axis in (0, 1):
            seen_l, h_l, pocket_l = (a if axis == 0 else a.T for a in (seen, h, pocket))
            total_l, weight_l = (a if axis == 0 else a.T for a in (total, weight))
            for i in np.flatnonzero(pocket_l.any(axis=1)):
                have = np.flatnonzero(seen_l[i])
                if len(have) < 2:
                    continue
                want = np.flatnonzero(pocket_l[i])
                want = want[(want > have[0]) & (want < have[-1])]
                total_l[i, want] += np.interp(want, have, h_l[i, have])
                weight_l[i, want] += 1
        ok = pocket & (weight > 0)
        out[ok] = total[ok] / weight[ok]
        return out

    def _publish(self):
        if self.origin is None:
            return
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.odom_frame
        msg.info.resolution = self.cell
        msg.info.width = self.n
        msg.info.height = self.n
        msg.info.origin.position.x = float(self.origin[0])
        msg.info.origin.position.y = float(self.origin[1])
        msg.info.origin.orientation.w = 1.0
        msg.data = array('b', self.cost.tobytes())
        self.cost_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LidarTerrain()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
