"""Turn Gazebo's perfect GPS fix into one that errs like a real receiver.

Gazebo's NavSat sensor is set to publish the true position (gps/fix_raw).
This node adds the error and the covariance and republishes it on gps/fix.

A standalone (non-RTK) receiver is not wrong by a fresh random amount on each
fix. Most of its error is an offset that changes slowly, so consecutive fixes
agree with each other much better than they agree with the truth. The model
here is the sum of three parts, per axis:

  slow offset   satellite orbit/clock and atmosphere errors; about a metre,
                wandering over many minutes
  fast offset   signal reflections (multipath); a few decimetres, changing
                over tens of seconds
  noise         what is left fix to fix; a few centimetres

Each offset is a first-order Gauss-Markov process: a random walk that is
pulled back towards zero, so it stays near its stated size and takes about
its stated time to change. The defaults add up to about 1.1 m per horizontal
axis (roughly 1.3 m CEP), in line with the 1.5 m CEP class figure quoted for
common single-band receivers with SBAS. They are typical values, not
measurements of the rover's own receiver; set the parameters from a log of
the real one, left stationary for an hour, when that is available.

The covariance stamped on the fix is what the receiver would claim for
itself. Like a real one, it says nothing about the error being correlated in
time, so a filter that trusts it will follow the slow offset.
"""
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import NavSatFix

EARTH_RADIUS_M = 6378137.0


class GaussMarkov:
    """A 3-axis (east, north, up) offset that wanders with a set size and time."""

    def __init__(self, stddev, time_s, rng):
        self.stddev = np.asarray(stddev, float)
        self.time_s = time_s
        self.rng = rng
        self.value = rng.normal(0.0, 1.0, 3) * self.stddev

    def step(self, dt):
        keep = math.exp(-dt / self.time_s)
        self.value = (self.value * keep
                      + self.rng.normal(0.0, 1.0, 3) * self.stddev * math.sqrt(1.0 - keep * keep))
        return self.value


class GpsCovariance(Node):

    def __init__(self):
        super().__init__('gps_covariance')
        # Error actually applied (metres are per horizontal axis).
        self.declare_parameter('slow_offset_stddev', 1.1)
        self.declare_parameter('slow_offset_time_s', 600.0)
        self.declare_parameter('fast_offset_stddev', 0.3)
        self.declare_parameter('fast_offset_time_s', 30.0)
        self.declare_parameter('noise_stddev', 0.05)
        self.declare_parameter('vertical_scale', 1.7)   # vertical error / horizontal
        self.declare_parameter('seed', -1)              # -1: different every run
        # Accuracy the receiver reports about itself.
        self.declare_parameter('horizontal_stddev', 1.2)
        self.declare_parameter('vertical_stddev', 2.5)

        seed = self.get_parameter('seed').value
        self.rng = np.random.default_rng(None if seed < 0 else seed)
        self.axes = np.array([1.0, 1.0, self.get_parameter('vertical_scale').value])
        self.slow = GaussMarkov(self.get_parameter('slow_offset_stddev').value * self.axes,
                                self.get_parameter('slow_offset_time_s').value, self.rng)
        self.fast = GaussMarkov(self.get_parameter('fast_offset_stddev').value * self.axes,
                                self.get_parameter('fast_offset_time_s').value, self.rng)
        self.last_stamp = None

        self.pub = self.create_publisher(NavSatFix, 'gps/fix', 10)
        self.sub = self.create_subscription(
            NavSatFix, 'gps/fix_raw', self.on_fix, 10)

    def on_fix(self, msg):
        stamp = Time.from_msg(msg.header.stamp).nanoseconds * 1e-9
        dt = 0.0 if self.last_stamp is None else max(0.0, stamp - self.last_stamp)
        self.last_stamp = stamp

        noise = self.rng.normal(0.0, 1.0, 3) * self.get_parameter('noise_stddev').value * self.axes
        east, north, up = self.slow.step(dt) + self.fast.step(dt) + noise

        msg.longitude += math.degrees(
            east / (EARTH_RADIUS_M * math.cos(math.radians(msg.latitude))))
        msg.latitude += math.degrees(north / EARTH_RADIUS_M)
        msg.altitude += up

        h_var = self.get_parameter('horizontal_stddev').value ** 2
        v_var = self.get_parameter('vertical_stddev').value ** 2
        msg.position_covariance = [
            h_var, 0.0, 0.0,
            0.0, h_var, 0.0,
            0.0, 0.0, v_var,
        ]
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GpsCovariance()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
