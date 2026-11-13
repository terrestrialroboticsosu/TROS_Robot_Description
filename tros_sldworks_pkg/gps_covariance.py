"""Republish a NavSatFix with a position covariance filled in.

Gazebo's NavSat sensor publishes fixes with an all-zero covariance, which
robot_localization treats as a near-perfect measurement. This node stamps a
diagonal covariance on each fix so the global EKF weights GPS sensibly.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix


class GpsCovariance(Node):

    def __init__(self):
        super().__init__('gps_covariance')
        self.declare_parameter('horizontal_stddev', 1.2)  # metres
        self.declare_parameter('vertical_stddev', 2.5)    # metres
        self.pub = self.create_publisher(NavSatFix, 'gps/fix', 10)
        self.sub = self.create_subscription(
            NavSatFix, 'gps/fix_raw', self.on_fix, 10)

    def on_fix(self, msg):
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
