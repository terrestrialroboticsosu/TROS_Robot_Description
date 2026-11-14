"""Detect ArUco markers in a camera image and publish their poses.

Ported from Team Mountaineers' ros2_aruco
(https://github.com/team-mountaineers-urc-2025/ros2_aruco), itself a fork of
JMU-ROBOTICS-VIVA's ros2_aruco. MIT licence:

    Copyright (c) 2020 JMU-ROBOTICS-VIVA

    Permission is hereby granted, free of charge, to any person obtaining a
    copy of this software and associated documentation files (the
    "Software"), to deal in the Software without restriction, including
    without limitation the rights to use, copy, modify, merge, publish,
    distribute, sublicense, and/or sell copies of the Software, and to permit
    persons to whom the Software is furnished to do so, subject to the
    following conditions:

    The above copyright notice and this permission notice shall be included
    in all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
    OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
    MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN
    NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
    DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
    OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE
    USE OR OTHER DEALINGS IN THE SOFTWARE.

Changes from that version:
- Works with OpenCV 4.6 (Ubuntu 24.04's own) through 5: it uses the newer
  ArucoDetector class when it exists and the older functions otherwise, and
  copes with ids coming back as (N, 1) or (N,). The original crashed on 5.
- Positions are published as OpenCV measures them, in the camera's optical
  frame (x right, y down, z forward). The original swapped x and y, so after
  a TF transform the marker landed about 1.5 m from where it really was.
- Each marker gets its own id. The original gave every marker in an image the
  id of the first one.
- The aruco_dictionary_id parameter is used (it was ignored).
- Output is a standard vision_msgs/Detection3DArray (class_id = marker id)
  instead of a custom message, plus a PoseArray for RViz.

Subscribes: image_topic (sensor_msgs/Image), camera_info_topic (CameraInfo)
Publishes:  aruco/detections (vision_msgs/Detection3DArray)
            aruco/poses (geometry_msgs/PoseArray)
"""
import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose, PoseArray
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, Image
from vision_msgs.msg import Detection3D, Detection3DArray, ObjectHypothesisWithPose


class ArucoDetector(Node):

    def __init__(self):
        super().__init__('aruco_detector')
        self.declare_parameter('marker_size', 0.15)          # black square, metres
        self.declare_parameter('aruco_dictionary_id', 'DICT_4X4_50')
        self.declare_parameter('image_topic', '/chassis_camera/image_raw')
        self.declare_parameter('camera_info_topic', '/chassis_camera/camera_info')
        self.declare_parameter('camera_frame', '')           # '' = use the image's frame

        self.marker_size = self.get_parameter('marker_size').value
        name = self.get_parameter('aruco_dictionary_id').value
        if not hasattr(cv2.aruco, name):
            raise ValueError(f'Unknown ArUco dictionary {name}')
        dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, name))
        if hasattr(cv2.aruco, 'ArucoDetector'):             # OpenCV 4.7 and later
            self.detect = cv2.aruco.ArucoDetector(
                dictionary, cv2.aruco.DetectorParameters()).detectMarkers
        else:                                               # OpenCV 4.6 and earlier
            params = cv2.aruco.DetectorParameters_create()
            self.detect = lambda image: cv2.aruco.detectMarkers(   # noqa: E731
                image, dictionary, parameters=params)
        half = self.marker_size / 2.0
        # Corner order OpenCV returns: top-left, top-right, bottom-right, bottom-left.
        self.corners_3d = np.array([[-half, half, 0], [half, half, 0],
                                    [half, -half, 0], [-half, -half, 0]], np.float32)

        self.camera_matrix = None
        self.distortion = None
        self.bridge = CvBridge()
        self.camera_frame = self.get_parameter('camera_frame').value

        self.create_subscription(CameraInfo, self.get_parameter('camera_info_topic').value,
                                 self._on_info, qos_profile_sensor_data)
        self.create_subscription(Image, self.get_parameter('image_topic').value,
                                 self._on_image, qos_profile_sensor_data)
        self.detections_pub = self.create_publisher(Detection3DArray, 'aruco/detections', 10)
        self.poses_pub = self.create_publisher(PoseArray, 'aruco/poses', 10)
        self.get_logger().info(f'Looking for {name} markers of {self.marker_size} m')

    def _on_info(self, msg):
        self.camera_matrix = np.array(msg.k, float).reshape(3, 3)
        self.distortion = np.array(msg.d, float)

    def _on_image(self, msg):
        if self.camera_matrix is None:
            self.get_logger().warn('No camera_info yet', throttle_duration_sec=5.0)
            return
        gray = self.bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        corners, ids, _ = self.detect(gray)
        if ids is None or len(ids) == 0:
            return

        header = msg.header
        if self.camera_frame:
            header.frame_id = self.camera_frame
        detections = Detection3DArray(header=header)
        poses = PoseArray(header=header)
        for marker_id, image_corners in zip(np.asarray(ids).ravel(), corners):
            ok, rvec, tvec = cv2.solvePnP(self.corners_3d, image_corners.reshape(4, 2),
                                          self.camera_matrix, self.distortion,
                                          flags=cv2.SOLVEPNP_IPPE_SQUARE)
            if not ok:
                continue
            x, y, z, w = Rotation.from_rotvec(rvec.ravel()).as_quat()
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = map(float, tvec.ravel())
            pose.orientation.x, pose.orientation.y = float(x), float(y)
            pose.orientation.z, pose.orientation.w = float(z), float(w)

            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = str(int(marker_id))
            hypothesis.hypothesis.score = 1.0
            hypothesis.pose.pose = pose
            detection = Detection3D(header=header)
            detection.id = str(int(marker_id))
            detection.results.append(hypothesis)
            detection.bbox.center = pose
            detection.bbox.size.x = detection.bbox.size.y = self.marker_size
            detections.detections.append(detection)
            poses.poses.append(pose)

        self.detections_pub.publish(detections)
        self.poses_pub.publish(poses)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
