"""Record where the AprilTags are while you map the arena.

Run it next to SLAM while you drive around:

    ros2 run task_solution tag_mapper --ros-args -p use_sim_time:=true \
        -p output_file:=$HOME/arcx6_ws/src/PARC2026-ENG-ARCX6/ros2_ws/src/task_solution/maps/tags.yaml

Each time a tag is seen, its position and facing direction are converted to the
map frame. Many sightings are combined (median), and the file is rewritten every
few seconds and on Ctrl+C:

    tags:
      1: {x: 3.78, y: 2.0, z: 1.0, facing_yaw: -2.36, samples: 120}

facing_yaw is the direction the tag's printed face points (rad, from map +x).
A good spot to look at tag N from distance d is:
    x + d*cos(facing_yaw), y + d*sin(facing_yaw), robot yaw = facing_yaw + pi
"""

import math
import os

import cv2
import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
import tf2_ros

DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)


def detect_tags(gray, K, D, tag_size):
    """Return {id: (tvec, normal)} in the camera optical frame.
    tvec: tag centre. normal: unit vector out of the printed face (towards the viewer)."""
    if hasattr(cv2.aruco, 'ArucoDetector'):          # OpenCV >= 4.7
        corners, ids, _ = cv2.aruco.ArucoDetector(DICT).detectMarkers(gray)
    else:                                            # OpenCV 4.6 (Ubuntu 24.04)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, DICT)
    out = {}
    if ids is None:
        return out
    h = tag_size / 2.0
    # Corner order from the detector: top-left, top-right, bottom-right, bottom-left.
    obj = np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], dtype=np.float64)
    for c, tag_id in zip(corners, ids.ravel()):
        ok, rvec, tvec = cv2.solvePnP(obj, c.reshape(4, 2).astype(np.float64), K, D,
                                      flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if not ok:
            continue
        R, _ = cv2.Rodrigues(rvec)
        out[int(tag_id)] = (tvec.ravel(), R[:, 2])   # tag z axis = out of the face
    return out


def quat_to_matrix(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class TagMapper(Node):

    def __init__(self):
        super().__init__('tag_mapper')
        self.declare_parameter('image_topic', '/top_camera_color/image_raw')
        self.declare_parameter('camera_info_topic', '/top_camera_color/camera_info')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('tag_size', 0.165)          # black square edge, m
        self.declare_parameter('tag_ids', [1, 4, 7, 9])    # ignore anything else
        self.declare_parameter('max_range', 2.5)           # m: farther sightings are too noisy
        self.declare_parameter('output_file', os.path.expanduser('~/tags.yaml'))
        self.declare_parameter('save_period', 3.0)         # s

        self.map_frame = self.get_parameter('map_frame').value
        self.tag_size = float(self.get_parameter('tag_size').value)
        self.allowed = {int(i) for i in self.get_parameter('tag_ids').value}
        self.max_range = float(self.get_parameter('max_range').value)
        self.output = os.path.expanduser(self.get_parameter('output_file').value)

        self.bridge = CvBridge()
        self.K = None
        self.D = None
        self.samples = {}      # id -> list of (x, y, z, yaw)
        self.dirty = False

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_subscription(CameraInfo, self.get_parameter('camera_info_topic').value,
                                 self.on_info, qos_profile_sensor_data)
        self.create_subscription(Image, self.get_parameter('image_topic').value,
                                 self.on_image, qos_profile_sensor_data)
        self.create_timer(float(self.get_parameter('save_period').value), self.save)
        self.get_logger().info(f'Recording tags {sorted(self.allowed)} -> {self.output}')

    def on_info(self, msg):
        if self.K is None:
            self.K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
            self.D = np.array(msg.d, dtype=np.float64)
            self.get_logger().info('Camera intrinsics received.')

    def on_image(self, msg):
        if self.K is None:
            return
        gray = self.bridge.imgmsg_to_cv2(msg, 'mono8')
        found = {i: v for i, v in detect_tags(gray, self.K, self.D, self.tag_size).items()
                 if i in self.allowed and np.linalg.norm(v[0]) <= self.max_range}
        if not found:
            return
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, msg.header.frame_id, Time.from_msg(msg.header.stamp),
                timeout=Duration(seconds=0.1))
        except tf2_ros.TransformException as e:
            self.get_logger().warn(f'No {self.map_frame} transform yet: {e}',
                                   throttle_duration_sec=5.0)
            return
        R = quat_to_matrix(tf.transform.rotation)
        t = tf.transform.translation
        t = np.array([t.x, t.y, t.z])
        for tag_id, (pos_cam, normal_cam) in found.items():
            p = R @ pos_cam + t
            n = R @ normal_cam
            yaw = math.atan2(n[1], n[0])
            lst = self.samples.setdefault(tag_id, [])
            if not lst:
                self.get_logger().info(f'Tag {tag_id} first seen at ({p[0]:.2f}, {p[1]:.2f}).')
            lst.append((p[0], p[1], p[2], yaw))
            if len(lst) > 300:
                lst.pop(0)
            self.dirty = True

    def estimate(self, lst):
        a = np.array(lst)
        yaw = math.atan2(np.median(np.sin(a[:, 3])), np.median(np.cos(a[:, 3])))
        return {'x': round(float(np.median(a[:, 0])), 3),
                'y': round(float(np.median(a[:, 1])), 3),
                'z': round(float(np.median(a[:, 2])), 3),
                'facing_yaw': round(yaw, 4),
                'samples': len(lst)}

    def save(self):
        if not self.dirty:
            return
        data = {'frame': self.map_frame,
                'tags': {i: self.estimate(l) for i, l in sorted(self.samples.items())}}
        os.makedirs(os.path.dirname(self.output) or '.', exist_ok=True)
        with open(self.output, 'w') as f:
            yaml.safe_dump(data, f, sort_keys=False)
        self.dirty = False
        missing = sorted(self.allowed - set(self.samples))
        self.get_logger().info(
            f'Saved {len(self.samples)} tag(s) to {self.output}'
            + (f'; not seen yet: {missing}' if missing else ''))


def main(args=None):
    rclpy.init(args=args)
    node = TagMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
