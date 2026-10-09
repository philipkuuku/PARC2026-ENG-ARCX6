import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.duration import Duration

from sensor_msgs.msg import Image, PointCloud2
from sensor_msgs_py import point_cloud2
from geometry_msgs.msg import PointStamped

import message_filters
import tf2_ros
import tf2_geometry_msgs  # noqa: F401  (registers PointStamped transform support)

from cv_bridge import CvBridge
import cv2
import numpy as np


class GreenDetector(Node):

    def __init__(self):
        super().__init__('green_detector')

        self.bridge = CvBridge()

        #  Parameters 
        # OpenCV HSV ranges: H 0-179, S 0-255, V 0-255
        self.declare_parameter('image_topic', '/top_camera_color/image_raw')
        self.declare_parameter('cloud_topic', '/top_camera_depth/points')
        self.declare_parameter('target_frame', 'map')

        self.declare_parameter('h_low', 35)
        self.declare_parameter('h_high', 85)
        self.declare_parameter('s_low', 80)
        self.declare_parameter('s_high', 255)
        self.declare_parameter('v_low', 50)
        self.declare_parameter('v_high', 255)
        self.declare_parameter('min_area', 30.0)
        self.declare_parameter('min_circularity', 0.1)  # 1.0 = perfect circle
        #self.declare_parameter('max_area', 5000)
        self.declare_parameter('kernel_size', 1)        # 1 = no morphological cleanup

        self.declare_parameter('patch_half_size', 6)    
        self.declare_parameter('min_valid_points', 8)
        self.declare_parameter('max_range', 14.0)       # metres

        # TF 
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        #  Subscriptions
        image_topic = self.get_parameter('image_topic').value
        cloud_topic = self.get_parameter('cloud_topic').value

        img_sub = message_filters.Subscriber(
            self, Image, image_topic, qos_profile=qos_profile_sensor_data
        )
        cloud_sub = message_filters.Subscriber(
            self, PointCloud2, cloud_topic, qos_profile=qos_profile_sensor_data
        )
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [img_sub, cloud_sub], queue_size=10, slop=0.1
        )
        self.sync.registerCallback(self.synced_callback)

        # Publishers 
        self.debug_pub = self.create_publisher(
            Image, '/green_detector/debug_image', 10
        )
        self.cam_pub = self.create_publisher(
            PointStamped, '/green_detector/target_camera', 10
        )
        self.map_pub = self.create_publisher(
            PointStamped, '/green_detector/target_map', 10
        )

        self.get_logger().info(
            f'Green detector started | image: {image_topic} | cloud: {cloud_topic}'
        )

    
    def detect(self, frame):
        """Return (mask, best) where best = (area, (u, v), contour, circularity) or None."""
        p = self.get_parameter

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        lower = np.array([p('h_low').value, p('s_low').value, p('v_low').value])
        upper = np.array([p('h_high').value, p('s_high').value, p('v_high').value])
        mask = cv2.inRange(hsv, lower, upper)

        k = int(p('kernel_size').value)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        min_area = p('min_area').value
        
        min_circ = p('min_circularity').value

        best = None
        self.last_largest = 0.0
        self.last_contour_count = len(contours)
        for c in contours:
            area = cv2.contourArea(c)
            self.last_largest = max(self.last_largest, area)
            if area < min_area:
                continue

            perimeter = cv2.arcLength(c, True)
            if perimeter == 0:
                continue
            circularity = 4.0 * np.pi * area / (perimeter ** 2)
            if circularity < min_circ:
                continue

            M = cv2.moments(c)
            if M['m00'] == 0:
                continue
            u = int(M['m10'] / M['m00'])
            v = int(M['m01'] / M['m00'])

            if best is None or area > best[0]:
                best = (area, (u, v), c, circularity)

        return mask, best
    def lookup_3d(self, cloud_msg, u, v):
        """Median XYZ of a small patch around pixel (u, v) in an organized cloud.
        Returns np.array([x, y, z]) in the cloud's frame, or None."""
        half = int(self.get_parameter('patch_half_size').value)
        h, w = cloud_msg.height, cloud_msg.width

        # read_points expects flat indices into the organized cloud: v * width + u
        uvs = [
            vv * w + uu
            for vv in range(max(0, v - half), min(h, v + half + 1))
            for uu in range(max(0, u - half), min(w, u + half + 1))
        ]
        if not uvs:
            return None

        pts = point_cloud2.read_points(
            cloud_msg, field_names=('x', 'y', 'z'), skip_nans=False, uvs=uvs
        )
        # Works whether read_points returns a generator of tuples or a structured array
        arr = np.array([[pt[0], pt[1], pt[2]] for pt in pts], dtype=float)
        if arr.size == 0:
            return None

        arr = arr[np.isfinite(arr).all(axis=1)]
        if len(arr) < int(self.get_parameter('min_valid_points').value):
            return None

        return np.median(arr, axis=0)

    def synced_callback(self, img_msg, cloud_msg):
        frame = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='bgr8')
        mask, best = self.detect(frame)

        debug = frame.copy()

        if best is None:
            if self.last_contour_count == 0:
                self.get_logger().info(
                    'No green pixels in mask', throttle_duration_sec=2.0
                )
            else:
                self.get_logger().info(
                    f'{self.last_contour_count} green blob(s) in mask, '
                    f'largest area={self.last_largest:.1f}px, none passed '
                    f'min_area/min_circularity',
                    throttle_duration_sec=2.0
                )

        if best is not None:
            area, (u, v), contour, circ = best
            cv2.drawContours(debug, [contour], -1, (0, 0, 255), 2)
            cv2.circle(debug, (u, v), 5, (255, 0, 0), -1)
            label = f'({u},{v}) A={area:.0f} C={circ:.2f}'

            xyz = self.lookup_3d(cloud_msg, u, v)

            if xyz is None:
                self.get_logger().warn(
                    f'Circle seen at ({u},{v}) but no valid depth there',
                    throttle_duration_sec=1.0
                )
            elif np.linalg.norm(xyz) > self.get_parameter('max_range').value:
                self.get_logger().info(
                    f'Circle too far ({np.linalg.norm(xyz):.1f} m), ignoring',
                    throttle_duration_sec=1.0
                )
            else:
                label += f' d={np.linalg.norm(xyz):.2f}m'
                self.publish_target(cloud_msg.header, xyz, u, v, area)

            cv2.putText(
                debug, label, (10, 25),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2
            )

        mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        combined = np.hstack((debug, mask_bgr))
        out = self.bridge.cv2_to_imgmsg(combined, encoding='bgr8')
        out.header = img_msg.header
        self.debug_pub.publish(out)

    def publish_target(self, header, xyz, u, v, area):
        pt = PointStamped()
        pt.header = header  # frame_id = cloud frame, stamp = cloud stamp
        pt.point.x = float(xyz[0])
        pt.point.y = float(xyz[1])
        pt.point.z = float(xyz[2])
        self.cam_pub.publish(pt)

        target_frame = self.get_parameter('target_frame').value
        try:
            pt_map = self.tf_buffer.transform(
                pt, target_frame, timeout=Duration(seconds=0.2)
            )
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.get_logger().warn(
                f'TF to {target_frame} failed: {e}', throttle_duration_sec=2.0
            )
            return

        self.map_pub.publish(pt_map)
        self.get_logger().info(
            f'GREEN: pixel=({u},{v}) area={area:.0f} | '
            f'{target_frame}: x={pt_map.point.x:.2f} '
            f'y={pt_map.point.y:.2f} z={pt_map.point.z:.2f}',
            throttle_duration_sec=1.0
        )


def main(args=None):
    rclpy.init(args=args)
    node = GreenDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()