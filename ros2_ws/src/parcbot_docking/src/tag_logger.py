#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import json
import os
from ament_index_python.packages import get_package_share_directory

from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

class TagWorldCoordinateLogger(Node):
    def __init__(self):
        super().__init__('tag_world_coordinate_logger')
        
        # Initialize TF2 buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Target tags to map (including tag 0)
        self.target_tag_ids = [0, 1, 4, 7, 9]
        
        self.mapped_tags = {}
        
        # Save directly inside your package's src directory
        package_share = get_package_share_directory('parcbot_docking')  
        self.output_file = os.path.join(package_share, 'src', 'mapped_tag_world_coordinates.json')
        
        # Timer to periodically check TF tree for tag positions (e.g., every 0.5 seconds)
        self.timer = self.create_timer(0.5, self.lookup_tag_transforms)
        
        self.get_logger().info(f"TF Tag World Logger Initialized. Saving to: {self.output_file}")

    def lookup_tag_transforms(self):
        updated = False
        target_reference_frame = 'map' # Change to 'odom' if your world root uses odom
        
        detected_in_this_cycle = []

        for tag_id in self.target_tag_ids:
            child_frame = f'tag_{tag_id}'
            
            try:
                # Look up the latest transform from the global frame to the tag frame
                trans = self.tf_buffer.lookup_transform(
                    target_reference_frame,
                    child_frame,
                    rclpy.time.Time()
                )
                
                position = trans.transform.translation
                rotation = trans.transform.rotation
                
                coord_data = {
                    "id": tag_id,
                    "reference_frame": target_reference_frame,
                    "position": {
                        "x": position.x,
                        "y": position.y,
                        "z": position.z
                    },
                    "orientation": {
                        "x": rotation.x,
                        "y": rotation.y,
                        "z": rotation.z,
                        "w": rotation.w
                    }
                }
                
                self.mapped_tags[str(tag_id)] = coord_data
                updated = True
                detected_in_this_cycle.append(tag_id)
                
                # Terminal printout for successful detection
                self.get_logger().info(
                    f"[DETECTED & MAPPED] Tag {tag_id} -> X: {position.x:.3f}, Y: {position.y:.3f}, Z: {position.z:.3f}"
                )
                
            except (LookupException, ConnectivityException, ExtrapolationException):
                pass

        # If no tags were found in this check, print a quiet status line optionally (or keep it clean)
        if not detected_in_this_cycle:
            self.get_logger().debug("Scanning for tags in TF tree...", throttle_duration_sec=2.0)

        # Save to the package's src directory if new/updated coordinates were found
        if updated:
            self.save_coordinates()

    def save_coordinates(self):
        try:
            with open(self.output_file, 'w') as f:
                json.dump(self.mapped_tags, f, indent=4)
        except Exception as e:
            self.get_logger().error(f"Failed to save coordinates to file: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = TagWorldCoordinateLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()