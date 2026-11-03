std_msgs/Header header     # stamp: visit time, frame_id: camera
uint8 tag_id              # 1, 4, 7 or 9
# JPEG in which the tag is clearly visible:
sensor_msgs/CompressedImage image
