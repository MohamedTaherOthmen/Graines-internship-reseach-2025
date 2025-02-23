#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan

class ObstacleDetector:
    def __init__(self):
        rospy.init_node("obstacle_detector")

        # Subscribe to the LiDAR scan topic
        self.scan_sub = rospy.Subscriber("/scan", LaserScan, self.scan_callback)

        # Set obstacle detection threshold (meters)
        self.obstacle_threshold = 0.5  # Adjust as needed

    def scan_callback(self, scan):
        ranges = scan.ranges
        num_readings = len(ranges)

        if num_readings == 0:
            rospy.logwarn("No LiDAR data received!")
            return

        # Define directional sectors
        left_sector = ranges[int(num_readings * 3 / 4):] + ranges[:int(num_readings / 4)]  # -135° to -45°
        front_sector = ranges[int(num_readings / 4):int(num_readings * 3 / 4)]  # -45° to 45°
        right_sector = ranges[int(num_readings * 3 / 4):] + ranges[:int(num_readings / 4)]  # 45° to 135°
        back_sector = ranges[int(num_readings / 2):]  # 135° to -135°

        # Check for obstacles
        left_clear = all(r > self.obstacle_threshold or r == 0 for r in left_sector)
        front_clear = all(r > self.obstacle_threshold or r == 0 for r in front_sector)
        right_clear = all(r > self.obstacle_threshold or r == 0 for r in right_sector)
        back_clear = all(r > self.obstacle_threshold or r == 0 for r in back_sector)

        # Print obstacle status
        left_status = "Clear" if left_clear else "Obstacle Detected"
        front_status = "Clear" if front_clear else "Obstacle Detected"
        right_status = "Clear" if right_clear else "Obstacle Detected"
        back_status = "Clear" if back_clear else "Obstacle Detected"

        rospy.loginfo(f"Left: {left_status} | Front: {front_status} | Right: {right_status} | Back: {back_status}")

if __name__ == "__main__":
    detector = ObstacleDetector()
    rospy.spin()

