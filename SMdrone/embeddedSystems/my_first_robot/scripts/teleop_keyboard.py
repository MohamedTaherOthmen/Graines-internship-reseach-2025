#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import sys, select, termios, tty

# Key mapping
MOVE_BINDINGS = {
    'w': (1, 0),  # Forward
    's': (-1, 0), # Backward
    'a': (0, 1),  # Rotate Left
    'd': (0, -1), # Rotate Right
}

SPEED = 0.5
TURN_SPEED = 1.0

def get_key():
    """Capture keyboard input."""
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)
    rospy.init_node("teleop_keyboard")
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
    
    print("Use 'WASD' keys to control the robot. Press 'q' to quit.")

    try:
        while not rospy.is_shutdown():
            key = get_key()
            if key in MOVE_BINDINGS:
                twist = Twist()
                twist.linear.x = SPEED * MOVE_BINDINGS[key][0]
                twist.angular.z = TURN_SPEED * MOVE_BINDINGS[key][1]
                pub.publish(twist)
            elif key == 'q':
                break  # Quit
    except Exception as e:
        print(f"Error: {e}")
    finally:
        twist = Twist()  # Stop the robot
        pub.publish(twist)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
