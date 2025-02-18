#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
import sys
import select
import termios
import tty

class TeleopNode:
    def __init__(self):
        rospy.init_node('robot_teleop', anonymous=True)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.settings = termios.tcgetattr(sys.stdin)
        self.speed = 0.0
        self.turn = 0.0
        
        # Control parameters (adjust these as needed)
        self.linear_step = 0.1  # m/s
        self.angular_step = 0.5  # rad/s
        self.max_linear_speed = 1.0  # m/s
        self.max_angular_speed = 2.0  # rad/s

        self.msg = """
Control Your Robot!
---------------------------
z : Move forward
s : Move backward
q : Turn left
d : Turn right
b : Increase speed
n : Decrease speed
x : Stop
w : resume movement

"""

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run(self):
        print(self.msg)
        while not rospy.is_shutdown():
            key = self.get_key()
            if key == 'q':  # Move forward
                self.speed = min(self.speed + self.linear_step, self.max_linear_speed)
            elif key == 'd':  # Move backward
                self.speed = max(self.speed - self.linear_step, -self.max_linear_speed)
            elif key == 'z':  # Turn left
                self.turn = min(self.turn + self.angular_step, self.max_angular_speed)
            elif key == 's':  # Turn right
                self.turn = max(self.turn - self.angular_step, -self.max_angular_speed)
            elif key == 'b':  # Increase speed
                self.linear_step = min(self.linear_step + 0.05, 0.5)  # Cap speed increment
                print(f"Speed increment increased to {self.linear_step:.2f} m/s")
            elif key == 'n':  # Decrease speed
                self.linear_step = max(self.linear_step - 0.05, 0.05)  # Minimum speed increment
                print(f"Speed increment decreased to {self.linear_step:.2f} m/s")
            elif key == 'x':  # Stop
                self.speed = 0.0
                self.turn = 0.0
            elif key == 'w':  # Start (resume movement)
                self.speed = self.linear_step  # Resume with default speed
            elif key == '\x03':  # Ctrl+C to quit
                break

            twist = Twist()
            twist.linear.x = self.speed
            twist.angular.z = self.turn
            self.pub.publish(twist)
            
            print(f"Current speed: linear={self.speed:.2f} m/s, angular={self.turn:.2f} rad/s", end='\r')

        # Stop the robot before exiting
        twist = Twist()
        self.pub.publish(twist)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

if __name__ == '__main__':
    try:
        node = TeleopNode()
        node.run()
    except rospy.ROSInterruptException:
        pass