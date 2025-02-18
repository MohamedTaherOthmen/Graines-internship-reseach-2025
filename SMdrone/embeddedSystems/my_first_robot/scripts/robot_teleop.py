#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
import sys
import select
import termios
import tty
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState

class TeleopNode:
    def __init__(self):
        rospy.init_node('robot_teleop', anonymous=True)
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.settings = termios.tcgetattr(sys.stdin)
        self.speed = 0.0
        self.turn = 0.0

        # Contrôles et limites
        self.linear_step = 0.1  # m/s
        self.angular_step = 0.5  # rad/s
        self.max_linear_speed = 1.0  # m/s
        self.max_angular_speed = 2.0  # rad/s

        self.last_speed = 0.0  # Sauvegarde la dernière vitesse

        # Attendre que le service Gazebo soit disponible
        rospy.wait_for_service('/gazebo/set_model_state')
        self.set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

        self.msg = """
Contrôlez Votre Robot !
---------------------------
z : Avancer
s : Reculer
q : Tourner à gauche
d : Tourner à droite
b : Augmenter l'incrément de vitesse
n : Diminuer l'incrément de vitesse
x : Stop
w : Reprendre dernier mouvement
r : Réinitialiser position du robot

CTRL+C pour quitter
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

    def reset_robot_position(self):
        """Réinitialise la position du robot au point d'origine"""
        state = ModelState()
        state.model_name = "my_first_robot"  # Assure-toi que c'est bien le nom de ton robot dans Gazebo !
        state.pose.position.x = 0.0
        state.pose.position.y = 0.0
        state.pose.position.z = 0.0
        state.pose.orientation.x = 0.0
        state.pose.orientation.y = 0.0
        state.pose.orientation.z = 0.0
        state.pose.orientation.w = 1.0  # Quaternion valide
        state.twist.linear.x = 0.0
        state.twist.angular.z = 0.0
        state.reference_frame = "world"

        try:
            self.set_model_state(state)
            print("\nRobot réinitialisé à sa position d'origine.")
        except rospy.ServiceException as e:
            print(f"\nErreur lors de la réinitialisation : {e}")

    def run(self):
        print(self.msg)
        while not rospy.is_shutdown():
            key = self.get_key()

            if key == 'z':  
                self.speed = min(self.speed + self.linear_step, self.max_linear_speed)
                self.turn = 0.0
            elif key == 's':  
                self.speed = max(self.speed - self.linear_step, -self.max_linear_speed)
                self.turn = 0.0
            elif key == 'q':  
                self.turn = min(self.turn + self.angular_step, self.max_angular_speed)
            elif key == 'd':  
                self.turn = max(self.turn - self.angular_step, -self.max_angular_speed)
            elif key == 'b':  
                self.linear_step = min(self.linear_step + 0.05, 0.5)
                print(f"\nIncrément de vitesse augmenté à {self.linear_step:.2f} m/s")
            elif key == 'n':  
                self.linear_step = max(self.linear_step - 0.05, 0.05)
                print(f"\nIncrément de vitesse diminué à {self.linear_step:.2f} m/s")
            elif key == 'x':  
                self.speed = 0.0
                self.turn = 0.0
            elif key == 'w':  
                self.speed = self.last_speed
            elif key == 'r':  # Réinitialisation du robot
                self.reset_robot_position()
                self.speed = 0.0
                self.turn = 0.0
            elif key == '\x03':  
                break

            if key in ['z', 's', 'w']:
                self.last_speed = self.speed

            twist = Twist()
            twist.linear.x = self.speed
            twist.angular.z = self.turn
            self.pub.publish(twist)

            print(f"\rVitesse actuelle : linéaire={self.speed:.2f} m/s, angulaire={self.turn:.2f} rad/s      ", end='', flush=True)

        twist = Twist()
        self.pub.publish(twist)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

if __name__ == '__main__':
    try:
        node = TeleopNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
