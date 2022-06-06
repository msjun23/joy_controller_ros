#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

'''
Jetson AGX Xavier
axes: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
buttons: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

Left stick
    left/right: axes[0] (1.0 ~ -1.0)
    up/down:    axes[1] (1.0 ~ -1.0)
    click:      buttons[13]
Right stick
    left/right: axes[2] (1.0 ~ -1.0)
    up/down:    axes[3] (1.0 ~ -1.0)
    click:      buttons[14]

Left button
    left/right: axes[6] (1.0 / -1.0)
    up/down:    axes[7] (1.0 / -1.0)
Right button
    A:          buttons[0]
    B:          buttons[1]
    X:          buttons[3]
    Y:          buttons[4]
    
Front button
    L1:         buttons[6]
    L2:         buttons[8], axes[5] (-1.0)
    R1:         buttons[7]
    R2:         buttons[9], axes[4] (-1.0)

SELECT:         buttons[10]
START:          buttons[11]
HOME:           x
'''

class joy_node:
    def __init__(self):
        rospy.init_node('joy_node', anonymous=True)
        
        rospy.Subscriber("/joy", Joy, self.JoyCallback)
        self.pub_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        
        self.cmd_vel = Twist()
        
        rospy.on_shutdown(self.QuitHandler)
        rospy.spin()
        
    def JoyCallback(self, joy):
        # For Jetson AGX Xavier
        if (joy.buttons[6] == 1 or joy.buttons[7] == 1):        # L1 or R1 is accelerater
            self.cmd_vel.linear.x = joy.axes[1] / 4.0 #2.25         # 0.44 ~ -0.44
            self.cmd_vel.linear.y = joy.axes[0] / 4.0 #2.25         # 0.44 ~ -0.44
            self.cmd_vel.linear.z = 0.0
            
            self.cmd_vel.angular.x = 0.0
            self.cmd_vel.angular.y = 0.0
            self.cmd_vel.angular.z = joy.axes[2] / 4.0 #2.0          # 0.5 ~ -0.5
            self.pub_vel.publish(self.cmd_vel)
            rospy.loginfo('Running...')
        else:                                                   # Without L1 or R1, robot don't move
            self.cmd_vel.linear.x = 0.0
            self.cmd_vel.linear.y = 0.0
            self.cmd_vel.linear.z = 0.0
            
            self.cmd_vel.angular.x = 0.0
            self.cmd_vel.angular.y = 0.0
            self.cmd_vel.angular.z = 0.0
            self.pub_vel.publish(self.cmd_vel)
    
    def QuitHandler(self):
        rospy.loginfo('Shuting down process...')
        
        self.cmd_vel.linear.x = 0.0
        self.cmd_vel.linear.y = 0.0
        self.cmd_vel.linear.z = 0.0
        
        self.cmd_vel.angular.x = 0.0
        self.cmd_vel.angular.y = 0.0
        self.cmd_vel.angular.z = 0.0
        self.pub_vel.publish(self.cmd_vel)

if __name__ == '__main__':
    n = joy_node()
    