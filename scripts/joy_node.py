#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

'''
axes: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
buttons: [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

Left stick
    left/right: axes[0] (1.0 ~ -1.0)
    up/down:    axes[1] (1.0 ~ -1.0)
    click:      buttons[9]
Right stick
    left/right: axes[3] (1.0 ~ -1.0)
    up/down:    axes[4] (1.0 ~ -1.0)
    click:      buttons[10]

Left button
    left/right: axes[6] (1.0 / -1.0)
    up/down:    axes[7] (1.0 / -1.0)
Right button
    A:          buttons[0]
    B:          buttons[1]
    X:          buttons[2]
    Y:          buttons[3]
    
Front button
    L1:         buttons[4]
    L2:         axes[2] (-1.0)
    R1:         buttons[5]
    R2:         axes[5] (-1.0)

SELECT:         buttons[6]
START:          buttons[7]
HOME:           buttons[8]
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
        # if (joy.buttons[4] == 1 or joy.buttons[5] == 1):        # L1 or R1 is accelerater
        #     self.cmd_vel.linear.x = joy.axes[1] / 2   # 0.5 ~ -0.5
        #     self.cmd_vel.linear.y = joy.axes[0] / 2   # 0.5 ~ -0.5
        #     self.cmd_vel.linear.z = 0.0
            
        #     self.cmd_vel.angular.x = 0.0
        #     self.cmd_vel.angular.y = 0.0
        #     self.cmd_vel.angular.z = joy.axes[3]
        #     self.pub_vel.publish(self.cmd_vel)
        #     rospy.loginfo('Running...')
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
    