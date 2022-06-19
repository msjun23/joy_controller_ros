#!/usr/bin/env python3
import rospy
import cv2
import os

from std_msgs.msg import String
from sensor_msgs.msg import Joy, Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge, CvBridgeError
from actionlib_msgs.msg import GoalID


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
        rospy.Subscriber("/camera/color/image_raw", Image, self.Capture)
        self.pub_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.mode_pub = rospy.Publisher('/umbot_mode', String, queue_size=10)
        self.cancel_pub = rospy.Publisher('/move_base/cancel', GoalID, queue_size=1)

        self.cmd_vel = Twist()
        self.path = os.getcwd()
        
        rospy.on_shutdown(self.QuitHandler)
        rospy.spin()
        
    def JoyCallback(self, joy):
        if (joy.buttons[6] == 1 or joy.buttons[7] == 1):        # L1 or R1 is accelerater
            # Without L1 or R1, robot don't move
            self.cmd_vel.linear.x = joy.axes[1] / 4.0 #2.25         # 0.44 ~ -0.44
            self.cmd_vel.linear.y = joy.axes[0] / 4.0 #2.25         # 0.44 ~ -0.44
            self.cmd_vel.linear.z = 0.0
            
            self.cmd_vel.angular.x = 0.0
            self.cmd_vel.angular.y = 0.0
            self.cmd_vel.angular.z = joy.axes[2] / 4.0 #2.0          # 0.5 ~ -0.5
            
            if (joy.buttons[8] == 1 and joy.buttons[9] == 1):   # L2 and R2 is speed up
                self.cmd_vel.linear.x *= 2
                self.cmd_vel.linear.y *= 2
                
            self.pub_vel.publish(self.cmd_vel)
            rospy.loginfo('Running...')
        elif (joy.buttons[0] == 1):    # Stop
            self.cmd_vel.linear.x = 0.0
            self.cmd_vel.linear.y = 0.0
            self.cmd_vel.linear.z = 0.0
            
            self.cmd_vel.angular.x = 0.0
            self.cmd_vel.angular.y = 0.0
            self.cmd_vel.angular.z = 0.0
            
            cancel_msg = GoalID()
            
            self.pub_vel.publish(self.cmd_vel)
            self.cancel_pub.publish(cancel_msg)
            self.mode_pub.publish('stop')
        elif (joy.buttons[1] == 1):     # Cleaning mode
            self.mode_pub.publish('cleaning')
        elif (joy.buttons[3] == 1):     # Disinfection mode
            self.mode_pub.publish('disinfection')
        elif (joy.buttons[4] == 1):
            rospy.loginfo('sibal')
            # file_name = self.path + '/save/' + str(self.img.header.seq) + '.jpg'
            # file_name = '~/datasets/capture/' + str(self.img.header.seq) + '.jpg'
            # rospy.loginfo(file_name)
            # cv2.imwrite(file_name, self.cv_img)
        else:                                                   # Without any key input, don't publish any command
            pass
        
    def Capture(self, data):
        bridge = CvBridge()
        self.img = data
        self.cv_img = bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
            
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
    