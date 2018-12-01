#!/usr/bin/env python

import rospy
import roslib
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from gazebo_msgs.msg import ModelState
from gazebo_msgs.msg import ModelStates
import gazebo_msgs.msg
import time
from std_msgs.msg import String
from gazebo_msgs.srv import SetModelState
import math
import numpy as np


class Posepublisher():

     def __init__(self):

        rospy.init_node('Posen')
        # publish to /gazebo/mode_states
        p = rospy.Publisher('/gazebo/set_model_state',ModelState, queue_size = 5)

        rate = 10

        r = rospy.Rate(rate)

        # create the ModelState Variable
        ms = ModelState()


        listx = np.arange(0, 25, 1)
        sizeoflist = len(listx)

        while not rospy.is_shutdown():

            s = 0
            while s < sizeoflist :
                    #counter = counter + 1
                    ms.model_name = 'unit_box'
                    #pose components

                    #for position and orientation of object:
                    ms.pose.position.x = float(listx[s])
                    ms.pose.position.y = 15.0
                    ms.pose.position.z = 0.0
                    ms.pose.orientation.x = 0.0
                    ms.pose.orientation.y = 0.0
                    ms.pose.orientation.z = 0.0
                    ms.pose.orientation.w = 1.0
                    s += 1
                    p.publish(ms)
                    r.sleep()

if __name__ == '__main__':

        Posepublisher()

        rospy.loginfo("Posen node terminated")