#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import math
from math import sin, cos, pi,tan, atan2

#define a global
pose = [0.0,0.0,0.0,0.0]

def callback(msg):
    #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

    global pose
    x=msg.pose.pose.position.x
    y=msg.pose.pose.position.y
    q0 = msg.pose.pose.orientation.w
    q1 = msg.pose.pose.orientation.x
    q2 = msg.pose.pose.orientation.y
    q3 = msg.pose.pose.orientation.z
    theta=atan2(2*(q0*q3+q1*q2),1-2*(q2*q2+q3*q3))
    pose=[x,y,theta]
    rospy.loginfo(pose)

def carposlistener():


    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('carposlistener', anonymous=True)

    rospy.Subscriber("base_pose_ground_truth", Odometry, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    carposlistener()
