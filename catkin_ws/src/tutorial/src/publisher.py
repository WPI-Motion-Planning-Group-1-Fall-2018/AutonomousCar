#! /user/bin/env python

import rospy
from std_msgs.msg import String #Import the String message from the std_msgs package

rospy.init_node('topic_publisher')  #Initiate a Node name Topic Publisher
pub = rospy.Publisher('phrases', String, queue_size=10) #create a publisher object, that will publish on the "phrases"topic messages.

rate = rospy.Rate(2)  #Set a publish rate of 2hz
msg_str = String()  	#Create a var of type String
msg_str = "Hello World - Ros Tutorial"  #Initialze 'msg_str' variable

while not rospy.is_shutdown():   #Create a loop that will go until someone stops the program execution
	
	pub.publish(msg_str)
	rate.sleep()		#Make sure the rate maintains at 2 Hz
