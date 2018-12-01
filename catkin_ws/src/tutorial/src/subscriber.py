#! /usr/bin/env python

import rospy
from std_msgs.msg import String

def callback(msg):		#Define a function called 'callback' that recieves a parameter named 'msg'

	print msg.data		#Print the value 'data' inside the 'mg' parameter

rospy.init_node('topic_subscriber')	#Initiate a node topic_s
sub = rospy.Subscriber('/phrases', String, callback)	#Create a Subscriber object that will listen to the "phrases" topic and will call the
							#'Callback' function each time it reads something from the topic

rospy.spin()			#Create a loop that will keep the program in execution.
