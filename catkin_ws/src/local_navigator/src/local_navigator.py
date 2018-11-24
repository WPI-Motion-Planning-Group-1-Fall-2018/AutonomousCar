#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry, OccupancyGrid, MapMetaData 
from std_msgs.msg import String
import math
from math import sin, cos, pi,tan, atan2
from geometry_msgs.msg import Twist


#define a global
pose = [0.0,0.0,0.0,0.0]

class LocalNav:

    def callbackCarPos(self,msg):

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


    def callbackOccGrid(self,msg):
        #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
        width=msg.info.width
        height=msg.info.width
        origin=msg.info.origin
        resolution = msg.info.resolution
        maptime = msg.info.map_load_time
        
        obstacle = False
        
        i = 0
        
        #Check to see what is the greatest value 
        for x in range (0, width):
            for y in range (0, height):
                if i < 45300:
                    cellval = msg.data[i]
                    if (cellval == 100):
                        obstacle = True 
                i = i + 1
                
        if obstacle:
            rospy.loginfo("Obstacle found")
        
        #rospy.loginfo("Width: %s and Height: %s", width, height)
        #rospy.loginfo("Origin: %s", origin)



    def __init__(self):

        #self.pose = 

        #self.pub_msg = Twist() # replace with John's type
        #self.heading = #someinit val
        #self.send_msg_ = False
        #self.send_ = False
        #self.x1 = #init val
        #self.y1 = #init val

        # subscribe for the car pose ground truth position
        rospy.Subscriber("base_pose_ground_truth", Odometry, self.callbackCarPos, queue_size =1)

        # subscribe for the local cost map that John is generating

        rospy.Subscriber("local_costmap", OccupancyGrid, self.callbackOccGrid, queue_size=1)
    
        rospy.spin()

        # Continue to run while rospy is not dead
        #while not rospy.is_shutdown():
            # if 
            #rospy.loginfo("Running")


if __name__ == '__main__':
    
    rospy.init_node('local_navigation')
    try:
        ln = LocalNav()
    except rospy.ROSInterruptException: pass
