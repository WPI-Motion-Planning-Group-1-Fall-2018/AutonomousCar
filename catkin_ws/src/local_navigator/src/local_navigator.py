#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry, OccupancyGrid, MapMetaData, Path
from std_msgs.msg import String
import math
from math import sin, cos, pi,tan, atan2
from geometry_msgs.msg import Twist, PoseStamped
import numpy as np


#define a global
pose = [0.0,0.0,0.0,0.0]

class LocalNav:
    def callbackPath(self,data):
        rospy.loginfo(rospy.get_caller_id() + ' !!!!!!!!!!!!!!!!I heard %s', data.poses[0].pose.position.x)
        self.gotPath_ = True
        # 


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
        self.gotCarPose_ = True


    def callbackOccGrid(self,msg):
        #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
        width=msg.info.width
        height=msg.info.width
        origin=msg.info.origin
        resolution = msg.info.resolution
        maptime = msg.info.map_load_time
        
        region1 = False
        region2 = False
        region3 = False
        region4 = False
        region5 = False
        region6 = False
        obstacle = False
        
        # create a zero array
        board = np.zeros([width,height])
        i = 0
        
        #Make a matrix 
        for x in range (0, width):
            for y in range (0, height):
                
                board[x][y] = msg.data[i]
                '''if i < 45300:
                    cellval = msg.data[i]
                    if (cellval == 100):
                        obstacle = True'''
                i = i + 1
                
        #Check for obstacles in the various regions
        
        for x in range (0, width):
            for y in range (100, height):
                
                '''if (board[x][y] == 100):
                        obstacle = True
                        foundx = x
                        foundy = y'''
                
                if ((y>=180) and (y<=239) and (x>=120) and (x<=179)):
                    if (board[x][y] == 100):
                        obstacle = True
                        region1 = True
                        foundx = x
                        foundy = y
                
                if ((y>=240) and (y<=299) and (x>=120) and (x<=179)):
                    if (board[x][y] == 100):
                        obstacle = True
                        region2 = True
                        foundx = x
                        foundy = y
                
                        
                if ((y>=180) and (y<=239) and (x>=180) and (x<=239)):
                    if (board[x][y] == 100):
                        obstacle = True
                        region3 = True
                        foundx = x
                        foundy = y
                
                if ((y>=240) and (y<=299) and (x>=180) and (x<=239)):
                    if (board[x][y] == 100):
                        obstacle = True
                        region4 = True
                        foundx = x
                        foundy = y
                
                if ((y>=180) and (y<=239) and (x>=60) and (x<=119)):
                    if (board[x][y] == 100):
                        obstacle = True
                        region5 = True
                        foundx = x
                        foundy = y
                
                if ((y>=240) and (y<=299) and (x>=60) and (x<=119)):
                    if (board[x][y] == 100):
                        obstacle = True
                        region6 = True
                        foundx = x
                        foundy = y
                
                                 
                
        if obstacle:
            rospy.loginfo("Obstacle found")
            rospy.loginfo("Lastfoundx: %s and LastFoundy: %s", foundx, foundy)
        else:
            rospy.loginfo("NO Obstacle found, keep trucking at speed")
            
        if region1:
            rospy.loginfo("Obstacle in Region 1 - Immediate Front")
               
        if region2:
            rospy.loginfo("Obstacle in Region 2 - Distant Front")
        
        if region3:
            rospy.loginfo("Obstacle in Region 3 - Immediate Front Left")
               
        if region4:
            rospy.loginfo("Obstacle in Region 4 - Distant Front Left")
        
        if region5:
            rospy.loginfo("Obstacle in Region 5 - Immediate Front Right")
               
        if region6:
            rospy.loginfo("Obstacle in Region 6 - Distant Front Right")
            
            
        #rospy.loginfo("Width: %s and Height: %s", width, height)
        #rospy.loginfo("Origin: %s", origin)
        
        self.gotCostMap_ = True



    def __init__(self):

        
        self.gotPath_ = False
        self.gotCarPose_ = False
        self.gotCostMap_ = False

        # subscribe for the car pose ground truth position
        rospy.Subscriber("base_pose_ground_truth", Odometry, self.callbackCarPos, queue_size =1)
        
        # Jean's subscription changed for object oriented
        rospy.Subscriber("/path", Path, self.callbackPath, queue_size=1)

        # subscribe for the local cost map that John is generating

        rospy.Subscriber("local_costmap", OccupancyGrid, self.callbackOccGrid, queue_size=1)
    
        #rospy.spin()

        while not rospy.is_shutdown():
            if self.gotPath_:
                #Do something with the path beyond the callback
                self.gotPath_ = False
            if self.gotCarPose_:
                #Do something knowing the carPose
                self.gotCarPose_ = False
            if self.gotCostMap_:
                #Do something knowing obstacles
                self.gotCostMap_ = False


if __name__ == '__main__':
    
    rospy.init_node('local_navigation')
    try:
        ln = LocalNav()
    except rospy.ROSInterruptException: pass
