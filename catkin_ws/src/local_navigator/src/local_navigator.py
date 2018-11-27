#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry, OccupancyGrid, MapMetaData, Path
from std_msgs.msg import String
import math
from math import sin, cos, pi,tan, atan2
from geometry_msgs.msg import Twist, PoseStamped
import numpy as np

class LocalNav:
    def callbackPath(self,data):
        
        # This callback will functionally only need to run once to pull the current
        # global path into a local list in order, other times it will just post the current Path
        
        if not self.gotNewPath:
            #rospy.loginfo(rospy.get_caller_id() + ' !!!!!!!!!!!!!!!! I heard %s', data.poses[0].pose.position.x)
            length = len(data.poses)
            #rospy.loginfo(rospy.get_caller_id() + ' !!!!!!!!!!!!!!!! Length of path %s', length)
            for i in range (0,length):
                x = data.poses[i].pose.position.x
                y = data.poses[i].pose.position.y
                self.currentPath.append([x,y])
            self.nextGlobalWaypoint = self.currentPath[0]
            self.nextnextGlobalWaypoint = self.currentPath[1]
            self.gotNewPath = True
        
        if self.gotNewPath:
            rospy.loginfo(rospy.get_caller_id() + ': Path: %s', self.currentPath)
            
        self.gotPath_ = True
        # 


    def callbackCarPos(self,msg):

        #global pose
        x=msg.pose.pose.position.x
        y=msg.pose.pose.position.y
        q0 = msg.pose.pose.orientation.w
        q1 = msg.pose.pose.orientation.x
        q2 = msg.pose.pose.orientation.y
        q3 = msg.pose.pose.orientation.z
        theta=atan2(2*(q0*q3+q1*q2),1-2*(q2*q2+q3*q3))
        self.currentCarPose=[x,y,0,theta]
        rospy.loginfo(self.currentCarPose)
        self.gotCarPose_ = True


    def callbackOccGrid(self,msg):
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

    def euclid(self, point1, point2): #DONE
        dist = math.sqrt((point1[0]-point2[0])**2+(point1[1]-point2[1])**2)
        return dist


    def __init__(self):
        
        self.distToNextWP = 10 # 10 meters is close enough to next waypoint

        self.currentCarPose = [0.0,0.0,0.0,0.0]
        self.currentPath = [] # Will contain the path
        self.gotNewPath = False  #Allows trigger once for getting a new path
        
        self.nextGlobalWaypoint = []
        self.nextnextGlobalWaypoint = []
        
        self.gotPath_ = False
        self.gotCarPose_ = False
        self.gotCostMap_ = False

        ## Subscribers
        # subscribe for the car pose ground truth position
        rospy.Subscriber("base_pose_ground_truth", Odometry, self.callbackCarPos, queue_size =1)
        
        # Jean's subscription changed for object oriented
        rospy.Subscriber("/path", Path, self.callbackPath, queue_size=1)

        # subscribe for the local cost map that John is generating

        rospy.Subscriber("local_costmap", OccupancyGrid, self.callbackOccGrid, queue_size=1)
    
        # Publishers
        
    
        #rospy.spin()
        
        rate = rospy.Rate(10) # 10 Hz

        while not rospy.is_shutdown():
            if self.gotPath_:
                #Do something with the path beyond the callback
                self.gotPath_ = False
            if self.gotCarPose_:
                # Check current car position against current waypoint distance
                currentx = self.currentCarPose[0]
                currenty = self.currentCarPose[1]
                if self.gotNewPath:
                    distance = self.euclid([currentx,currenty], self.nextGlobalWaypoint)
                    if (distance <= self.distToNextWP):
                        # make the nextGlobalWP equal to nextnextGlobalWP
                        # increment nextnextGlobalWP to the next WP in path
                        rospy.loginfo('Reached the next waypoint %s with distance from %s', self.nextGlobalWaypoint,distance)
                        #publish the nextWaypoint with a desired velocity
                    else:
                        rospy.loginfo('Still going to waypoint %s', self.nextGlobalWaypoint)
                        #publish the nextWaypoint (same as before) with a desired velocity
                self.gotCarPose_ = False
            if self.gotCostMap_:
                #Do adjustments to the next waypoint based on obstacles in gotCarPose if above
                #Use this 
                self.gotCostMap_ = False
            rate.sleep()
            


if __name__ == '__main__':
    
    rospy.init_node('local_navigation')
    try:
        ln = LocalNav()
    except rospy.ROSInterruptException: pass
