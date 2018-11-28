#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry, OccupancyGrid, MapMetaData, Path
from std_msgs.msg import String
import math
from math import sin, cos, pi,tan, atan2
from geometry_msgs.msg import Twist, PoseStamped
import numpy as np
from prius_msgs.msg import LocalNav

class LocalNavigator:
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
            self.currentGlobalWaypoint = self.currentPath[0]
            self.nextGlobalWaypoint = self.currentPath[1]
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
        
        
        ## MIGHT NEED TO DO THIS TO CLEAR OBSTACLE FLAGS
        '''self.regionF1 = False
        self.regionF2 = False
        self.regionF3 = False
        self.regionF4 = False
        
        self.regionL1 = False
        self.regionL2 = False
        self.regionL3 = False
        self.regionL4 = False
        
        self.regionR1 = False
        self.regionR2 = False
        self.regionR3 = False
        self.regionR4 = False
        
        self.obstacle = False'''

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
                
                #F1
                if ((y>=180) and (y<=209) and (x>=140) and (x<=159)):
                    if (board[x][y] == 100):
                        self.obstacle = True
                        self.regionF1 = True
                        foundx = x
                        foundy = y
                        
                #F2
                if ((y>=210) and (y<=239) and (x>=140) and (x<=159)):
                    if (board[x][y] == 100):
                        self.obstacle = True
                        self.regionF2 = True
                        foundx = x
                        foundy = y
                        
                #F3
                if ((y>=240) and (y<=269) and (x>=140) and (x<=159)):
                    if (board[x][y] == 100):
                        self.obstacle = True
                        self.regionF3 = True
                        foundx = x
                        foundy = y
                        
                #F4
                if ((y>=270) and (y<=299) and (x>=140) and (x<=159)):
                    if (board[x][y] == 100):
                        self.obstacle = True
                        self.regionF4 = True
                        foundx = x
                        foundy = y
               
                #######     
                #R1
                if ((y>=180) and (y<=209) and (x>=120) and (x<=139)):
                    if (board[x][y] == 100):
                        self.obstacle = True
                        self.regionR1 = True
                        foundx = x
                        foundy = y
                        
                #R2
                if ((y>=210) and (y<=239) and (x>=120) and (x<=139)):
                    if (board[x][y] == 100):
                        self.obstacle = True
                        self.regionR2 = True
                        foundx = x
                        foundy = y
                        
                #R3
                if ((y>=240) and (y<=269) and (x>=120) and (x<=139)):
                    if (board[x][y] == 100):
                        self.obstacle = True
                        self.regionR3 = True
                        foundx = x
                        foundy = y
                        
                #R4
                if ((y>=270) and (y<=299) and (x>=120) and (x<=139)):
                    if (board[x][y] == 100):
                        self.obstacle = False
                        self.regionR4 = True
                        foundx = x
                        foundy = y
                
                #######     
                #L1
                if ((y>=180) and (y<=209) and (x>=160) and (x<=179)):
                    if (board[x][y] == 100):
                        self.obstacle = True
                        self.regionL1 = True
                        foundx = x
                        foundy = y
                        
                #L2
                if ((y>=210) and (y<=239) and (x>=160) and (x<=179)):
                    if (board[x][y] == 100):
                        self.obstacle = True
                        self.regionL2 = True
                        foundx = x
                        foundy = y
                        
                #L3
                if ((y>=240) and (y<=269) and (x>=160) and (x<=179)):
                    if (board[x][y] == 100):
                        self.obstacle = True
                        self.regionL3 = True
                        foundx = x
                        foundy = y
                        
                #L4
                if ((y>=270) and (y<=299) and (x>=160) and (x<=179)):
                    if (board[x][y] == 100):
                        self.obstacle = True
                        self.regionL4 = True
                        foundx = x
                        foundy = y
                                  
                
        if self.obstacle:
            rospy.loginfo("Obstacles found")
            rospy.loginfo("Lastfoundx: %s and LastFoundy: %s", foundx, foundy)
        else:
            rospy.loginfo("NO Obstacle found, keep trucking at speed")
        
            '''    
        if self.regionF1:
            rospy.loginfo("Obstacle in Region F1")
               
        if self.regionF2:
            rospy.loginfo("Obstacle in Region F2")
        
        if self.regionF3:
            rospy.loginfo("Obstacle in Region F3")
               
        if self.regionF4:
            rospy.loginfo("Obstacle in Region F4")
        
        if self.regionL1:
            rospy.loginfo("Obstacle in Region L1")
               
        if self.regionL2:
            rospy.loginfo("Obstacle in Region L2")
        
        if self.regionL3:
            rospy.loginfo("Obstacle in Region L3")
               
        if self.regionL4:
            rospy.loginfo("Obstacle in Region L4")
        
        if self.regionR1:
            rospy.loginfo("Obstacle in Region R1")
               
        if self.regionR2:
            rospy.loginfo("Obstacle in Region R2")
        
        if self.regionR3:
            rospy.loginfo("Obstacle in Region R3")
               
        if self.regionR4:
            rospy.loginfo("Obstacle in Region R4")'''
            
            
        #rospy.loginfo("Width: %s and Height: %s", width, height)
        #rospy.loginfo("Origin: %s", origin)
        
        self.gotCostMap_ = True

    def euclid(self, point1, point2): #DONE
        dist = math.sqrt((point1[0]-point2[0])**2+(point1[1]-point2[1])**2)
        return dist
        
    def checkobstacles(self):
        if self.regionF1:
            rospy.loginfo("Obstacle in Region F1")
            stopdist = 5 # 5 m from the location of the center of the car last checked
            
            #Immediately in front of car, need to stop
            #self.currentCarPose
            heading = self.currentCarPose[3]
            
            self.obstacle_avoid_wpx = self.currentCarPose[0] + cos(heading)*stopdist
            self.obstacle_avoid_wpy = self.currentCarPose[1] + sin(heading)*stopdist
            
            self.obstacle_avoid_spd = 0.0 # STOP
            
            rospy.loginfo("STOP at x: %s and y: %s at speed: %s",self.obstacle_avoid_wpx, self.obstacle_avoid_wpy, self.obstacle_avoid_spd)
            
        if self.regionF2:
            #obstacle is near, go to the left of it
            rospy.loginfo("Obstacle in Region F2")
            
            neardist = 7.5 # one obstacle regions worth
            lateraldist = 7
        
            heading = self.currentCarPose[3]
            
            self.obstacle_avoid_wpx = self.currentCarPose[0] + cos(heading)*neardist + cos(heading+1.5708)*lateraldist
            self.obstacle_avoid_wpy = self.currentCarPose[1] + sin(heading)*neardist + sin(heading+1.5708)*lateraldist
            
            self.obstacle_avoid_spd = 1.0 # slow down significantly
            
            rospy.loginfo("Waypoint at x: %s and y: %s at speed: %s",self.obstacle_avoid_wpx, self.obstacle_avoid_wpy, self.obstacle_avoid_spd)

        
        if self.regionF3:
            #Vehicle is closer, slow the further, go to the left of the obstacle
            rospy.loginfo("Obstacle in Region F3")
            
            lateraldist = 6
            neardist = 15 # 1.5 obstacle regions worth
        
            heading = self.currentCarPose[3]
            
            self.obstacle_avoid_wpx = self.currentCarPose[0] + cos(heading)*neardist + cos(heading+1.5708)*lateraldist
            self.obstacle_avoid_wpy = self.currentCarPose[1] + sin(heading)*neardist + sin(heading+1.5708)*lateraldist
            
            self.obstacle_avoid_spd = 1.5 # slow down significantly
            
            rospy.loginfo("Waypoint at x: %s and y: %s at speed: %s",self.obstacle_avoid_wpx, self.obstacle_avoid_wpy, self.obstacle_avoid_spd)

               
        if self.regionF4:
            #Vehicle in our lane in the distance, slow the car down but keep going straight
            rospy.loginfo("Obstacle in Region F4")
            
            neardist = 22.5 # 2.5 obstacle regions worth
        
            heading = self.currentCarPose[3]
            
            self.obstacle_avoid_wpx = self.currentCarPose[0] + cos(heading)*neardist
            self.obstacle_avoid_wpy = self.currentCarPose[1] + sin(heading)*neardist
            
            self.obstacle_avoid_spd = 2.0 # slow down significantly
            
            rospy.loginfo("Waypoint at x: %s and y: %s at speed: %s",self.obstacle_avoid_wpx, self.obstacle_avoid_wpy, self.obstacle_avoid_spd)

        
        if self.regionL1:
            rospy.loginfo("Something in Region L1")
            # Do nothing
               
        if self.regionL2:
            rospy.loginfo("Somethingin Region L2")
            # Do nothing
        
        if self.regionL3:
            rospy.loginfo("Something in Region L3")
            # Do nothing
               
        if self.regionL4:
            rospy.loginfo("Something in Region L4")
            # Do nothing
        
        if self.regionR1:
            rospy.loginfo("Something in Region R1")
            # Do nothing
               
        if self.regionR2:
            rospy.loginfo("Something in Region R2")
            # Do nothing
        
        if self.regionR3:
            rospy.loginfo("Something in Region R3")
            # Do nothing
               
        if self.regionR4:
            rospy.loginfo("Something in Region R4")
            # Do nothing


    def __init__(self):
        
        self.regionF1 = False
        self.regionF2 = False
        self.regionF3 = False
        self.regionF4 = False
        
        self.regionL1 = False
        self.regionL2 = False
        self.regionL3 = False
        self.regionL4 = False
        
        self.regionR1 = False
        self.regionR2 = False
        self.regionR3 = False
        self.regionR4 = False
        
        self.obstacle = False
        
        self.obstacle_avoid_wpx = 0.0
        self.obstacle_avoid_wpy = 0.0
        self.obstacle_avoid_spd = 0.0
        
        self.distToWP = 10 # 10 meters is close enough to next waypoint

        self.currentCarPose = [0.0,0.0,0.0,0.0]
        self.currentPath = [] # Will contain the path
        self.gotNewPath = False  #Allows trigger once for getting a new path
        
        self.lastGlobalWaypoint = []
        self.currentGlobalWaypoint = []
        self.nextGlobalWaypoint = []
        self.pathIndex = 0
        
        self.gotPath_ = False
        self.gotCarPose_ = False
        self.gotCostMap_ = False
        
        self.currentVelocity = 0.0
        self.currentheading = 0.0

        ## Subscribers
        # subscribe for the car pose ground truth position
        rospy.Subscriber("base_pose_ground_truth", Odometry, self.callbackCarPos, queue_size =1)
        
        # Jean's subscription changed for object oriented
        rospy.Subscriber("/path", Path, self.callbackPath, queue_size=1)

        # subscribe for the local cost map that John is generating

        rospy.Subscriber("local_costmap", OccupancyGrid, self.callbackOccGrid, queue_size=1)
    
        # Publishers
        pub = rospy.Publisher('local_nav_waypoints', LocalNav,queue_size=1)
        
        msg = LocalNav()
        #fake test values
        '''
        msg.x = 0.00
        msg.y = 10.00
        msg.speed = 50.0
        msg.max_speed = 100
        msg.max_accel = 80'''
    
        #rospy.spin()
        
        rate = rospy.Rate(1) # 10 Hz

        while not rospy.is_shutdown():
            if self.gotPath_:
                #Do something with the path beyond the callback
                self.gotPath_ = False
            if self.gotCarPose_:
                # Check current car position against current waypoint distance
                currentx = self.currentCarPose[0]
                currenty = self.currentCarPose[1]
                if self.gotNewPath:  #I have a path
                    self.checkobstacles()
                    
                    ## NEED TO CHECK FOR OBSTACLES AND SPECIFY INTERMEDIATE WP IF NECESSARY TO PUBLISH
                    '''if self.obstacle:  # Check if there are any obstacles currently
                        self.checkobstacles()
                        self.lastGlobalWaypoint = self.currentGlobalWaypoint
                        self.currentGlobalWaypoint = [self.obstacle_avoid_wpx, self.obstacle_avoid_wpy]
                        msg.x = self.obstacle_avoid_wpx
                        msg.y = self.obstacle_avoid_wpy
                        msg.speed = self.obstacle_avoid_spd
                        pub.publish(msg)  # publish an obstacle avoidance waypoint
                    
                    else: # no obstacles, check current location'''
                        
                        # If the last Globalwaypoint was in Path, then make the next WP in Path the currentWP 
                        # and make the nextGlobalwaypoint the next one over
                        
                    distance = self.euclid([currentx,currenty], self.currentGlobalWaypoint)
                    if (distance <= self.distToWP):  # Am i near the current WP?  Yes ...
                        # make the nextGlobalWP equal to nextnextGlobalWP
                        msg.x = 0.00
                        msg.y = 10.00
                        msg.speed = 50.0
                        msg.max_speed = 100
                        msg.max_accel = 80
     
                        # increment nextnextGlobalWP to the next WP in path
                        rospy.loginfo('Reached the waypoint %s with distance from %s', self.currentGlobalWaypoint,distance)
                        #publish the nextWaypoint with a desired velocity
                        pub.publish(msg)
                    else: # ... No
                        msg.x = 20.00
                        msg.y = 70.00
                        msg.speed = 80.0
                        msg.max_speed = 500
                        msg.max_accel = 100
     
                            
                        #CHECK FOR OBSTACLES
                        #publish the nextWaypoint (same as before) with a desired velocity
                        rospy.loginfo('Still going to waypoint %s', self.currentGlobalWaypoint)
                        pub.publish(msg)
                        
                self.gotCarPose_ = False
            if self.gotCostMap_:
                #Do adjustments to the next waypoint based on obstacles in gotCarPose if above
                #Use this 
                self.gotCostMap_ = False
            rate.sleep()
            


if __name__ == '__main__':
    
    rospy.init_node('local_navigation')
    try:
        ln = LocalNavigator()
    except rospy.ROSInterruptException: pass
