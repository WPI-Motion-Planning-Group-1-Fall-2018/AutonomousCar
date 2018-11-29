#!/usr/bin/env python
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

'''
def latLonToPoints(node_ref):
    #Pulls out the latitude and longitudes of the nodes in the
    #list of nodes and gets the points in the gazebo frame
    coords = np.array([])
    for node in node_ref:

        if self.checkCoordinateBoundaries(node):
            coords = np.append(coords,
                               self.node[node]
                               .get("lon"))
            coords = np.append(coords,
                               self.node[node]
                               .get("lat"))
            coords = np.reshape(coords,
                                (len(coords)/2,
                                 2))

    pointsXYZ = self.getPoints(coords)
    return pointsXYZ

def getPoints(coords):
    #Input : latitude and longitudnal coordinates
    #   Return the points in gazebo frame with respect
    #   to the starting coordinates
    if not coords.any():
        return []


    lon2 = np.radians(coords[:, 0])
    lat2 = np.radians(coords[:, 1])

    dLat = lat2-np.radians(self.getLat())
    dLon = lon2-np.radians(self.getLon())

    ## no idea what this letter a is
    a = (np.sin(dLat/2) * np.sin(dLat/2) +
         np.sin(dLon/2) * np.sin(dLon/2) *
         np.cos(np.radians(self.getLat())) *
         np.cos(lat2))

    ## no idea what this letter c is
    c = 2 * np.arctan2(np.sqrt(a), np.sqrt(1-a))
    
    distance = self.R * c

    angles = (np.arctan2(np.sin(dLon) * np.cos(lat2),
              np.cos(np.radians(self.getLat())) *
              np.sin(lat2) -
              np.sin(np.radians(self.getLat())) *
              np.cos(lat2) * np.cos(dLon)))
    point = np.array([distance*np.cos(angles) * 1000,
                      -distance*np.sin(angles) * 1000,
                      np.zeros(np.shape(distance))*1000])

    return point
'''

def Construct_Path_Msg(x, y, length):
    path = Path()
    #poses = [] 
    
    
    # Create path message
    path.header.frame_id="map"
    path.header.stamp=rospy.Time.now()
    
    #print("Make a new path")
           
    # make poses from data
    for i in range(0, length,1):
        pose = PoseStamped()
        #print ('testing',i) 
        pose.header.frame_id = "map"
        pose.header.seq = i
        pose.header.stamp = path.header.stamp 
        pose.pose.position.x = x[i]
        pose.pose.position.y = y[i]
        #print ('x is', pose.pose.position.x)
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 1.0
        #poses.append(pose)
        #print(pose)
        path.poses.append(pose)
        #print("PATH is ")
        #print(path)
        
             
    return path


def talker():
    x = [26.4527, 39.5884, 52.7228, 69.0941, 83.7454, 105.948, 123.622, 142.292, 162.782, 172.996, 171.868, 171.274 ]
    y = [13.998, 14.5571, 14.5571, 15.1161, 15.6748, 16.14200, 16.142, 16.0263, 16.2719, 48.2004, 82.6559, 122.569]
    index = len(x)
    
    pub = rospy.Publisher('path', Path, queue_size=1)  # define the publisher
    rospy.init_node('global_navigator', anonymous=True)  # start the node
    rate = rospy.Rate(1) # 1 hz
    while not rospy.is_shutdown():
        
        thePath = Construct_Path_Msg(x, y, index)
        pub.publish(thePath)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
