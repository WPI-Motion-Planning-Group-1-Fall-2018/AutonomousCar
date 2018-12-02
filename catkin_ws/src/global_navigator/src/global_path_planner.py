#!/usr/bin/env python
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import csv, math, operator
import numpy as np

###########################
##      Dictionaries     ##
###########################

def dictionaries(nodes_file, edges_file):
    
    # Nodes Dictionary

    # nodes_dict Format:
    #   {node_id1: [lon1, lat1],
    #    node_id2: [lon2, lat2], ...}

    nodes_dict = {}
    with open(nodes_file, mode='r') as infile:
        reader = csv.reader(infile)
        for rows in reader:
            if rows[0] != "id":
                id = int(rows[0])
                lon = float(rows[1])
                lat = float(rows[2])
                nodes_dict[id] = [lon, lat]

    #print("nodes_dict:", nodes_dict)

    # Edges Dictionary (Adjacency List, dictionary of dictionaries)

    # edges_dict:node connections and cost between nodes
    #   Format:
    #   {node1: {node2: cost12, node3: cost13},
    #    node2: {node1: cost12, node4: cost24}, ...}
    # 
    # edges_waypoints: actual piecewise lat/long path between nodes
    #   Format:
    #   {node1: {node2: [list of lon/lat coordinates between node1 and node2]},
    #    node2: {node3: [list of lon/lat coordinates between node2 and node3]}, ...}

    edges_dict = {}
    edges_waypoints = {}

    with open(edges_file, mode='r') as infile:
        reader = csv.reader(infile)
        for row in reader:
            if row[0] != "id": # ignore first line
                id = row[0]
                source = int(row[1])
                target = int(row[2])
                length = float(row[3])
                car_forward = int(row[5])
                car_backward = int(row[6])
                wkt = row[9]

                waypoints = []
                waypoints_start = wkt.find("(") + 1
                wkt = wkt[waypoints_start:-1]
                waypoints_string = wkt.split(", ")
                for point in waypoints_string:
                    waypoints.append([float(i) for i in point.split(" ")])

                if car_forward:
                    if source in edges_dict:
                        edges_dict[source].update({target: length})
                        edges_waypoints[source].update({target: waypoints})
                    else:
                        edges_dict[source] = {target: length}
                        edges_waypoints[source] = {target: waypoints}
                if car_backward:
                    if target in edges_dict:
                        edges_dict[target].update({source: length})
                        edges_waypoints[target].update({source: waypoints})
                    else:
                        edges_dict[target] = {source: length}
                        edges_waypoints[target] = {source: waypoints}

    #print("edges_dict:", edges_dict)
    #print("edges_waypoints:", edges_waypoints)
    #print(edges_dict[261180919])
    #print(edges_dict[369631024])
    #print(edges_waypoints[261180919])
    #print(edges_waypoints[369631024])

    return nodes_dict, edges_dict, edges_waypoints

#################
##     DFS     ##
#################

def dfs(edges_dict, start, goal):

    # DFS Search Algorithm

    # Initialize start and mark as visited
    stack = [start]
    visited = [start]

    while stack:
        # Examine last vertex from stack
        current_node = stack[-1]
        stack.pop()

        # Load neighbors from adjacency list
        try:
            neighbors = edges_dict[current_node].keys()
        except:
            neighbors = []

        # Mark non-obstacle and unvisited neighbors as visited, add to stack if haven't reached goal
        for neighbor in neighbors:
            if neighbor not in visited:
                visited.append(neighbor)
                if neighbor == goal: # end search if goal is reached
                    return visited
                else: # keep going
                    stack.append(neighbor)

######################
##     Dijkstra     ##
######################

def dijkstra(edges_dict, start, goal, nodes_dict):

    # Dijkstra Search Algorithm

    # Initialization
    open = {start: 0}
    closed = {}
    past_cost = {}
    parents = {}

    for node in nodes_dict.keys():
        past_cost[node] = float("inf")

    past_cost[start] = 0

    # Search while open not empty
    while open:
        # Find lowest cost open node, remove from open
        sorted_open = sorted(open.items(), key=operator.itemgetter(1))
        current = sorted_open[0][0]
        del open[current]

        # Break if end reached
        if current == goal:
            break
        
        # Load neighbors from adjacency list
        try:
            neighbors = edges_dict[current].keys()
        except:
            neighbors = []

        # Check neighbors of current node
        for nbr in neighbors:
            new_cost = past_cost[current] + cost(current, nbr, nodes_dict)
            f_n = new_cost # f(x) only dependent on g(x), not h(x) heuristic

            if nbr in open.keys():
                # If nbr already in open and lower cost than current, do nothing
                if past_cost[nbr] <= new_cost:
                    continue
            elif nbr in closed.keys():
                # If nbr already in closed and lower cost than current, do nothing
                if past_cost[nbr] <= new_cost:
                    continue
                # Move nbr from closed to open
                del closed[nbr]
                open.update({nbr: f_n})
            else:
                # Add nbr to open
                open.update({nbr: f_n})

            # Update nbr cost, make current the parent of nbr
            past_cost[nbr] = new_cost
            if nbr not in parents.keys():
                parents[nbr] = current

        # Add current node to closed nodes list
        closed.update({current: past_cost[current]})
                    
    # Reconstruct path from parents starting at end
    path = [goal]
    node = goal
    while node != start:
        node = parents[node]
        path.append(node)

    path.reverse()

    return path

####################
##     A star     ##
####################

def Astar(edges_dict, start, goal, nodes_dict):

    # A* Search Algorithm

    # Initialization
    open = {start: 0}
    closed = {}
    past_cost = {}
    parents = {}

    for node in nodes_dict.keys():
        past_cost[node] = float("inf")

    past_cost[start] = 0

    # Search while open not empty
    while open:
        # Find lowest cost open node, remove from open
        sorted_open = sorted(open.items(), key=operator.itemgetter(1))
        current = sorted_open[0][0]
        del open[current]

        # Break if end reached
        if current == goal:
            break
        
        # Load neighbors from adjacency list
        try:
            neighbors = edges_dict[current].keys()
        except:
            neighbors = []

        # Check neighbors of current node
        for nbr in neighbors:
            new_cost = past_cost[current] + cost(current, nbr, nodes_dict)
            f_n = new_cost + cost(nbr, goal, nodes_dict)

            if nbr in open.keys():
                # If nbr already in open and lower cost than current, do nothing
                if past_cost[nbr] <= new_cost:
                    continue
            elif nbr in closed.keys():
                # If nbr already in closed and lower cost than current, do nothing
                if past_cost[nbr] <= new_cost:
                    continue
                # Move nbr from closed to open
                del closed[nbr]
                open.update({nbr: f_n})
            else:
                # Add nbr to open
                open.update({nbr: f_n})

            # Update nbr cost, make current the parent of nbr
            past_cost[nbr] = new_cost
            if nbr not in parents.keys():
                parents[nbr] = current

        # Add current node to closed nodes list
        closed.update({current: past_cost[current]})
                    
    # Reconstruct path from parents starting at end
    path = [goal]
    node = goal
    while node != start:
        node = parents[node]
        path.append(node)

    path.reverse()

    return path

###############################
##     Support Functions     ##
###############################

def cost(x1, x2, nodes_dict):
    x1coord = nodes_dict[x1]
    x2coord = nodes_dict[x2]

    cost = math.sqrt((x2coord[0] - x1coord[0])**2 + (x2coord[1] - x1coord[1])**2)
    # in units of degrees, used as distance here because small enough area that it is essentially a linear conversion

    return cost

def getPoints(coords, nodes_dict):
        '''Input : latitude and longitudnal coordinates
           Return the points in gazebo frame with respect
           to the starting coordinates'''
        if not any(coords):
            return []

        # Hard code center of OSM rectangle and radius of earth
        lon1 = -87.735200
        lat1 = 41.950200
        R = 6371 # radius of the earth

        lon2 = np.radians(coords[0])
        lat2 = np.radians(coords[1])

        dLon = lon2 - np.radians(lon1)
        dLat = lat2 - np.radians(lat1)

        ## no idea what this letter a is
        a = (np.sin(dLat/2) * np.sin(dLat/2) +
             np.sin(dLon/2) * np.sin(dLon/2) *
             np.cos(np.radians(lat1)) *
             np.cos(lat2))

        ## no idea what this letter c is
        c = 2 * np.arctan2(np.sqrt(a), np.sqrt(1-a))

        distance = R * c

        angles = (np.arctan2(np.sin(dLon) * np.cos(lat2),
                  np.cos(np.radians(lat1)) *
                  np.sin(lat2) -
                  np.sin(np.radians(lat1)) *
                  np.cos(lat2) * np.cos(dLon)))
        point = [distance*np.cos(angles) * 1000,
                 -distance*np.sin(angles) * 1000]

        return point

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

def main(algorithm, start, goal):
    
    # Pre-processing

    nodes_dict, edges_dict, edges_waypoints = dictionaries('nodes.csv', 'edges.csv')

    # Run algorithm of choice

    if algorithm == 1:
        path_ids = dfs(edges_dict, start, goal)
    elif algorithm == 2:
        path_ids = dijkstra(edges_dict, start, goal, nodes_dict)
    elif algorithm == 3:
        path_ids = Astar(edges_dict, start, goal, nodes_dict)

    # Convert path IDs to path lat-longs using waypoints

    path_latlong = []
    for i in range(len(path_ids)):
        if i != len(path_ids) - 1:
            if i == 0:
                node = edges_waypoints[path_ids[i]][path_ids[i + 1]]
                path_latlong.extend(node)
            else:
                node = edges_waypoints[path_ids[i]][path_ids[i + 1]][1::]
                path_latlong.extend(node)

    path_gazebo = []
    for node in path_latlong:
        path_gazebo.append(getPoints(node, nodes_dict))

    print("Number of nodes:", len(path_ids))
    print("Number of waypoints:", len(path_latlong))

    return path_gazebo

################################################
##     Run and Publish Planning Algorithm     ##
################################################

def talker():
    
    # Hard-coded Inputs
    
    algorithm = 3
    start = 261123517
    #goal = 3235485081 # 2-node path
    goal = 369630931 # many-node path, could be accomplished in 2 steps
    
    path = main(algorithm, start, goal)
    
    x = []
    y = []
    for node in path:
        x.append(node[0])
        y.append(node[1])
    
    #x = [26.4527, 39.5884, 52.7228, 69.0941, 83.7454, 105.948, 123.622, 142.292, 162.782, 172.996, 171.868, 171.274 ]
    #y = [13.998, 14.5571, 14.5571, 15.1161, 15.6748, 16.14200, 16.142, 16.0263, 16.2719, 48.2004, 82.6559, 122.569]
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
