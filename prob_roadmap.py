# coding: utf-8
# Author: YWiyogo
# Description: Probabilistic Roadmap implementation to support the graph-mode motion planning in 3D space
# by utilizing the 2.5 D map representation and random sampling to create the nodes.
# The starter code of this code is based on the exercise of lesson 8 about probabilistic roadmap
# 1. Load the obstacle map data
# 2. Sample nodes (use KDTrees here)
# 3. Connect nodes (use KDTrees here)
# 4. Visualize graph
# 5. Define heuristic
# 6. Define search method
# 7. Execute and visualize
#

import sys
import networkx as nx
nx.__version__  # should be 2.1
import time
import numpy.linalg as LA  # for heuristic calculation
import numpy as np
import matplotlib.pyplot as plt
from shapely.geometry import Polygon, Point, LineString
from queue import PriorityQueue

# YW: sample points randomly
# then use KDTree to find nearest neighbor polygon
# and test for collision
from sklearn.neighbors import KDTree


class Area:
    '''
    A class for defining an area
    '''

    def __init__(self, *args, **kwargs):
        self.north_min = args[0]
        self.north_max = args[1]
        self.east_min = args[2]
        self.east_max = args[3]


def gen_circular_random(center, r, num_samples, area):
    '''
    Generate uniform random 2D samples in a circular area
    center: center of the circle (list)
    r: radius (int)
    num_samples: number of samples (int)
    area: [minx, miny, maxx, maxy]
    The concept idea is based on https://stackoverflow.com/a/30564256
    '''
    i = 0
    x_samples = []
    y_samples = []
    while i < num_samples:
        r_sample, theta = np.sqrt(np.random.uniform(0, r)) * np.sqrt(r), 2 * np.pi * np.random.uniform(0, 1)
        x = int(center[0] + r_sample * np.cos(theta))
        y = int(center[1] + r_sample * np.sin(theta))
        if x > area.east_min or x < area.east_max:
            if y > area.north_min or y < area.north_max:
                x_samples.append(x)
                y_samples.append(y)
                i = i + 1

    if False:  #for debugging the function
        plt.figure()
        plt.title("Uniform Random Sampling in Circle")
        plt.scatter(x_samples, y_samples)
        plt.show()

    return (x_samples, y_samples)


class Obstacle(Area):

    def __init__(self, *args, **kwargs):
        Area.__init__(self, args[0], args[1], args[2], args[3])


class KdSampler:
    '''
    Represent the KD-Tree Sampling 
    '''

    def __init__(self, data, num_samples, start=(), goal=(), safety_dist=1):
        if type(start) is not tuple or type(goal) is not tuple:
            raise TypeError("Start and goal coordinate has to be a tuple")
        self._num_samples = num_samples
        self._min_z = 5
        self._max_z = 15
        self._start_node = start
        self._goal_node = goal
        self._area = Area(
            np.min(data[:, 0] - data[:, 3]), np.max(data[:, 0] - data[:, 3]), np.min(data[:, 1] - data[:, 4]),
            np.max(data[:, 1] - data[:, 4]))
        self._kept_samples = []
        self._removed_samples = []
        # Take the center of the obstacle to the KDTree
        # KDtree input and the query have to be the same dimensions
        self._obstKD_Tree = KDTree(data[:, 0:3])
        dist = np.linalg.norm(np.array(goal[:2]) - np.array(start[:2]))
        center = (np.array(goal[:2]) + np.array(start[:2])) / 2
        rad_deviation = 5   # to deal with a dead end or possible redirections
        radius = (dist / 2) + rad_deviation
        print("Air distance: ", dist)
        # Generate samples in the circle with diameter from start and goal points
        xvals, yvals = gen_circular_random(center, radius , num_samples, self._area)
        zvals = np.random.uniform(self._min_z, self._max_z, num_samples).astype(int)

        rand_3dsamples = list(zip(xvals, yvals, zvals))
        rand_3dsamples.append(tuple(start))
        rand_3dsamples.append(tuple(goal))

        # check the nearest obstacle centers

        for point3d in rand_3dsamples:
            # get the nearest 3 obstacle centers
            data_indices = self._obstKD_Tree.query([point3d], k=3, return_distance=False)[0]
            # check for the collision using polygon
            collision = False
            for i in data_indices:
                north, east, alt, d_north, d_east, d_alt = data[i, :]

                # YW NOTE: incorporate the safety distance in the obstacle object
                obstacle = Obstacle(north - d_north - safety_dist, north + d_north + safety_dist,
                                    east - d_east + safety_dist, east + d_east + safety_dist)
                corners = [(obstacle.north_min, obstacle.east_min), (obstacle.north_min, obstacle.east_max),
                           (obstacle.north_max, obstacle.east_max), (obstacle.north_max, obstacle.east_min)]
                height = alt + d_alt
                p = Polygon(corners)
                if p.contains(Point(point3d)) and (height >= point3d[2]):
                    #print("Colission => obstacle height: %d, sample height: %d" %(height, point3d[2]))
                    self._removed_samples.append(point3d)

                    point3d_list = list(point3d)
                    # list comparison only, avoid numpy array!
                    if point3d_list == start or point3d_list == goal:
                        print("WARNING: the start or goal node in {0}is removed!!".format(point3d))
                    collision = True
                    break
            if collision == False:
                self._kept_samples.append(point3d)

        # calculate the polygons of the obstacles
        self._polygons = []
        for i in range(data.shape[0]):
            north, east, alt, d_north, d_east, d_alt = data[i, :]

            #obstacle[north min, north max, east min, east max]
            obstacle = Obstacle(north - d_north - safety_dist, 
                                north + d_north + safety_dist, 
                                east - d_east - safety_dist, 
                                east + d_east+ safety_dist)
            corners = [
                (obstacle.north_min, obstacle.east_min),
                (obstacle.north_min, obstacle.east_max),
                (obstacle.north_max, obstacle.east_max),
                (obstacle.north_max, obstacle.east_min),
            ]
            height = alt + d_alt
            p = Polygon(corners)
            self._polygons.append((p, height))


def can_connect(n1, n2, polygons):
    '''
    Check if two nodes can be connected by checking the polygons obstacles and its height
    '''
    l = LineString([n1, n2])
    for p, height in polygons:
        if p.crosses(l) and height >= min(n1[2], n2[2]):
            return False
    return True


def create_graph(kdtree_sampler, k):
    '''
    Create a KD-Tree graph with
    '''
    g = nx.Graph()
    nodes = kdtree_sampler._kept_samples
    tree = KDTree(nodes)
    for n1 in nodes:
        # for each node connect try to connect to k nearest nodes
        idxs = tree.query([n1], k, return_distance=False)[0]
        for idx in idxs:
            n2 = nodes[idx]
            if n2 == n1:
                continue

            if can_connect(n1, n2, kdtree_sampler._polygons):
                g.add_edge(n1, n2, weight=1)
    return g


def heuristic(n1, n2):
    return LA.norm(np.array(n2) - np.array(n1))


def a_star_graph(graph, heuristic, start, goal):
    """
    Modified A* to work with NetworkX graphs.
    start: 3D tuple
    goal: 3D tuple
    """
    if type(start) is not tuple or type(goal) is not tuple:
        raise TypeError("Start and goal coordinate has to be a tuple")
    path = []
    queue = PriorityQueue()
    queue.put((0, start))
    visited = set(start)
    #create a branch dict
    branch = {}
    found = False

    while not queue.empty():
        item = queue.get()
        current_cost = item[0]
        current_node = item[1]

        if current_node == goal:
            print('Found a path.')
            found = True
            break
        else:
            for next_node in graph[current_node]:
                cost = graph.edges[current_node, next_node]['weight']
                new_cost = current_cost + cost + heuristic(next_node, goal)

                if next_node not in visited:
                    visited.add(next_node)
                    queue.put((new_cost, next_node))
                    branch[next_node] = (new_cost, current_node)

    #print("Branch: ", branch)
    path = []
    path_cost = 0
    if found:
        # retrace steps from the goal to the start point
        path.append(goal)
        n = goal
        path_cost = branch[n][0]
        while branch[n][1] != start:
            path.append(branch[n][1])
            n = branch[n][1]
        path.append(branch[n][1])
    else:
        print('**********************')
        print('Failed to find a path!')
        print('**********************')

    return path[::-1], path_cost


def visualize_graph(grid, data, graph, kdtree_sampler, path=[]):
    '''
    Visualize the 2D obstacle grid, the samples, connected nodes and the result path
    '''
    fig = plt.figure()

    # ----------------------------------------------
    # Subfigure 1 shows the samples, removed samples and the connections
    # ----------------------------------------------
    ax1 = fig.add_subplot(121)
    ax1.imshow(grid, cmap='Greys', origin='lower')
    nmin = np.min(data[:, 0])
    emin = np.min(data[:, 1])
    
    # draw points
    if len(kdtree_sampler._kept_samples):
        all_pts = np.array(kdtree_sampler._kept_samples)
        north_keepvals = all_pts[:, 0]
        east_keepvals = all_pts[:, 1]
        ax1.scatter(east_keepvals - emin, north_keepvals - nmin, c='green', label="kept")

    else:
        print("No kept samples!")

    if len(kdtree_sampler._removed_samples):
        removed_pts = np.array(kdtree_sampler._removed_samples)
        north_vals = removed_pts[:, 0]
        east_vals = removed_pts[:, 1]
        ax1.scatter(east_vals - emin, north_vals - nmin, c='orange', marker='x', label="removed")
    else:
        print("No removed samples!")

    for (n1, n2) in graph.edges:
        ax1.plot([n1[1] - emin, n2[1] - emin], [n1[0] - nmin, n2[0] - nmin], 'black', alpha=0.5)

    # Draw all nodes connected
    # for n1 in kdtree_sampler._kept_samples:
    #     ax1.scatter(n1[1] - emin, n1[0] - nmin, c='green',)

    # Draw connected nodes in darkgreen
    for n1 in graph.nodes:
        ax1.scatter(n1[1] - emin, n1[0] - nmin, c='blue', label="connected")

    ax1.scatter(kdtree_sampler._start_node[1] - emin, kdtree_sampler._start_node[0] - nmin, c='red', label="Start")
    ax1.scatter(kdtree_sampler._goal_node[1] - emin, kdtree_sampler._goal_node[0] - nmin, c='magenta', label = "Goal")
    ax1.set_xlabel('EAST')
    ax1.set_ylabel('NORTH')
    ax1.set_title("Connected Graph")
    #ax1.legend()

    # ----------------------------------------------
    # Subfigure 2 shows the path
    # ----------------------------------------------
    ax2 = fig.add_subplot(122)
    ax2.imshow(grid, cmap='Greys', origin='lower')

    # Add code to visualize path here
    nmin = np.min(data[:, 0])
    emin = np.min(data[:, 1])

    # draw nodes
    for n1 in graph.nodes:
        ax2.scatter(n1[1] - emin, n1[0] - nmin, c='blue')

    # draw edges
    for (n1, n2) in graph.edges:
        ax2.plot([n1[1] - emin, n2[1] - emin], [n1[0] - nmin, n2[0] - nmin], 'grey')
    if len(path) > 0:
        path_pairs = zip(path[:-1], path[1:])
        for (n1, n2) in path_pairs:
            ax2.plot([n1[1] - emin, n2[1] - emin], [n1[0] - nmin, n2[0] - nmin], 'red')

    ax2.scatter(kdtree_sampler._start_node[1] - emin, kdtree_sampler._start_node[0] - nmin, c='red')
    ax2.scatter(kdtree_sampler._goal_node[1] - emin, kdtree_sampler._goal_node[0] - nmin, c='magenta')
    ax2.set_ylabel('NORTH')
    ax2.set_xlabel('EAST')
    ax2.set_title("Result path")
    #ax2.legend()
    # set full screen
    figManager = plt.get_current_fig_manager()
    figManager.window.showMaximized()
    plt.show()


# --------------------------------------------------------------
# Test case
# --------------------------------------------------------------
if False:
    filename = 'colliders.csv'
    data = np.loadtxt(filename, delimiter=',', dtype='Float64', skiprows=2)
    map_data_range = [-316, -445, 605, 476]  #[north min, max, east min ,max]
    # Needs 3D points
    start = (0, 0, 5)
    goal = (400, 74, 5)
    print("Start: {0}, goal: {1}".format(start, goal))

    SAVETY_DISTANCE = 3
    kdtree_sampler = KdSampler(data, 150, start, goal, SAVETY_DISTANCE)

    print("Number of samples: ", len(kdtree_sampler._kept_samples))
    print("Number of rejected samples: ", len(kdtree_sampler._removed_samples))

    t0 = time.time()

    # YW NOTE: given a large amount of k allows the algorithm to connect
    # the nodes that has a long distance. k represents the maximum edge in a node.
    g = create_graph(kdtree_sampler, 8)
    print('graph took {0} seconds to build'.format(time.time() - t0))

    # Create a grid map of the world
    from planning_utils import create_grid
    # This will create a grid map at 1 m above ground level
    
    grid, north_min, east_min = create_grid(data, 5, SAVETY_DISTANCE)
    path, cost = a_star_graph(g, heuristic, start, goal)

    # Beware that a zip object is temporary
    path_pairs = zip(path[:-1], path[1:])
    visualize_graph(grid, data, g, kdtree_sampler, path)
    print("Path pairs: ")
    for (n1, n2) in path_pairs:
        print(n1, n2)
