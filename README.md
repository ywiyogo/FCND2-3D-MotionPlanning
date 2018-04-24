# Project: 3D Motion Planning

---

## Introduction
This 3D motion planning project simulates the planning algorithm of a drone in a 3D space of San Francisco (US). The goal of this project is to implement the basic planning algorithm using A*, collinearity check, Bresenham. Understanding the geodetic, ECEF (earth-centered, earth-fixed), NED (north, east, down) coordinates are the basic requirements to deal with the simulator.


## Writeup

### Understanding the starter code in `motion_planning.py` and `planning_utils.py`
These scripts contain a basic planning implementation. The main difference between `motion_planning.py` and `backyard_flyer_solution.py` is the waypoint generation. Instead of implementing the waypoint in a particular function (`calculate_box()`), `motion_planning.py` implements the waypoints by utilizing the A* motion planning in the function `plan_path()`.


### Implementing the Path Planning Algorithm

#### 1. Set the global home position
Read the first line of the csv file, extract lat0 and lon0 as floating point values and use the `self.set_home_position()` method to set global home. Notice that the extracted value from the csv file is in [latitute, longitute] format where the  `self.set_home_position()` input parameters are [longitute, latitude, altitude].


#### 2. Set the current local position
In the Drone class property, we can retrieve the global position of the drone. The class provide the property `self.global_position` which returns`np.array([self._longitude, self._latitude, self._altitude])`. Using the function `global_to_local()` from the `udacidrone.frame_utils` we can calculate the local position.

#### 3. Set the grid start position from the local position

By passing the csv data to the `create_grid()` function we can get the grid map and the minimum values of the north and east coordinates. These both minimum values can be used as the offset of the map where we define that the extracted minimum values are our (0,0) coordinate. Note, that we have to convert the float values to the integer values since the grid does not accept the float values.
```
grid_start = (int_cur_pose_local[0] - north_min, int_cur_pose_local[1] - east_min)
```

#### 4. Set the grid goal position from geodetic coordinates
In order to set the correct goal position from the geodetic coordinates, I use the manual mode to navigate the drone in the desired goal position. The function `global_to_local()` convert the global goal position the local position. Then, I subtract the values from the north and east offsets.

```
global_goal = [ -122.397712, 37.793741, self._altitude] 
print("global_goal (long-lat-alt): ", global_goal)
local_goal = global_to_local(global_goal, self.global_home) 
int_local_goal = local_goal.astype(int)
# convert the local NORTH EAST direction to the grid map
grid_goal = (int_local_goal[0] - north_min, 
                int_local_goal[1] - east_min)
```

#### 5. Modify A* to include diagonal motion (or replace A* altogether)
For optimization, we add the diagonal motions in the Action class, defined in the `planning_utils.py` file. The cost of the diagonal action is defined as a square root of 2.

    NE = (-1, 1, np.sqrt(2))
    SE = (1, 1, np.sqrt(2))
    SW = (1, -1, np.sqrt(2))
    NW = (-1, -1, np.sqrt(2))

The `valid_actions(grid, current_node)` function has to be extended for the diagonal motions:

    if (x - 1 < 0 and y + 1 > m) or grid[x - 1, y + 1] == 1:
        valid_actions.remove(Action.NE)
    if (x + 1 > n and y + 1 > m) or grid[x + 1, y + 1] == 1:
        valid_actions.remove(Action.SE)
    if (x + 1 > n and y - 1 < 0 )or grid[x + 1, y - 1] == 1:
        valid_actions.remove(Action.SW)
    if (x - 1 < 0 and y + 1 > m) or grid[x -1, y + 1] == 1:
        valid_actions.remove(Action.NW)

#### 6. Cull waypoints 

There are two possibilities to optimize the waypoints:

1. Collinearity check
 I used the concept of the collinearity check to prune waypoints in a line, as explained in the lesson 7. The collinearity check calculates the determinant of a matrix which consists of three consecutive waypoints.

2. Bresenham ray tracing
 Bresenham algorithm provides a ray tracing from a start point to a goal point. To use the Bresenham, we have to convert the grid waypoints to a graph waypoints. The path is represented as nodes and edges.

### Extra Challenges: Real World Planning Implementation

Based on the lesson 8, I modified the starter code of probabilistic roadmap exercise. In this implementation, I use the graph representation instead of the grid representation for the waypoints. First, I created a class KdSampler which conducts the sampling generation and the collision check. The KdSampler class and the helper functions are implemented in `prob_roadmap.py`. 

Due to the computing time of sampling the whole map, I generate the random sampling in a circular area between a start and a goal point. The figure below shows the 1000 random samples in a circle, implemented in the function `gen_circular_random(center, r, num_samples, area)`:

![circle sampling][circle]

Thus, I can generate dense samples in the smaller area that is more significant for the A star planning algorithm. From the start and goal, I can compute the distance. The middle of this distance is the center of the circle. Furthermore, I modified the A* algorithm so that I can use it to find the best path using the graph representation.

## Quick Start

1. Download the Motion-Planning simulator for this project from the [simulator releases respository](https://github.com/udacity/FCND-Simulator-Releases/releases).
2. Clone this repository
3. Setup the Python environment by cloning the [starter kit](https://github.com/udacity/FCND-Term1-Starter-Kit).
4. Activate the environment by running `source activate fcnd`
5. Open a new console, run the simulator and choose the motion planning simulator
6. Execute `python motion_planning.py` for testing the probabilistic roadmap or `python motion_planning.py --mode grid` for testing the grid solution using the collinearity check.

## Result
### Grid map and Collinearity Approach

The video [result_grid_collinear.mp4](./misc/result_grid_collinear.mp4) shows my result using the grid map and collinearity approach. We can observe that at the end the drone still have zig-zag waypoints.

### Probabilistic Roadmap Approach

To get the faster and better waypoints I implemented the probabilistic roadmap approach. The figure below shows the random sampling result on the left image. The right image shows the found path using the A*-graph.
![result][result_plot]

The video can be seen in [result180424.mp4](./misc/result180424.mp4).

The safety distance is essential to generate a safe motion planning. The gif image below shows the simulation if I don't incorporate the safety distance:

![wo_safety_dist][wo_safety_dist]

This is another possible result of incorporating the safety distance:

![res-2-plot][res-2-plot]

![res-2][res-2]

## Further Work

The next approach to deal with the dynamic real world planning is to implement the receding horizon algorithm and automatic replanning, which are not implemented here.
Based on the lesson 8, the receding horizon can be implemented by following these steps:

1. Load the colliders data
2. Discretize the search space into a grid or graph
3. Define a start and goal location
4. Find a coarse 2D plan from start to goal
5. Choose a location along that plan and discretize a local volume around that location (for example, you might try a 40x40 m area that is 10 m high discretized into 1m^3 voxels)
6. Define the goal in the local volume to a a node or voxel at the edge of the volume in the direction of the next waypoint in the coarse global plan.
7. Plan a path through the 3D grid or graph to that node or voxel at the edge of the local volume.


[//]: # (References)
[result_plot]: ./misc/result180424.png
[wo_safety_dist]: ./misc/without_safety_dist.gif
[res-2]: ./misc/result180424-2.gif
[res-2-plot]: ./misc/result180424-2.png
[circle]: ./misc/circle_sampling.png