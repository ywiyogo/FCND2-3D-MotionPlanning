import argparse
import time
import msgpack
from enum import Enum, auto

import numpy as np

from planning_utils import a_star, heuristic, create_grid, prune_path
from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
from udacidrone.frame_utils import global_to_local, local_to_global

import matplotlib.pyplot as plt
import prob_roadmap as prmap

class States(Enum):
    MANUAL = auto()
    ARMING = auto()
    TAKEOFF = auto()
    WAYPOINT = auto()
    LANDING = auto()
    DISARMING = auto()
    PLANNING = auto()

# YW ATTENTION:
# local coordinate is always represented in NED
# global coordinate is always represented in geodatic (lon, lat, alt)

class MotionPlanning(Drone):
    '''
    The MotionPlanning class inherits the Drone class of udacidrone
    '''
    def __init__(self, connection, planningmode):
        super().__init__(connection)

        self.target_position = np.array([0.0, 0.0, 0.0])
        self.waypoints = []
        self.in_mission = True
        self.check_state = {}
        self.mode = planningmode
        # initial state
        self.flight_state = States.MANUAL

        # register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        if self.flight_state == States.TAKEOFF:
            if -1.0 * self.local_position[2] > 0.95 * self.target_position[2]:
                self.waypoint_transition()
        elif self.flight_state == States.WAYPOINT:
            if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < 1.0:
                if len(self.waypoints) > 0:
                    self.waypoint_transition()
                else:
                    if np.linalg.norm(self.local_velocity[0:2]) < 1.0:
                        self.landing_transition()

    def velocity_callback(self):
        if self.flight_state == States.LANDING:
            if self.global_position[2] - self.global_home[2] < 0.1:
                if abs(self.local_position[2]) < 0.01:
                    self.disarming_transition()

    def state_callback(self):
        if self.in_mission:
            if self.flight_state == States.MANUAL:
                self.arming_transition()
            elif self.flight_state == States.ARMING:
                if self.armed:
                    self.plan_path()
            elif self.flight_state == States.PLANNING:
                self.takeoff_transition()
            elif self.flight_state == States.DISARMING:
                if ~self.armed & ~self.guided:
                    self.manual_transition()

    def arming_transition(self):
        self.flight_state = States.ARMING
        print("arming transition")
        self.arm()
        self.take_control()

    def takeoff_transition(self):
        self.flight_state = States.TAKEOFF
        print("takeoff transition")
        self.takeoff(self.target_position[2])

    def waypoint_transition(self):
        self.flight_state = States.WAYPOINT
        print("waypoint transition")
        self.target_position = self.waypoints.pop(0)
        print('target position', self.target_position)
        self.cmd_position(self.target_position[0], self.target_position[1], self.target_position[2], self.target_position[3])

    def landing_transition(self):
        self.flight_state = States.LANDING
        print("landing transition")
        self.land()

    def disarming_transition(self):
        self.flight_state = States.DISARMING
        print("disarm transition")
        self.disarm()
        self.release_control()

    def manual_transition(self):
        self.flight_state = States.MANUAL
        print("manual transition")
        self.stop()
        self.in_mission = False

    def send_waypoints(self):
        print("Sending waypoints to simulator ...")
        data = msgpack.dumps(self.waypoints)
        self.connection._master.write(data)

    def plan_path(self):
        self.flight_state = States.PLANNING
        print("Searching for a path ...")
        TARGET_ALTITUDE = 5
        SAFETY_DISTANCE = 5

        self.target_position[2] = TARGET_ALTITUDE
        # -------------------------------------
        # YW [1]:  read lat0, lon0 from colliders into floating point values
        # -------------------------------------
        with open('colliders.csv') as f:
            first_line = f.readline()
            geo_lat_lon=[]
            for col in [x.strip() for x in first_line.split(',')]:
                geo_lat_lon.append(float(col.split(' ')[1]))
                
        print("Global home coordinate in Lat-Long: ",geo_lat_lon)
        # YW: set home position to (lon0, lat0, 0). Beware of the lat0, lon0 in collider.csv!
        self.set_home_position(geo_lat_lon[1], geo_lat_lon[0], 0)
        
        # -------------------------------------
        # YW [2]: retrieve current global position
        # -------------------------------------
        global_pose = self.global_position
        print("Current global pose (lon, lat, alt): ", global_pose)
        print("Global home coord (lon, lat, alt): ", self.global_home)
        print("\n")

        # YW: convert to current local position using global_to_local()
        cur_pose_local = global_to_local(global_pose, self.global_home)
        print("------------------------")
        print("local pose: ", cur_pose_local)
        print('global home {0} \nglobal position {1} \nlocal position {2}'.format(self.global_home, self.global_position,
                                                                         self.local_position))
        print("------------------------\n")

        # -------------------------------------
        # YW [3]: Read in obstacle map
        # -------------------------------------
        data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)
        int_cur_pose_local = cur_pose_local.astype(int)
        print("local int_cur_pose_local: ", int_cur_pose_local)
        
        global_goal = [ -122.396580, 37.796085, self._altitude] 
        
        print("global_goal (long-lat-alt): ", global_goal)
        local_goal = global_to_local(global_goal, self.global_home) 
        int_local_goal = local_goal.astype(int)
        print("local_goal : ", int_local_goal)
        # Define a grid for a particular altitude and safety margin around obstacles
        grid, north_min, east_min = create_grid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)
        print("North min = {0}, east min = {1}".format(north_min, east_min))
        # plt.imshow(grid, origin='lower') 
        # plt.xlabel('EAST')
        # plt.ylabel('NORTH')
        # plt.show()

        grid_start = (int_cur_pose_local[0] - north_min, int_cur_pose_local[1] - east_min)
        # YW: convert start position to current position rather than map center
        # Don't add tuple and numpy array
        print("grid_start: ", grid_start)
        # Set goal as some arbitrary position on the grid
        # -------------------------------------
        # YW: adapt to set goal as latitude / longitude position and convert
        # use the manual mode to get the appropriate goal coordinate
        # -------------------------------------
        # convert the local NORTH EAST direction to the grid map
        grid_goal = (int_local_goal[0] - north_min, 
                    int_local_goal[1] - east_min)
        #grid_goal = (-north_min + 20, - east_min )
        print("grid_goal: ", grid_goal)
        print("Grid size: ", grid.shape)
        # Run A* to find a path from start to goal
        # YW: add diagonal motions with a cost of sqrt(2) to your A* implementation
        # or move to a different search space such as a graph (not done here) -> planning_utils
        print('Local Start and Goal: ', grid_start, " -> ", grid_goal)
        print("------------------------\n") 
        
        if self.mode == 'grid':
            path, _ = a_star(grid, heuristic, grid_start, grid_goal)
            # YW [6]: prune path to minimize number of waypoints
            #  (if you're feeling ambitious): Try a different approach altogether!
            path = prune_path(path)
            print("Path: ",path)
            # Convert path to waypoints
            waypoints = [[p[0] + north_min, p[1] + east_min, TARGET_ALTITUDE, 0] for p in path]
            print("Waypoints: ",waypoints)
        elif self.mode== "graph":
            # Use KdSampler instead of create grid
            graph_start = tuple(int_cur_pose_local.tolist())
            graph_goal = tuple(int_local_goal.tolist())
            # map_data_range = [-316, -445, 595, 466]
            # start = [map_data_range[0] + 100, map_data_range[1] + 100]
            # goal =  [map_data_range[2] - 200, map_data_range[3] - 150]
            print("Start: {0}, goal: {1}".format(graph_start, graph_goal))

            kdtree_sampler = prmap.KdSampler(data, 140, graph_start, graph_goal, SAFETY_DISTANCE)
            
            print("Number of samples: ", len(kdtree_sampler._kept_samples))
            print("Number of rejected samples: ", len(kdtree_sampler._removed_samples))
            t0 = time.time()
            # YW NOTE: given a large amount of k allows the algorithm to connect
            # the nodes that has a long distance

            g = prmap.create_graph(kdtree_sampler, 10)
            print('graph took {0} seconds to build'.format(time.time() - t0))
            path, cost = prmap.a_star_graph(g, heuristic, graph_start, graph_goal)
            print("A* graph length {0}, path: {1} \ncost: {2}".format(len(path), path, cost) )
            prmap.visualize_graph(grid, data, g, kdtree_sampler, path)

            # Convert path to waypoints
            waypoints = [[p[0], p[1], TARGET_ALTITUDE, 0] for p in path]
        else:
            print("Error: Unknown planning mode!")

        print("Waypoints: ",waypoints)
        # Set self.waypoints
        self.waypoints = waypoints
        # send waypoints to sim (this is just for visualization of waypoints)
        self.send_waypoints()

    def start(self):
        self.start_log("Logs", "NavLog.txt")

        print("starting connection")
        self.connection.start()

        # Only required if they do threaded
        # while self.in_mission:
        #    pass

        self.stop_log()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    # YW: add 2 modes: grid and graph (real world challenge)
    parser.add_argument('--mode', type=str, default='graph', help="mode grid/graph, i.e '--mode grid'")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), timeout=60)
    drone = MotionPlanning(conn, args.mode)
    time.sleep(1)

    drone.start()
