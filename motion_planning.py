import argparse
import time
import msgpack
from enum import Enum, auto

import numpy as np
import pandas as pd

from planning_utils import a_star, heuristic, create_grid, obstacles_2D
from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
from udacidrone.frame_utils import global_to_local

from planner import RrtStar


class States(Enum):
    MANUAL = auto()
    ARMING = auto()
    TAKEOFF = auto()
    WAYPOINT = auto()
    LANDING = auto()
    DISARMING = auto()
    PLANNING = auto()


class MotionPlanning(Drone):

    def __init__(self, connection):
        super().__init__(connection)

        self.target_position = np.array([0.0, 0.0, 0.0])
        self.waypoints = []
        self.in_mission = True
        self.check_state = {}

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

        # Read lat0, lon0 from colliders into floating point values
        header = pd.read_csv("colliders.csv", nrows=1, sep=', ')
        center_lonlat = []  # Centre of map
        for pos in header.columns:
            angle = float(pos.split(' ')[1])
            center_lonlat.insert(0, angle)  # CSV gives Latitude then Longitude
        # Set home position to (lon0, lat0, 0)
        self.set_home_position(center_lonlat[0], center_lonlat[1], 0)
        # self.set_home_position(-122.398794, 37.793355, 0)

        # Retrieve current global position
        # Convert to current local position using global_to_local()
        local_position = global_to_local(self.global_position, self.global_home)
        print('global home {0}, position {1}, local position {2}'.format(self.global_home, self.global_position,
                                                                         self.local_position))
        # Read in obstacle map
        data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)
        
        # Define a grid for a particular altitude and safety margin around obstacles
        grid, north_offset, east_offset = create_grid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)
        print("North offset = {0}, east offset = {1}".format(north_offset, east_offset))
        # Define starting point on the grid 
        local_start = local_position[:2]
        grid_start = (int(local_start[0] - north_offset), int(local_start[1] - east_offset))

        # Set goal as latitude / longitude position and convert
        goal_lonlat = [-122.397888, 37.793448, TARGET_ALTITUDE]
        local_goal = list(global_to_local(goal_lonlat, self.global_home)[:2])
        grid_goal = (int(local_goal[0] - north_offset), int(local_goal[1] - east_offset))

        # Run A* to find a path from start to goal
        print('Local Start and Goal: ', grid_start, grid_goal)
        path_astar, _ = a_star(grid, heuristic, grid_start, grid_goal)

        # I have implemented RRT*. It runs slowly withthe simulator unfortunatley as it is a resource hog but can be tested in the "playground.py" file
        # Create 2D slice of map data at set altitde
        obstacle_list, map_limits = obstacles_2D(data, TARGET_ALTITUDE, SAFETY_DISTANCE)

        rrt_star = RrtStar( start = local_start,
                            goal = local_goal,
                            map_limits = [map_limits[0], map_limits[1], map_limits[2], map_limits[3]],
                            obstacle_list = obstacle_list,
                            path_max = 50.0,
                            max_iter = 2500)
        rrt_star.plan()
        # rrt_star.draw_map()
        path_rrtstar = rrt_star.get_path()

        # Convert path to waypoints
        waypoints_astar = [[p[0] + north_offset, p[1] + east_offset, TARGET_ALTITUDE, 0] for p in path_astar]
        waypoints_rrtstar = [[int(p[0]), int(p[1]), TARGET_ALTITUDE, 0] for p in path_rrtstar]
        # Set self.waypoints
        # self.waypoints = waypoints_astar
        self.waypoints = waypoints_rrtstar
        # Send waypoints to sim (this is just for visualization of waypoints)
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
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), timeout=60)
    drone = MotionPlanning(conn)
    time.sleep(1)

    drone.start()
