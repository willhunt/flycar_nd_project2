# %%
import numpy as np
import pandas as pd
from planner import RrtStar
from planning_utils import obstacles_2D
from udacidrone.frame_utils import global_to_local

# Read in obstacle map
data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)

TARGET_ALTITUDE = 5
SAFETY_DISTANCE = 5

obstacle_list, map_limits = obstacles_2D(data, TARGET_ALTITUDE, SAFETY_DISTANCE)

start_lonlat = (-122.397451, 37.792481, 5)
goal_lonlat = (-122.398416, 37.793698, 5)

map_start = (0, 0)

local_goal = global_to_local(goal_lonlat, start_lonlat)
map_goal = (local_goal[0], local_goal[1])

rrt_star = RrtStar( start = map_start,
                    goal = map_goal,
                    map_limits = [map_limits[0], map_limits[1], map_limits[2], map_limits[3]],
                    obstacle_list = obstacle_list,
                    path_max = 50.0,
                    max_iter = 10000)

rrt_star.plan(animation=False)
rrt_star.draw_map()


