# Write-up for Project 2 - 3D Motion Planning
## Explanantion of provided code
#### Task  
Explain what's going on in  `motion_planning.py` and `planning_utils.py`
#### Response
* The code is mainly the same as in Project 1 except that the waypoints are defined by a path planning function, `plan_path` which is called when the drone is in the *armed* state.
* The function `plan_path`:
    * Loads in 3D map data with obstacles defined as cubes by centre values and dimension. 
    * Creates 2D map from data to reflect slice at an altitude of 5m. This is done over a discretised grid checking cell by cell if an obstacle is present.
    * Defines arbitrary goal point
    * Run A* seacrh algorithm to find a path to the goal. Actions are defined as moving up/left/down/right to neighbouring grid cells which are checked for validity (obstacle free, within map). The heuristic function is defined as the euclidean distance to the goal.
    * Convert grid cells into waypoints in required coordinate system.

## Implementation of planner
#### Task
Adapt A* planning function or implement alternative as discussed in "Real World Planning" lesson.
#### Response
I have implemented the RRT* planning algorithm to solve the project. The planner can be tested in the `playground.py` file. Some of the code has been adapted from the [Python Robotics](https://github.com/AtsushiSakai/PythonRobotics) package although significant modifications have been made including:
* Total rewrite
* Rectangular obstacles instead of curcular
* Use of KD-Trees for nearest obstacle and node lookup to increase speed for large graphs
* When paths cross obstacles points are placed near edge of object rather than searching for a new point
* Final path optimisation to remove unecessary waypoints

The Udacity drone simulator uses up a lot of resources so the RRT* planning is slow. The planner can be testing in the `playground.py` file which plots the route through the buildings.
