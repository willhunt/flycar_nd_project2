# Project 2, 3D Motion Planning
## Udacity Flying Car Nanodegree
3D motion planning project for the Udacity flying car Nanodegree program. For the project an alternative solution to the homework has been implemented in order to explore more advanced topics. The RRT* algortihm has been coded to find a path to the goal in downtown San Francisco.

## How to Launch the simulation?
#### Download the Udacity simulator
[Latest release](https://github.com/udacity/FCND-Simulator-Releases/releases)

#### Clone this package
```sh
$ cd /home/<project folder>/ 
$ git clone https://github.com/willhunt/flycar_nd_project2.git
```

#### Launch simulator
Launch the Udacity drone simulator and select the Planning option. There is currently a bug (v0.1.1) where the drone starts inside a building (released 2 years ago so does not look likely to be fixed....). To start fly the drone out of the building and disarm. This can be doneby flying forwards and up at the same time (space bar + W). 

#### Run the code
```sh
$ python motion_planning.py
```
## RRT* Planner
The RRT* planning algorithm has been implemented to solve the project. The planner can be tested in the `playground.py` file. Some of the code has been adapted from the [Python Robotics](https://github.com/AtsushiSakai/PythonRobotics) package although significant modifications have been made including:
* Rectangular obstacles instead of circular
* Use of KD-Trees for nearest obstacle and node lookup to increase speed for large graphs
* When paths cross obstacles points are placed near edge of object rather than searching for a new point
* Final path optimisation to remove unecessary waypoints

### Planner results
200 Iterations    | 1,000 Iterations | 10,000 Iterations
:-------------:   |:-----------------: |:-----------------: 
<img src="/assets/Figure1.png" alt="200iters" width="auto" height="180"> | <img src="/assets/Figure2.png" alt="1000iters" width="auto" height="180"> | <img src="/assets/Figure3.png" alt="10000iters" width="auto" height="180">    


## Notes
The Udacity drone simulator uses up a lot of resources so the RRT* planning is slow. The planner can be testing in the `playground.py` file which plots the route through the buildings.

Starter code from Udacity [FCND-Motion-Planning](https://github.com/udacity/FCND-Motion-Planning).