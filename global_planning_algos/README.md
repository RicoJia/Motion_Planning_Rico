# Global Path Planning Algorithms

### Author: Rico Ruotong Jia

### Description
This package currently has 1.A star algorithm 2. theta star algorithm 3. a visualization node. The PRM package is used to provide 
a valid set of map points to choose from for waypoint selection. 

##### Algorithm Description 
###### A star

 <img src="https://user-images.githubusercontent.com/39393023/80922483-084fe380-8d43-11ea-9882-59d19478622c.png" alt="Kitten" title="A cute kitten" width="400" />

In A star, the algorithm finds waypoints from any given previous waypoint's closest neighbors. Then, if an edge between two 
waypoints is away from any obstacle by at least safe distance (robot radius), then this edge is added. Since the PRM package has provided 
non occupied obstacles, no extra occupancy check is performed here. 

###### Theta Star
 <img src="https://user-images.githubusercontent.com/39393023/80922481-071eb680-8d43-11ea-813b-8b1fe5fa07e6.png" alt="Kitten" title="A cute kitten" width="400" />
 
Theta star was implemented as a child class of A star and inherits all the above features. The most significant addition is line of sight checking. 
If a waypoint's parent waypoint and one of its closest neighbors can form a collision-free, line-of-sight edge, then this edge might be considered
for planning. 

##### Not reachable goal
In both algorithms, If start or goal are not reachable, a ROS_Fatal message will be printed, and the visualization node will die.     

### Usage
To visualize A star path, do 
```
$ roslaunch global_planning_algos global_planning.launch algo_select:=0
```

To visualize theta star path, do
```
$ roslaunch global_planning_algos global_planning.launch algo_select:=1
```

