# Global Path Planning Algorithms

### Author: Rico Ruotong Jia

### Description
This package currently has 1.Lifelong Planning A* 2. D* Lite 3. a visualization node. The PRM package is used to provide 
a grid map for visualization and planning . 

##### Algorithm Description 
###### Life-long Planning A Star

 <img src="https://user-images.githubusercontent.com/39393023/80922483-084fe380-8d43-11ea-9882-59d19478622c.png" alt="Kitten" title="A cute kitten" width="400" />

In A star, the algorithm finds waypoints from any given previous waypoint's closest neighbors. Then, if an edge between two 
waypoints is away from any obstacle by at least safe distance (robot radius), then this edge is added. Since the PRM package has provided 
non occupied obstacles, no extra occupancy check is performed here. 

###### D Star Lite 
 <img src="https://user-images.githubusercontent.com/39393023/80922481-071eb680-8d43-11ea-813b-8b1fe5fa07e6.png" alt="Kitten" title="A cute kitten" width="400" />
 
##### Not reachable goal
In both algorithms, If start or goal are not reachable, a ROS_Fatal message will be printed, and the visualization node will die.     

### Usage
To visualize LPA* path, do 
```
$ roslaunch global_planning_algos global_planning.launch algo_select:=3
```

To visualize D* Lite path, do
```
$ roslaunch global_planning_algos global_planning.launch algo_select:=4
```

