# Probablistic Road Map (PRM)

### Author: Rico Jia

### Description
Probablistic Road Map provides nodes in unoccupied areas and edges that connect them. 
Due to the robot size, the nodes should be far from obstacles. 

In this package, a grid map of the PRM is also provided. 

An implementation of the PRM is shown below. The obstacles are marked in red, free paths are in blue, 
 and nodes are in yellow. 

 <img src="https://user-images.githubusercontent.com/39393023/79608851-4dc1af00-80bb-11ea-80ac-ce465d7ddabb.png" alt="Kitten" title="A cute kitten" width="400" />
-  
The grid map is the following, light grey areas are free spaces, dark grey areas are buffer zones where the robot might hit an obstacle, 
black zones are obstacles.   
 <img src="https://user-images.githubusercontent.com/39393023/79704567-ab9af600-8277-11ea-81d2-f29b19eb05c2.png" alt="Kitten" title="A cute kitten" width="400" />

                                                                                                                                               
### Usage
To see the PRM, do
```
$ roslaunch prm visualize_prm.launch visualization_mode:=1
```

To see the grid map, do
```$xslt
roslaunch prm visualize_prm.launch visualization_mode:=0
```

To change parameters, go to ```prm/config/params.yaml```

### Additional Information
- Service
 ```
update_grid_map_data (defined in grid_map_node.cpp)
 ```