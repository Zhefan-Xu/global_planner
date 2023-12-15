# Global Planner Library for Autonomous Robots
This package is a library implementing some of the global waypoint planning algorithms, such as RRT, RRT*, DEP (an unknown exploration planner), for autonomous robots. 

### Install:
Tested on Ubuntu 18.04, 20.04 with ROS melodic/Noetic. Current version depends on [Octomap](http://wiki.ros.org/octomap).
```
cd ~/catkin_ws/src
git clone https://github.com/Zhefan-Xu/global_planner.git
cd ~/catkin_ws
catkin_make
```
### Run Planner DEMO:
Start the simulator by running:
```
roslaunch global_planner rrtInteractive.launch
```
Use ```2D Nav Goal``` in ```Rviz``` to select start and goal position in the map.
![Screenshot from 2022-01-22 11-47-43](https://user-images.githubusercontent.com/55560905/150648123-8c1d9102-0b44-4851-82f5-fff0101be0ac.png)


### Parameters:
RRT planner paramters can be changed in ```global_planner/cfg/planner.yaml```. The followings are the default values: 
- ```collision_box: [0.4, 0.4, 0.4]```
- ```env_box: [-100, 100, -100, 100, 0, 1.5]```
- ```timeout: 2.0```
- ```rrt_incremental_distance: 0.3```
- ```rrt_connect_goal_ratio: 0.2```
- ```goal_reach_distance: 0.4```
- ```map_resolution: 0.2```
- ```vis_RRT: False``` (Not available for RRT*)
- ```vis_path: True```
- ```neighborhood_radius: 1.0``` (RRT* Only)
- ```max_num_neighbors: 10``` (RRT* Only)

### Code Exmaple & API:
Please see example ```global_planner/src/rrtInteractiveNode```. The example shows how to set start and goal position, and also how to find path. 

Note: the planner needs to call octomap service ```octomap_binary```, make sure to turn on your ```octomap server```.
