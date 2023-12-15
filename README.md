# Global Planner Library for Autonomous Robots
This package is a library implementing some global waypoint planning algorithms, such as RRT, RRT*, [DEP](https://github.com/Zhefan-Xu/DEP) (our unknown exploration planner), based on occupancy voxel map and Octomap for autonomous mobile robots. 

**Author**: [Zhefan Xu](https://zhefanxu.com/) and Christopher Suzuki from the Computational Engineering & Robotics Lab (CERLAB) at Carnegie Mellon University (CMU).


## I. Installation Guide
This repo has been tested on ROS Melodic with Ubuntu 18.04 and ROS Noetic with Ubuntu 20.04 and it depends on [map_manager](https://github.com/Zhefan-Xu/map_manager) which provides the occupancy voxel map implementation and [octomap_ros](http://wiki.ros.org/octomap) for octree-based map. 

```
# install dependency
sudo apt install ros-[melodic/noetic]-octomap* # octomap

cd ~/catkin_ws/src
git clone https://github.com/Zhefan-Xu/map_manager.git # occupancy voxel map. please refer to the original repo if you meet any issue.
git clone https://github.com/Zhefan-Xu/global_planner.git
cd ~/catkin_ws
catkin_make
```
## II. Run Planner DEMO
For simple RRT-based planner demo, please run the following commands:
```
roslaunch global_planner rrtInteractive.launch
```
Use ```2D Nav Goal``` in ```Rviz``` to select start and goal position in the map as shown below:

![Screenshot from 2022-01-22 11-47-43](https://user-images.githubusercontent.com/55560905/150648123-8c1d9102-0b44-4851-82f5-fff0101be0ac.png)


## III. Parameters
- RRT-based planner paramters can be found in ```global_planner/cfg/planner.yaml```. The followings are the default values: 
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
    
- DEP planner parameters (for unknown exploration) can be found in ```global_planner/cfg/dep_params.yaml```.

## IV. Code Exmaple & API
Please see example ```global_planner/src/rrt_interactive_node.cpp``` for RRT-based planner and ```global_planner/src/test_dep_node.cpp``` for unkonwn exploration planner. The example shows how to use the code API for your applications. 

## V. Issues
If you cannot visulize the Octomap with ROS noetic, that is because the [octomap_ros](http://wiki.ros.org/octomap) package has some tf name incompatibility issue. To solve that, please try building the octomap_ros from [source](https://github.com/OctoMap/octomap_mapping) with the following steps:

**Step1: download the octomap mapping source files**

```
cd ~/catkin_ws/src
git clone https://github.com/OctoMap/octomap_mapping.git
```

**Step2: modify source files and catkin make**
Please change all the frame id ```/map``` to ```map``` in ```octomap_mapping/octomap_server/OctomapServer.cpp``` and ```octomap_mapping/octomap_server/octomap_server_static.cpp```. There should be 2 places. 
```
cd ~/catkin_ws
catkin_make
```
