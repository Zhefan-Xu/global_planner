/*
*	File: dep.h
*	---------------
*   dynamic exploration planner header file
*/

#ifndef DEP_H
#define DEP_H

#include <map_manager/dynamicMap.h>
#include <nav_msgs/Odometry.h>
#include <global_planner/PRMKDTree.h>


namespace globalPlanner{
	class DEP{
	private:
		std::string ns_;
		std::string hint_;

		ros::NodeHandle nh_;
		ros::Subscriber odomSub_;

		nav_msgs::Odometry odom_;
		std::shared_ptr<mapManager::dynamicMap> map_; 
		std::shared_ptr<PRM::KDTree> roadmap_;


		// parameters
		std::string odomTopic_;
		Eigen::Vector3d globalRegionSize_;
		Eigen::Vector3d localRegionSize_;

	public:
		DEP(const ros::NodeHandle& nh);

		void initMap(const std::shared_ptr<mapManager::dynamicMap>& map);
		void initParam();
		void initModules();
		void registerCallback();

		void makePlan();
		void buildRoadMap();
		void getBestViewCandidates();
		void findBestPath();

		// callback functions
		void odomCB(const nav_msgs::OdometryConstPtr& odom);

		// help function
		std::shared_ptr<PRM::Node> randomConfigBBox();

	};
}


#endif


