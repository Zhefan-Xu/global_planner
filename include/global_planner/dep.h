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
#include <global_planner/utils.h>


namespace globalPlanner{
	class DEP{
	private:
		std::string ns_;
		std::string hint_;

		ros::NodeHandle nh_;
		ros::Publisher roadmapPub_;
		ros::Subscriber odomSub_;
		ros::Timer visTimer_;

		nav_msgs::Odometry odom_;
		std::shared_ptr<mapManager::dynamicMap> map_; 
		std::shared_ptr<PRM::KDTree> roadmap_;


		// parameters
		std::string odomTopic_;
		Eigen::Vector3d globalRegionSize_;
		Eigen::Vector3d localRegionSize_;
		int sampleThresh_;
		double distThresh_;
		double horizontalFOV_;
		double verticalFOV_;
		double dmin_;
		double dmax_;
		int nnNum_;
		double maxConnectDist_;

		// data
		bool odomReceived_ = false;
		Eigen::Vector3d position_;
		double currYaw_;
		std::vector<double> yaws;
		std::map<double, int> yawNumVoxels;

		// visualization data
		std::vector<std::shared_ptr<PRM::Node>> prmNodeVec_;

	public:
		DEP(const ros::NodeHandle& nh);

		void setMap(const std::shared_ptr<mapManager::dynamicMap>& map);
		void initParam();
		void initModules();
		void registerPub();
		void registerCallback();

		bool makePlan();
		void buildRoadMap();
		void getBestViewCandidates();
		void findBestPath();


		// callback functions
		void odomCB(const nav_msgs::OdometryConstPtr& odom);
		void visCB(const ros::TimerEvent&);

		// visualization functions
		void publishRoadmap();

		// help function
		std::shared_ptr<PRM::Node> randomConfigBBox(const Eigen::Vector3d& region);
		bool sensorRangeCondition(const shared_ptr<PRM::Node> n1, shared_ptr<PRM::Node> n2);
		bool sensorFOVCondition(const Eigen::Vector3d n);
		double calculateUnknown(const shared_ptr<PRM::Node> n);

	};
}


#endif


