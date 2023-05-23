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
#include <global_planner/PRMAstar.h>
#include <global_planner/utils.h>



namespace globalPlanner{
	class DEP{
	private:
		std::string ns_;
		std::string hint_;

		ros::NodeHandle nh_;
		ros::Publisher roadmapPub_;
		ros::Publisher candidatePathPub_;
		ros::Publisher bestPathPub_;
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
		std::vector<double> yaws_;
		double minVoxelThresh_;
		int minCandidateNum_;
		int maxCandidateNum_;

		// data
		bool odomReceived_ = false;
		Eigen::Vector3d position_;
		double currYaw_;
		std::vector<std::shared_ptr<PRM::Node>> prmNodeVec_; // all nodes		
		std::vector<std::shared_ptr<PRM::Node>> goalCandidates_;
		std::vector<std::vector<std::shared_ptr<PRM::Node>>> candidatePaths_;
		std::vector<std::shared_ptr<PRM::Node>> bestPath_;


	public:
		DEP(const ros::NodeHandle& nh);

		void setMap(const std::shared_ptr<mapManager::dynamicMap>& map);
		void initParam();
		void initModules();
		void registerPub();
		void registerCallback();

		bool makePlan();
		void buildRoadMap();
		void updateInformationGain();
		void getBestViewCandidates(std::vector<std::shared_ptr<PRM::Node>>& goalCandidates);
		void findCandidatePath(const std::vector<std::shared_ptr<PRM::Node>>& goalCandidates);
		void findBestPath(const std::vector<std::vector<std::shared_ptr<PRM::Node>>>& candidatePaths);


		// callback functions
		void odomCB(const nav_msgs::OdometryConstPtr& odom);
		void visCB(const ros::TimerEvent&);

		// visualization functions
		void publishRoadmap();
		void publishCandidatePaths();
		void publishBestPath();

		// help function
		std::shared_ptr<PRM::Node> randomConfigBBox(const Eigen::Vector3d& region);
		bool sensorRangeCondition(const shared_ptr<PRM::Node>& n1, const shared_ptr<PRM::Node>& n2);
		bool sensorFOVCondition(const Eigen::Vector3d& sample, const Eigen::Vector3d& pos);
		int calculateUnknown(const shared_ptr<PRM::Node>& n, std::unordered_map<double, int>& yawNumVoxels);
		double calculatePathLength(const std::vector<shared_ptr<PRM::Node>> path);

	};
}


#endif


