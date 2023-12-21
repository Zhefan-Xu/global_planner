/*
*	FILE: rrtOccMap.h
*	---------------------------
*	RRT planner based on occupancy map
*/

#ifndef RRTOCCMAP_H
#define RRTOCCMAP_H
#include <ros/ros.h>
#include <global_planner/rrtBase.h>
#include <Eigen/Eigen>
#include <nav_msgs/Path.h>
#include <map_manager/occupancyMap.h>
#include <visualization_msgs/MarkerArray.h>

namespace globalPlanner{
	template <std::size_t N>
	class rrtOccMap : public rrtBase<N>{
	protected:
		ros::NodeHandle nh_;
		ros::Publisher rrtVisPub_;
		ros::Timer visTimer_;
		std::shared_ptr<mapManager::occMap> map_;

		double mapRes_;
		double envLimit_[6];
		double sampleRegion_[6];
		double maxShortcutThresh_;
		bool ignoreUnknown_;
		bool passGoalCheck_;

		std::vector<KDTree::Point<N>> currPlan_;

	public:
		rrtOccMap(const ros::NodeHandle& nh);
		~rrtOccMap();
		void initParam();
		void registerPub();
		void registerCallback();
		void setMap(const std::shared_ptr<mapManager::occMap>& map);
		void updateMap();
		void updateSampleRegion();
		void makePlan(std::vector<KDTree::Point<N>>& plan);
		void makePlan(nav_msgs::Path& path);

		void randomConfig(KDTree::Point<N>& qRand);
		void shortcutWaypointPaths(const std::vector<KDTree::Point<N>>& plan, std::vector<KDTree::Point<N>>& planSc);
		bool isCurrPathValid();
		bool hasNewGoal(const geometry_msgs::Pose& goal);
		void pathMsgConverter(const std::vector<KDTree::Point<N>>& pathTemp, nav_msgs::Path& path);
		bool checkCollision(const KDTree::Point<N>& q);

		void visCB(const ros::TimerEvent&);
		void publishRRTPath();
	}; 

	template <std::size_t N>
	rrtOccMap<N>::rrtOccMap(const ros::NodeHandle& nh) : rrtBase<N>(), nh_(nh){
		this->initParam();
		this->registerPub();
		this->registerCallback();
	}

	template <std::size_t N>
	void rrtOccMap<N>::initParam(){
		// load parameters:
		// Map Resolution for Collisiong checking
		if (not this->nh_.getParam("rrt/map_resolution", this->mapRes_)){
			this->mapRes_ = 0.2;
			cout << "[RRTPlanner]: No Map Resolition Parameter. Use default map resolution: 0.2." << endl;
		}
		else{
			cout << "[RRTPlanner]: Map resolution: " << this->mapRes_  << "." << endl;
		}

		// Timeout:
		if (not this->nh_.getParam("rrt/timeout", this->timeout_)){
			this->timeout_ = 0.5;
			cout << "[RRTPlanner]: No Time Out Parameter. Use default: 0.5." << endl;
		}
		else{
			cout << "[RRTPlanner]: Time out is set to: " << this->timeout_  << "." << endl;
		}

		// Environment Size (maximum)
		if (not this->nh_.getParam("rrt/env_box", this->envBox_)){
			std::vector<double> defaultEnvBox {-100, 100, -100, 100, 0, 1.5};
			cout << "[RRTPlanner]: No Environment Box Parameter. Use default env box: [-100, 100, -100, 100, 0, 1.5]." << endl;
		}

		// Incremental Distance (For RRT)
		if (not this->nh_.getParam("rrt/incremental_distance", this->delQ_)){
			this->delQ_ = 0.3;
			cout << "[RRTPlanner]: No RRT Incremental Distance Parameter. Use default value: 0.3 m." << endl;
		}
		else{
			cout << "[RRTPlanner]: RRT Incremental Distance: " << this->delQ_ << " m." << endl;
		}

		// Goal Reach Distance
		if (not this->nh_.getParam("rrt/goal_reach_distance", this->dR_)){
			this->dR_ = 0.4;
			cout << "[RRTPlanner]: No RRT Goal Reach Distance Parameter. Use default value: 0.4 m." << endl;
		}
		else{
			cout << "[RRTPlanner]: RRT Goal Reach Distance: " << this->dR_ << " m." << endl;
		}

		// RRT Goal Connect Ratio
		if (not this->nh_.getParam("rrt/connect_goal_ratio", this->connectGoalRatio_)){
			this->connectGoalRatio_ = 0.2;
			cout << "[RRTPlanner]: No Goal Connect Ratio Parameter. Use default: 0.2." << endl;
		}
		else{
			cout << "[RRTPlanner]: Goal Connect Ratio: " << this->connectGoalRatio_  << "." << endl;
		}	

		// maximum shortcut threshold
		if (not this->nh_.getParam("rrt/max_shortcut_dist", this->maxShortcutThresh_)){
			this->maxShortcutThresh_ = 5.0;
			cout << "[RRTPlanner]: No Max Shortcut Distance Threshold Parameter. Use default: 5.0." << endl;
		}	
		else{
			cout << "[RRTPlanner]: Max Shortcut Distance: " << this->maxShortcutThresh_ << " m." << endl;
		}

		// ignore unknown
		if (not this->nh_.getParam("rrt/ignore_unknown", this->ignoreUnknown_)){
			this->ignoreUnknown_ = false;
			cout << "[RRTPlanner]: No ignore unknown Parameter. Use default: false." << endl;
		}	
		else{
			cout << "[RRTPlanner]: Ignore unknown: " << this->ignoreUnknown_ << "." << endl;
		}

		// goal check
		if (not this->nh_.getParam("rrt/pass_goal_check", this->passGoalCheck_)){
			this->passGoalCheck_ = false;
			cout << "[RRTPlanner]: No pass goal check Parameter. Use default: false." << endl;
		}	
		else{
			cout << "[RRTPlanner]: Pass goal check: " << this->passGoalCheck_ << "." << endl;
		}

	}

	template <std::size_t N>
	rrtOccMap<N>::~rrtOccMap(){}

	template <std::size_t N>
	void rrtOccMap<N>::registerPub(){
		this->rrtVisPub_ = this->nh_.advertise<visualization_msgs::MarkerArray>("rrt/planned_path", 100);
	}

	template <std::size_t N>
	void rrtOccMap<N>::registerCallback(){
		this->visTimer_ = this->nh_.createTimer(ros::Duration(0.1), &rrtOccMap<N>::visCB, this);
	}

	template <std::size_t N>
	void rrtOccMap<N>::setMap(const std::shared_ptr<mapManager::occMap>& map){
		this->map_ = map;
		Eigen::Vector3d mapSizeMin, mapSizeMax;
		this->map_->getMapRange(mapSizeMin, mapSizeMax);	
		this->envLimit_[0] = mapSizeMin(0); this->envLimit_[1] = mapSizeMax(0); this->envLimit_[2] = mapSizeMin(1); this->envLimit_[3] = mapSizeMax(1); this->envLimit_[4] = mapSizeMin(2); this->envLimit_[5] = mapSizeMax(2);
		this->updateSampleRegion();
	}

	template <std::size_t N>
	void rrtOccMap<N>::updateMap(){}

	template <std::size_t N>
	void rrtOccMap<N>::updateSampleRegion(){
		double xmin = std::max(this->envBox_[0], this->envLimit_[0]); this->sampleRegion_[0] = xmin;
		double xmax = std::min(this->envBox_[1], this->envLimit_[1]); this->sampleRegion_[1] = xmax;
		double ymin = std::max(this->envBox_[2], this->envLimit_[2]); this->sampleRegion_[2] = ymin;
		double ymax = std::min(this->envBox_[3], this->envLimit_[3]); this->sampleRegion_[3] = ymax;
		double zmin = std::max(this->envBox_[4], this->envLimit_[4]); this->sampleRegion_[4] = zmin;
		double zmax = std::min(this->envBox_[5], this->envLimit_[5]); this->sampleRegion_[5] = zmax;
	}

	template <std::size_t N>
	void rrtOccMap<N>::makePlan(std::vector<KDTree::Point<N>>& plan){
		Eigen::Vector3d qGoal = KDTree::point2Eig(this->goal_);
		if (not this->passGoalCheck_){
			if (not this->ignoreUnknown_){
				if (this->map_->isUnknown(qGoal)){
					plan.push_back(this->start_);
					cout << "[Global Planner]: Goal is unknown. Please change your goal." << endl;
					return;
				}
			}
		}


		ros::Time startTime = ros::Time::now();
		ros::Time currTime;
		bool findPath = false;
		bool timeout = false;
		double dT;
		int sampleNum = 0;
		KDTree::Point<N> qBack;

		// cout << "[RRTPlanner]: Start Planning!" << endl;
		double nearestDistance = std::numeric_limits<double>::max();
		KDTree::Point<N> nearestPoint = this->start_;
		double currentDistance = KDTree::Distance(nearestPoint, this->goal_);
		this->addVertex(this->start_);
	
		while (ros::ok() and not findPath and not timeout){
			currTime = ros::Time::now();
			dT = (currTime - startTime).toSec();
			if (dT >= this->timeout_){
				timeout = true;
			}

			// 1. random sample
			KDTree::Point<N> qRand;
			double randomValue = randomNumber(0, 1);
			if (randomValue >= this->connectGoalRatio_){
				this->randomConfig(qRand);
			}
			else{
				qRand = this->goal_;
			}

			// 2. find nearest neighbor
			KDTree::Point<N> qNear;
			this->nearestVertex(qRand, qNear);

			// 3. new config by steering functino
			KDTree::Point<N> qNew;
			this->newConfig(qNear, qRand, qNew);

			// 4. Add new config to vertex and edge
			Eigen::Vector3d qNewEig = KDTree::point2Eig(qNew);
			Eigen::Vector3d qNearEig = KDTree::point2Eig(qNear);
			// if (this->hasNoEdge(qNear, qNew) and not this->map_->isInflatedOccupiedLine(qNewEig, qNearEig)){
			bool lineValidCheck;
			// if (this->ignoreUnknown_){
			lineValidCheck = not this->map_->isInflatedOccupiedLine(qNewEig, qNearEig);
			// }
			// else{
				// lineValidCheck = this->map_->isInflatedFreeLine(qNewEig, qNearEig);
			// }
			if (this->hasNoEdge(qNear, qNew) and lineValidCheck){
				this->addVertex(qNew);
				this->addEdge(qNear, qNew);
				++sampleNum;

				// 5. check whehter goal is reached
				findPath = this->isReach(qNew);
				if (findPath){
					qBack = qNew;
				}
				else{
					currentDistance = KDTree::Distance(qNew, this->goal_);
					if (currentDistance < nearestDistance){
						nearestDistance = currentDistance;
						nearestPoint = qNew;
					}
				}
			}
		}

		// cout << "[RRTPlanner]: Finish planning with sample number: " << sampleNum << endl;

		// backtrace
		std::vector<KDTree::Point<N>> planRaw;
		if (findPath){
			this->backTrace(qBack, planRaw);
			// cout << "[RRTPlanner]: Path Found! Time: " << dT << "s. " << endl;
		}
		else{
			this->backTrace(nearestPoint, planRaw);
			if (planRaw.size() == 1){
				plan = planRaw;
				cout << "[RRTPlanner]: TIMEOUT! Start position might not be feasible!!" << endl;
				return;
			}
			else{
				cout << "[RRTPlanner]: TIMEOUT!"<< "(>" << this->timeout_ << "s)" << ", Return closest path. Distance: " << nearestDistance << " m." << endl;
			}
		}

		// short cut path
		this->shortcutWaypointPaths(planRaw, plan);
		this->currPlan_ = plan;
	}

	template <std::size_t N>
	void rrtOccMap<N>::makePlan(nav_msgs::Path& path){
		std::vector<KDTree::Point<N>> planTemp;
		this->makePlan(planTemp);
		this->pathMsgConverter(planTemp, path);
	}

	template <std::size_t N>
	void rrtOccMap<N>::randomConfig(KDTree::Point<N>& qRand){
		bool valid = false;
		Eigen::Vector3d p (-100, -100, -100);
		while (ros::ok() and not valid){
			p(0) = randomNumber(this->sampleRegion_[0], this->sampleRegion_[1]);
			p(1) = randomNumber(this->sampleRegion_[2], this->sampleRegion_[3]);
			p(2) = randomNumber(this->sampleRegion_[4], this->sampleRegion_[5]);
			if (this->ignoreUnknown_){
				valid = not this->map_->isInflatedOccupied(p);
			}
			else{
				valid = this->map_->isInflatedFree(p);
			}
		}
		qRand[0] = p(0); qRand[1] = p(1); qRand[2] = p(2);
	}

	template <std::size_t N>
	void rrtOccMap<N>::shortcutWaypointPaths(const std::vector<KDTree::Point<N>>& plan, std::vector<KDTree::Point<N>>& planSc){
		size_t ptr1 = 0; size_t ptr2 = 2;
		planSc.push_back(plan[ptr1]);

		if (plan.size() == 1){
			return;
		}

		if (plan.size() == 2){
			planSc.push_back(plan[1]);
			return;
		}

		while (ros::ok()){
			if (ptr2 > plan.size()-1){
				break;
			}
			KDTree::Point<N> p1 = plan[ptr1]; KDTree::Point<N> p2 = plan[ptr2];
			Eigen::Vector3d pos1 = KDTree::point2Eig(p1);
			Eigen::Vector3d pos2 = KDTree::point2Eig(p2);
			// if (not this->map_->isInflatedOccupiedLine(pos1, pos2) and KDTree:: Distance(p1, p2) <= this->maxShortcutThresh_){
			bool lineValidCheck;
			// if (this->ignoreUnknown_){
			lineValidCheck = not this->map_->isInflatedOccupiedLine(pos1, pos2);
			// }
			// else{
				// lineValidCheck = this->map_->isInflatedFreeLine(pos1, pos2);
			// }
			if (lineValidCheck and KDTree:: Distance(p1, p2) <= this->maxShortcutThresh_){
				if (ptr2 == plan.size()-1){
					planSc.push_back(p2);
					break;
				}
				++ptr2;
			}
			else{
				if (ptr2 == plan.size()-1){
					planSc.push_back(p2);
					break;
				}
				planSc.push_back(plan[ptr2-1]);
				ptr1 = ptr2-1;
				ptr2 = ptr1+2;
			}
		}
	}

	template <std::size_t N>
	bool rrtOccMap<N>::isCurrPathValid(){
		if (this->currPlan_.size() == 0){
			return false;
		}
		for (size_t i=0; i<this->currPlan_.size()-1; ++i){
			KDTree::Point<N> currP = this->currPlan_[i];
			KDTree::Point<N> nextP = this->currPlan_[i+1];
			Eigen::Vector3d currPEig = KDTree::point2Eig(currP);
			Eigen::Vector3d nextPEig = KDTree::point2Eig(nextP);
			// if (this->map_->isInflatedOccupiedLine(currPEig, nextPEig)){
			bool lineValidCheck;
			// if (this->ignoreUnknown_){
			lineValidCheck = not this->map_->isInflatedOccupiedLine(currPEig, nextPEig);
			// }
			// else{
				// lineValidCheck = this->map_->isInflatedFreeLine(currPEig, nextPEig);
			// }
			if (not lineValidCheck){
				return false;
			}
		}
		return true;
	}

	template <std::size_t N>
	bool rrtOccMap<N>::hasNewGoal(const geometry_msgs::Pose& goal){
		bool notNewGoal = (goal.position.x == this->goal_[0]) and (goal.position.y == this->goal_[1]) and (goal.position.z == this->goal_[2]);
		if (notNewGoal){
			return false;
		}
		else{
			return true;
		}
	}

	template <std::size_t N>
	void rrtOccMap<N>::pathMsgConverter(const std::vector<KDTree::Point<N>>& pathTemp, nav_msgs::Path& path){
		std::vector<geometry_msgs::PoseStamped> pathVec;
		for (KDTree::Point<N> p: pathTemp){
			geometry_msgs::PoseStamped ps;
			ps.header.stamp = ros::Time();
			ps.header.frame_id = "map";
			ps.pose.position.x = p[0];
			ps.pose.position.y = p[1];
			ps.pose.position.z = p[2];
			pathVec.push_back(ps);
		}
		path.poses = pathVec;
		path.header.stamp = ros::Time();
		path.header.frame_id = "map";
	}

	template <std::size_t N>
	bool rrtOccMap<N>::checkCollision(const KDTree::Point<N>& q){return false;}

	template <std::size_t N>
	void rrtOccMap<N>::visCB(const ros::TimerEvent&){
		this->publishRRTPath();
	}

	template <std::size_t N>
	void rrtOccMap<N>::publishRRTPath(){
		std::vector<visualization_msgs::Marker> pathVisVec;
		visualization_msgs::Marker waypoint;
		visualization_msgs::Marker line;
		geometry_msgs::Point p1, p2;
		std::vector<geometry_msgs::Point> lineVec;
		for (size_t i=0; i < this->currPlan_.size(); ++i){
			KDTree::Point<N> currentPoint = this->currPlan_[i];
			if (i != this->currPlan_.size() - 1){
				KDTree::Point<N> nextPoint = this->currPlan_[i+1];
				p1.x = currentPoint[0];
				p1.y = currentPoint[1];
				p1.z = currentPoint[2];
				p2.x = nextPoint[0];
				p2.y = nextPoint[1];
				p2.z = nextPoint[2]; 
				lineVec.push_back(p1);
				lineVec.push_back(p2);
			}
			// waypoint
			waypoint.header.frame_id = "map";
			waypoint.id = 1+i;
			waypoint.ns = "rrt_path";
			waypoint.type = visualization_msgs::Marker::SPHERE;
			waypoint.pose.position.x = currentPoint[0];
			waypoint.pose.position.y = currentPoint[1];
			waypoint.pose.position.z = currentPoint[2];
			waypoint.lifetime = ros::Duration(0.5);
			waypoint.scale.x = 0.2;
			waypoint.scale.y = 0.2;
			waypoint.scale.z = 0.2;
			waypoint.color.a = 0.8;
			waypoint.color.r = 0.3;
			waypoint.color.g = 1;
			waypoint.color.b = 0.5;
			pathVisVec.push_back(waypoint);
		}
		line.header.frame_id = "map";
		line.points = lineVec;
		line.ns = "rrt_path";
		line.id = 0;
		line.type = visualization_msgs::Marker::LINE_LIST;
		line.lifetime = ros::Duration(0.5);
		line.scale.x = 0.05;
		line.scale.y = 0.05;
		line.scale.z = 0.05;
		line.color.a = 1.0;
		line.color.r = 0.5;
		line.color.g = 0.1;
		line.color.b = 1;
		pathVisVec.push_back(line);		
	}
}

#endif