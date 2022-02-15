/*
*	File: rrtOctomap.h
*	---------------
*   RRT planner class based on Octomap.
*/
#ifndef RRTOCTOMAP_H
#define RRTOCTOMAP_H
#include<ros/ros.h>
#include <global_planner/rrtBase.h>
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/GetOctomap.h>
#include <octomap_msgs/conversions.h>
#include <nav_msgs/Path.h>
#include <limits>
#include <visualization_msgs/MarkerArray.h>
#include <thread>
#include <mutex>


namespace globalPlanner{
	template <std::size_t N>
	class rrtOctomap : public rrtBase<N>{
	private:
		ros::NodeHandle nh_;

	protected:
		ros::ServiceClient mapClient_;
		ros::Publisher RRTVisPub_;
		ros::Publisher pathVisPub_;

		double mapRes_;
		double envLimit_[6];
		double sampleRegion_[6];
		octomap::OcTree* map_;
		
		visualization_msgs::MarkerArray RRTVisMsg_;
		visualization_msgs::MarkerArray pathVisMsg_;
		std::vector<visualization_msgs::Marker> RRTVisvec_; // we update this
		std::vector<visualization_msgs::Marker> pathVisVec_; // update this
		bool visRRT_;
		bool visPath_;
	

	public:
		std::thread RRTVisWorker_;
		std::thread pathVisWorker_;

		// default constructor
		rrtOctomap();

		// constructor using ros param:
		rrtOctomap(const ros::NodeHandle& nh);

		// constructor using point format
		rrtOctomap(const ros::NodeHandle& nh, KDTree::Point<N> start, KDTree::Point<N> goal, std::vector<double> collisionBox, std::vector<double> envBox, double mapRes, double delQ=0.3, double dR=0.2, double connectGoalRatio=0.10, double timeout=1.0, bool visRRT=false, bool visPath=true);

		// constructor using vector format
		rrtOctomap(const ros::NodeHandle& nh, std::vector<double> start, std::vector<double> goal,  std::vector<double> collisionBox, std::vector<double> envBox, double mapRes, double delQ=0.3, double dR=0.2, double connectGoalRatio=0.10, double timeout=1.0, bool visRRT=false, bool visPath=true);

		// constructor without start and goal
		rrtOctomap(const ros::NodeHandle& nh, std::vector<double> collisionBox, std::vector<double> envBox, double mapRes, double delQ=0.3, double dR=0.2, double connectGoalRatio=0.10, double timeout=1.0, bool visRRT=false, bool visPath=true);
		
		// constructor without nh	
		rrtOctomap(std::vector<double> collisionBox, std::vector<double> envBox, double mapRes, double delQ=0.3, double dR=0.2, double connectGoalRatio=0.10, double timeout=1.0, bool visRRT=false, bool visPath=true);


		// update octomap
		virtual void updateMap();	
		void updateSampleRegion();// helper function for update sample region
		
		// collision checking function based on map and collision box: TRUE => Collision
		virtual bool checkCollision(const KDTree::Point<N>& q);
		bool checkCollision(const octomap::point3d& p);
		bool checkCollisionPoint(const octomap::point3d &p, bool ignoreUnkown=true);
		bool checkCollisionLine(const KDTree::Point<N>& q1, const KDTree::Point<N>& q2);
		bool checkCollisionLine(const octomap::point3d& p1, const octomap::point3d& p2);

		// shortcut path
		void shortcutWaypointPaths(const std::vector<KDTree::Point<N>>& plan, std::vector<KDTree::Point<N>>& planSc);

		// random sample in valid space (based on current map)
		virtual void randomConfig(KDTree::Point<N>& qRand);

		// *** Core function: make plan based on all input ***
		virtual void makePlan(std::vector<KDTree::Point<N>>& plan);
		virtual void makePlan(nav_msgs::Path& plan);

		// Visualization
		void startVisModule();
		void publishRRTVisMsg();
		void publishPathVisMsg();
		void updateRRTVisVec(const KDTree::Point<N>& qNew, const KDTree::Point<N>& qNear, int id);
		void updatePathVisVec(const std::vector<KDTree::Point<N>> &plan);

		// Helper function for collision checking:
		void point2Octomap(const KDTree::Point<N>& q, octomap::point3d& p);

		// Helper function: convert to ROS message
		void pathMsgConverter(const std::vector<KDTree::Point<N>>& pathTemp, nav_msgs::Path& path);


		double getMapRes();
	};


	// ===========================Function Definition=======================================
	template <std::size_t N>
	rrtOctomap<N>::rrtOctomap() : rrtBase<N>(){}

	template <std::size_t N>
	rrtOctomap<N>::rrtOctomap(const ros::NodeHandle& nh) : nh_(nh), rrtBase<N>(){
		// load parameters:
		// Map Resolution for Collisiong checking
		if (not this->nh_.getParam("map_resolution", this->mapRes_)){
			this->mapRes_ = 0.2;
			cout << "[Global Planner INFO]: No Map Resolition Parameter. Use default map resolution: 0.2." << endl;
		}

		// Visualize RRT 
		if (not this->nh_.getParam("vis_RRT", this->visRRT_)){
			this->visRRT_ = false;
			cout << "[GLobal Planner INFO]: No RRT visualization by default." << endl;
		}

		// Visualize Path
		if (not this->nh_.getParam("vis_path", this->visPath_)){
			this->visPath_ = true;
			cout << "[Global Planner INFO]: Visualize Path by default." << endl;
		}

		// Collision Box
		if (not this->nh_.getParam("collision_box", this->collisionBox_)){
			std::vector<double> defaultCollisionBox {1.0, 1.0, 0.6};
			this->collisionBox_ = defaultCollisionBox;
			cout << "[Global Planner INFO]: No Collision Box Parameter. Use default collision Box: [1.0, 1.0, 0.6]." << endl;
		}

		// Environment Size (maximum)
		if (not this->nh_.getParam("env_box", this->envBox_)){
			std::vector<double> defaultEnvBox {-100, 100, -100, 100, 0, 1.5};
			cout << "[Global Planner INFO]: No Environment Box Parameter. Use default env box: [-100, 100, -100, 100, 0, 1.5]." << endl;
		}

		// Incremental Distance (For RRT)
		if (not this->nh_.getParam("rrt_incremental_distance", this->delQ_)){
			this->delQ_ = 0.3;
			cout << "[Global Planner INFO]: No RRT Incremental Distance Parameter. Use default value: 0.3m." << endl;
		}

		// Goal Reach Distance
		if (not this->nh_.getParam("goal_reach_distance", this->dR_)){
			this->dR_ = 0.4;
			cout << "[Global Planner INFO]: No RRT Goal Reach Distance Parameter. Use default value: 0.4m." << endl;
		}

		// RRT Connect Goal Ratio
		if (not this->nh_.getParam("rrt_connect_goal_ratio", this->connectGoalRatio_)){
			this->connectGoalRatio_ = 0.2;
			cout << "[Global Planner INFO]: No RRT Connect Goal Ratio Parameter. Use default value: 0.2." << endl;
		}

		// Time out:
		if (not this->nh_.getParam("timeout", this->timeout_)){
			this->timeout_ = 2.0;
			cout << "[Global Planner INFO]: No Timeout Parameter. Use default value: 3.0s." << endl;
		}


		this->mapClient_ = this->nh_.serviceClient<octomap_msgs::GetOctomap>("/octomap_binary");
		this->updateMap();

		// Visualization:
		this->startVisModule();
	
	}

	template <std::size_t N>
	rrtOctomap<N>::rrtOctomap(const ros::NodeHandle& nh, KDTree::Point<N> start, KDTree::Point<N> goal, std::vector<double> collisionBox, std::vector<double> envBox, double mapRes, double delQ, double dR, double connectGoalRatio, double timeout, bool visRRT, bool visPath)
	: nh_(nh), mapRes_(mapRes), visRRT_(visRRT), visPath_(visPath), rrtBase<N>(start, goal, collisionBox, envBox, delQ, dR, connectGoalRatio){
		this->mapClient_ = this->nh_.serviceClient<octomap_msgs::GetOctomap>("/octomap_binary");
		this->updateMap();

		// Visualization:
		this->startVisModule();
	}

	template <std::size_t N>
	rrtOctomap<N>::rrtOctomap(const ros::NodeHandle& nh, std::vector<double> start, std::vector<double> goal,  std::vector<double> collisionBox, std::vector<double> envBox, double mapRes, double delQ, double dR, double connectGoalRatio, double timeout, bool visRRT, bool visPath)
	: nh_(nh), mapRes_(mapRes), visRRT_(visRRT), visPath_(visPath), rrtBase<N>(start, goal, collisionBox, envBox, delQ, dR, connectGoalRatio, timeout){
		this->mapClient_ = this->nh_.serviceClient<octomap_msgs::GetOctomap>("/octomap_binary");
		this->updateMap();

		// Visualization:
		this->startVisModule();
	}

	template <std::size_t N>
	rrtOctomap<N>::rrtOctomap(const ros::NodeHandle& nh, std::vector<double> collisionBox, std::vector<double> envBox, double mapRes, double delQ, double dR, double connectGoalRatio, double timeout, bool visRRT, bool visPath)
	:  nh_(nh), mapRes_(mapRes), visRRT_(visRRT), visPath_(visPath), rrtBase<N>(collisionBox, envBox, delQ, dR, connectGoalRatio, timeout){
		this->mapClient_ = this->nh_.serviceClient<octomap_msgs::GetOctomap>("/octomap_binary");
		this->updateMap();
		
		// Visualization:
		this->startVisModule();
	}
	
	template <std::size_t N>
	rrtOctomap<N>::rrtOctomap(std::vector<double> collisionBox, std::vector<double> envBox, double mapRes, double delQ, double dR, double connectGoalRatio, double timeout, bool visRRT, bool visPath)
	: mapRes_(mapRes), visRRT_(visRRT), visPath_(visPath), rrtBase<N>(collisionBox, envBox, delQ, dR, connectGoalRatio, timeout){}




	template <std::size_t N>
	void rrtOctomap<N>::updateMap(){
		octomap_msgs::GetOctomap mapSrv;
		bool service_success = this->mapClient_.call(mapSrv);
		ros::Rate rate(10);
		while (not service_success and ros::ok()){
			service_success = this->mapClient_.call(mapSrv);
			ROS_INFO("[Global Planner INFO]: Wait for Octomap Service...");
			rate.sleep();
		}
		octomap::AbstractOcTree* abtree = octomap_msgs::binaryMsgToMap(mapSrv.response.map);
		this->map_ = dynamic_cast<octomap::OcTree*>(abtree);
		// this->map_->setResolution(this->mapRes_);
		double min_x, max_x, min_y, max_y, min_z, max_z;
		this->map_->getMetricMax(max_x, max_y, max_z);
		this->map_->getMetricMin(min_x, min_y, min_z);
		this->envLimit_[0] = min_x; this->envLimit_[1] = max_x; this->envLimit_[2] = min_y; this->envLimit_[3] = max_y; this->envLimit_[4] = min_z; this->envLimit_[5] = max_z;
		this->updateSampleRegion();
		cout << "[Global Planner INFO]: Map updated!" << endl;
	}

	template <std::size_t N>
	void rrtOctomap<N>::updateSampleRegion(){
		double xmin = std::max(this->envBox_[0], this->envLimit_[0]); this->sampleRegion_[0] = xmin;
		double xmax = std::min(this->envBox_[1], this->envLimit_[1]); this->sampleRegion_[1] = xmax;
		double ymin = std::max(this->envBox_[2], this->envLimit_[2]); this->sampleRegion_[2] = ymin;
		double ymax = std::min(this->envBox_[3], this->envLimit_[3]); this->sampleRegion_[3] = ymax;
		double zmin = std::max(this->envBox_[4], this->envLimit_[4]); this->sampleRegion_[4] = zmin;
		double zmax = std::min(this->envBox_[5], this->envLimit_[5]); this->sampleRegion_[5] = zmax;
	}

	template <std::size_t N>
	bool rrtOctomap<N>::checkCollision(const KDTree::Point<N>& q){
		octomap::point3d p;
		this->point2Octomap(q, p);
		return this->checkCollision(p);
	}

	template <std::size_t N>
	bool rrtOctomap<N>::checkCollision(const octomap::point3d& p){
		double xmin, xmax, ymin, ymax, zmin, zmax; // bounding box for collision checking
		xmin = p.x() - this->collisionBox_[0]/2; xmax = p.x() + this->collisionBox_[0]/2;
		ymin = p.y() - this->collisionBox_[1]/2; ymax = p.y() + this->collisionBox_[1]/2;
		zmin = p.z() - this->collisionBox_[2]/2; zmax = p.z() + this->collisionBox_[2]/2;

		int xNum = (xmax - xmin)/this->mapRes_;
		int yNum = (ymax - ymin)/this->mapRes_;
		int zNum = (zmax - zmin)/this->mapRes_;

		int xID, yID, zID;
		for (xID=0; xID<=xNum; ++xID){
			for (yID=0; yID<=yNum; ++yID){
				for (zID=0; zID<=zNum; ++zID){
					if (this->checkCollisionPoint(octomap::point3d(xmin+xID*this->mapRes_, ymin+yID*this->mapRes_, zmin+zID*this->mapRes_), false)){
						return true;
					}
				}
			}
		}
		return false;
	}

	template <std::size_t N>
	bool rrtOctomap<N>::checkCollisionPoint(const octomap::point3d &p, bool ignoreUnkown){
		octomap::OcTreeNode* nptr = this->map_->search(p);
		if (nptr == NULL){
			if (not ignoreUnkown){
				return true;
			}
			else{
				return false;
			}
		}
		return this->map_->isNodeOccupied(nptr);
	}

	template <std::size_t N>
	bool rrtOctomap<N>::checkCollisionLine(const KDTree::Point<N>& q1, const KDTree::Point<N>& q2){
		octomap::point3d p1, p2;
		this->point2Octomap(q1, p1);
		this->point2Octomap(q2, p2);
		return this->checkCollisionLine(p1, p2);
	}
	
	template <std::size_t N>
	bool rrtOctomap<N>::checkCollisionLine(const octomap::point3d& p1, const octomap::point3d& p2){
		std::vector<octomap::point3d> ray;
		this->map_->computeRay(p1, p2, ray);
		if (this->checkCollision(p2)){
			return true;
		}

		for (octomap::point3d p: ray){
			if (this->checkCollision(p)){
				return true;
			}
		}
		return false;
	}

	template <std::size_t N>
	void rrtOctomap<N>::shortcutWaypointPaths(const std::vector<KDTree::Point<N>>& plan, std::vector<KDTree::Point<N>>& planSc){
		int ptr1 = 0; int ptr2 = 1;
		planSc.push_back(plan[ptr1]);
		while (true and ros::ok()){
			KDTree::Point<N> p1 = plan[ptr1]; KDTree::Point<N> p2 = plan[ptr2];
			if (not checkCollisionLine(p1, p2)){
				if (ptr2 >= plan.size()-1){
					planSc.push_back(p2);
					break;
				}
				++ptr2;
			}
			else{
				planSc.push_back(plan[ptr2-1]);
				ptr1 = ptr2-1;
			}
		}
	}

	template <std::size_t N>
	void rrtOctomap<N>::randomConfig(KDTree::Point<N>& qRand){
		bool valid = false;
		double x, y, z;
		octomap::point3d p;
		while (not valid){
			p.x() = randomNumber(this->sampleRegion_[0], this->sampleRegion_[1]);
			p.y() = randomNumber(this->sampleRegion_[2], this->sampleRegion_[3]);
			p.z() = randomNumber(this->sampleRegion_[4], this->sampleRegion_[5]);
			valid = not this->checkCollision(p);
		}

		qRand[0] = p.x(); qRand[1] = p.y(); qRand[2] = p.z();
	}

	template <std::size_t N>
	void rrtOctomap<N>::makePlan(std::vector<KDTree::Point<N>>& plan){
		if (this->visRRT_){
			this->RRTVisvec_.clear();
			this->RRTVisMsg_.markers = this->RRTVisvec_;
		}

		if (this->visPath_){
			this->pathVisVec_.clear();
			this->pathVisMsg_.markers = this->pathVisVec_;
		}

		bool findPath = false;
		bool timeout = false;
		ros::Time startTime = ros::Time::now();
		double dT;
		int sampleNum = 0;
		KDTree::Point<N> qBack;

		cout << "[Global Planner INFO]: Start planning!" << endl;
		double nearestDistance = std::numeric_limits<double>::max();  // if cannot find path to goal, find nearest way to goal
		KDTree::Point<N> nearestPoint = this->start_;
		double currentDistance = KDTree::Distance(nearestPoint, this->goal_);
		this->addVertex(this->start_);
		while (ros::ok() and not findPath and not timeout){	
			ros::Time currentTime = ros::Time::now();
			dT = (currentTime - startTime).toSec();
			if (dT >= this->timeout_){
				timeout = true;
			}

			// 1. sample:
			KDTree::Point<N> qRand;
			double randomValue = randomNumber(0, 1);
			if (randomValue >= this->connectGoalRatio_){ // random sample trick
				this->randomConfig(qRand);
			}
			else{
				qRand = this->goal_;
			}

			// 2. find nearest neighbor:
			KDTree::Point<N> qNear;
			this->nearestVertex(qRand, qNear);

			// 3. new config by steering function:
			KDTree::Point<N> qNew;
			this->newConfig(qNear, qRand, qNew);

			// 4. Add new config to vertex and edge:
			if (not this->checkCollisionLine(qNew, qNear)){
				this->addVertex(qNew);
				this->addEdge(qNear, qNew);
				++sampleNum;

				// 5. check whether goal has been reached
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

				// visualization:
				if (this->visRRT_){
					this->updateRRTVisVec(qNew, qNear, sampleNum);
				}
			}
		}
		cout << "[Global Planner INFO]: Finish planning. with sample number: " << sampleNum << endl;

		// final step: back trace using the last one
		std::vector<KDTree::Point<N>> planRaw;
		if (findPath){
			this->backTrace(qBack, planRaw);
			cout << "[Global Planner INFO]: path found! Time: " << dT << "s."<< endl;
		}
		else{
			this->backTrace(nearestPoint, planRaw);
			if (planRaw.size() == 1){
				plan = planRaw;
				cout << "[Global Planner INFO]: TIMEOUT! Start position might not be feasible!!" << endl;
				return;
			}
			else{
				cout << "[Global Planner INFO]: TIMEOUT!"<< "(>" << this->timeout_ << "s)" << ", Return closest path. Distance: " << nearestDistance << " m." << endl;
			}
		}
		this->shortcutWaypointPaths(planRaw, plan);

		// visualization
		if (this->visPath_){
			this->updatePathVisVec(plan);
			this->pathVisMsg_.markers = this->pathVisVec_;
		}
	}

	template <std::size_t N>
	void rrtOctomap<N>::makePlan(nav_msgs::Path& plan){
		std::vector<KDTree::Point<N>> planTemp;
		this->makePlan(planTemp);
		this->pathMsgConverter(planTemp, plan);
	}

	template <std::size_t N>
	void rrtOctomap<N>::startVisModule(){
		if (this->visRRT_){
			this->RRTVisPub_ = this->nh_.advertise<visualization_msgs::MarkerArray>("/rrt_vis_array", 1);
			this->RRTVisWorker_ = std::thread(&rrtOctomap<N>::publishRRTVisMsg, this);
		}

		if (this->visPath_){
			this->pathVisPub_ = this->nh_.advertise<visualization_msgs::MarkerArray>("/rrt_planned_path", 10);
			this->pathVisWorker_ = std::thread(&rrtOctomap<N>::publishPathVisMsg, this);
		}
	}

	template <std::size_t N>
	void rrtOctomap<N>::publishRRTVisMsg(){
		ros::Rate rate(5);
		while (ros::ok()){
			this->RRTVisMsg_.markers = this->RRTVisvec_;
			this->RRTVisPub_.publish(this->RRTVisMsg_);
			rate.sleep();
		}
	}
	
	template <std::size_t N>
	void rrtOctomap<N>::publishPathVisMsg(){
		ros::Rate rate(20);
		while (ros::ok()){
			this->pathVisPub_.publish(this->pathVisMsg_);
			rate.sleep();
		}
	}

	template <std::size_t N>
	void rrtOctomap<N>::updateRRTVisVec(const KDTree::Point<N>& qNew, const KDTree::Point<N>& qNear, int id){
		visualization_msgs::Marker point;
		visualization_msgs::Marker line;
		geometry_msgs::Point p1, p2;
		std::vector<geometry_msgs::Point> lineVec;
		
		// point:
		point.header.frame_id = "map";
		point.ns = "RRT_point";
		point.id = id;
		point.type = visualization_msgs::Marker::SPHERE;
		point.pose.position.x = qNew[0];
		point.pose.position.y = qNew[1];
		point.pose.position.z = qNew[2];
		point.lifetime = ros::Duration(1);
		point.scale.x = 0.2;
		point.scale.y = 0.2;
		point.scale.z = 0.2;
		point.color.a = 0.8;
		point.color.r = 1;
		point.color.g = 0;
		point.color.b = 0;
		this->RRTVisvec_.push_back(point);

		// line:
		p1.x = qNew[0];
		p1.y = qNew[1];
		p1.z = qNew[2];
		p2.x = qNear[0];
		p2.y = qNear[1];
		p2.z = qNear[2]; 
		lineVec.push_back(p1);
		lineVec.push_back(p2);

		line.header.frame_id = "map";
		line.ns = "RRT_line";
		line.points = lineVec;
		line.id = id;
		line.type = visualization_msgs::Marker::LINE_LIST;
		line.lifetime = ros::Duration(1);
		line.scale.x = 0.05;
		line.scale.y = 0.05;
		line.scale.z = 0.05;
		line.color.a = 1.0;
		line.color.r = 1*0.5;
		line.color.g = 0;
		line.color.b = 1;
		this->RRTVisvec_.push_back(line);
	}

	template <std::size_t N>
	void rrtOctomap<N>::updatePathVisVec(const std::vector<KDTree::Point<N>> &plan){
		this->pathVisVec_.clear();
		visualization_msgs::Marker waypoint;
		visualization_msgs::Marker line;
		geometry_msgs::Point p1, p2;
		std::vector<geometry_msgs::Point> lineVec;
		for (int i=0; i < plan.size(); ++i){
			KDTree::Point<N> currentPoint = plan[i];
			if (i != plan.size() - 1){
				KDTree::Point<N> nextPoint = plan[i+1];
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
			this->pathVisVec_.push_back(waypoint);
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
		this->pathVisVec_.push_back(line);
	}
	

	template <std::size_t N>
	void rrtOctomap<N>::point2Octomap(const KDTree::Point<N>& q, octomap::point3d& p){
		p.x() = q[0];
		p.y() = q[1];
		p.z() = q[2];
	}

	template <std::size_t N>
	void rrtOctomap<N>::pathMsgConverter(const std::vector<KDTree::Point<N>>& pathTemp, nav_msgs::Path& path){
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
	double rrtOctomap<N>::getMapRes(){
		return this->mapRes_;
	}


	// =================Operator overload==============================
	template <std::size_t N>
	std::ostream &operator<<(std::ostream &os, rrtOctomap<N> &rrtplanner){
        os << "========================INFO========================\n";
        os << "[Global Planner INFO]: RRT planner with octomap\n";
        os << "[Connect Ratio]: " << rrtplanner.getConnectGoalRatio() << "\n";
        // os << "[Start/Goal]:  " <<  rrtplanner.getStart() << "=>" <<  rrtplanner.getGoal() << "\n";
        std::vector<double> collisionBox = rrtplanner.getCollisionBox();
        os << "[Collision Box]: " << collisionBox[0] << " " << collisionBox[1] << " " <<  collisionBox[2] << "\n";
        os << "[Map Res]: " << rrtplanner.getMapRes() << "\n";
        os << "[Timeout]: " << rrtplanner.getTimeout() << "\n";
        os << "====================================================";
        return os;
    }
}

#endif