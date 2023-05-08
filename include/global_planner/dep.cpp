/*
*	File: dep.cpp
*	---------------
*   dynamic exploration planner implementation
*/

#include <global_planner/dep.h>
#include<random>


namespace globalPlanner{
	DEP::DEP(const ros::NodeHandle& nh) : nh_(nh){
		this->ns_ = "/DEP";
		this->hint_ = "[DEP]";
		this->initParam();
		this->initModules();
		this->registerCallback();
	}

	void DEP::initMap(const std::shared_ptr<mapManager::dynamicMap>& map){
		this->map_ = map;
	}

	void DEP::initParam(){
		// odom topic name
		if (not this->nh_.getParam(this->ns_ + "/odom_topic", this->odomTopic_)){
			this->odomTopic_ = "/CERLAB/quadcopter/odom";
			cout << this->hint_ << ": No odom topic name. Use default: /CERLAB/quadcopter/odom" << endl;
		}
		else{
			cout << this->hint_ << ": Odom topic: " << this->odomTopic_ << endl;
		}

		// local sample region size
		std::vector<double> localRegionSizeTemp;	
		if (not this->nh_.getParam(this->ns_ + "/local_region_size", localRegionSizeTemp)){
			this->localRegionSize_(0) = 1.0;
			this->localRegionSize_(1) = 1.0;
			this->localRegionSize_(2) = 1.0;
			cout << this->hint_ << ": No local region size param. Use default: [1 1 1]" <<endl;
		}
		else{
			this->localRegionSize_(0) = localRegionSizeTemp[0];
			this->localRegionSize_(1) = localRegionSizeTemp[1];
			this->localRegionSize_(2) = localRegionSizeTemp[2];
			cout << this->hint_ << ": Local Region Size: " << this->localRegionSize_[0] << this->localRegionSize_[1]<< this->localRegionSize_[2]<< endl;
		}
		// global sample region size
		std::vector<double> globalRegionSizeTemp;	
		if (not this->nh_.getParam(this->ns_ + "/global_region_size", globalRegionSizeTemp)){
			this->globalRegionSize_(0) = 5.0;
			this->globalRegionSize_(1) = 5.0;
			this->globalRegionSize_(2) = 5.0;
			cout << this->hint_ << ": No global region size param. Use default: [5 5 5]" <<endl;
		}
		else{
			this->globalRegionSize_(0) = globalRegionSizeTemp[0];
			this->globalRegionSize_(1) = globalRegionSizeTemp[1];
			this->globalRegionSize_(2) = globalRegionSizeTemp[2];
			cout << this->hint_ << ": Global Region Size: " << this->globalRegionSize_[0] << this->globalRegionSize_[1]<< this->globalRegionSize_[2]<< endl;
		}
		
		// Sample Threshold Value
		if (not this->nh_.getParam(this->ns_ + "/sampleThresh", this->sampleThresh_)){
			this->sampleThresh_ = 50;
			cout << this->hint_ << ": No sample thresh param. Use default: 50" << endl;
		}
		else{
			cout << this->hint_ << ": Sample Thresh: " << this->sampleThresh_ << endl;
		}
		if (not this->nh_.getParam(this->ns_ + "/distThresh", this->distThresh_)){
			this->distThresh_ = 0.8;
			cout << this->hint_ << ": No distance thresh param. Use default: 50" << endl;
		}
		else{
			cout << this->hint_ << ": Distance Thresh: " << this->distThresh_ << endl;
		}

	}

	void DEP::initModules(){
		// initialize roadmap
		this->roadmap_.reset(new PRM::KDTree ());
	}

	void DEP::registerPub(){
		// roadmap visualization publisher
		this->roadmapPub_ = this->nh_.advertise<visualization_msgs::MarkerArray>("/dep/roadmap", 10);
	}

	void DEP::registerCallback(){
		// odom subscriber
		this->odomSub_ = this->nh_.subscribe(this->odomTopic_, 1000, &DEP::odomCB, this);
	
		// visualization timer
		this->visTimer_ = this->nh_.createTimer(ros::Duration(0.05), &DEP::visCB, this);
	}

	void DEP::makePlan(){
		if (not this->odomReceived_) return;
		this->buildRoadMap();

		this->getBestViewCandidates();

		this->findBestPath();
	}

	void DEP::buildRoadMap(){
		// local sample (TODO)
		//std::vector<PRM::Node*> new_nodes;
		// double threshold = 500; // HardCode threshold
		bool saturate = false;
		bool regionSaturate = false;
		int countSample = 0;
		while (ros::ok() and not saturate){
			std::shared_ptr<PRM::Node> n;
			
			if (regionSaturate){
				int countFailureGlobal = 0;
				// Generate new node
				while (ros::ok() and true){
					if (countFailureGlobal > this->sampleThresh_){
						saturate = true;
						break;
					}
					n = this->randomConfigBBox(this->globalRegionSize_);
					// Check how close new node is other nodes
					shared_ptr<PRM::Node> nn = this->roadmap_->nearestNeighbor(n);
					double distToNN = (n->pos - nn->pos).norm();
					if (distToNN < this->distThresh_){
						++countFailureGlobal;
					}
					else{
						this->roadmap_->insert(n);
						this->prmNodeVec_.push_back(n);
						++countSample;
						break;
					}
				}
			}
			else{
				if (true){
					int countFailureLocal = 0;
					// Generate new node
					while (ros::ok() and true){
						if (countFailureLocal > this->sampleThresh_){
							regionSaturate = true;
							break;
						}
						n = this->randomConfigBBox(this->localRegionSize_);
						// Check how close new node is other nodes
						shared_ptr<PRM::Node> nn = this->roadmap_->nearestNeighbor(n);
						double distToNN = (n->pos - nn->pos).norm();
						if (distToNN < this->distThresh_){
							++countFailure;
						}
						else{
							this->roadmap_->insert(n);
							this->prmNodeVec_.push_back(n);
							++countSample;
							break;
						}
					}
				}
			}
		}
		cout << "newly added: " << count_sample << " samples" << endl;
		

		// node connection


		// information gain update
	}	 

	void DEP::getBestViewCandidates(){

	}

	void DEP::findBestPath(){

	}

	void DEP::odomCB(const nav_msgs::OdometryConstPtr& odom){
		this->odom_ = *odom;
		this->position_ = Eigen::Vector3d (this->odom_.pose.pose.position.x, this->odom_.pose.pose.position.y, this->odom_.pose.pose.position.z);
		this->currYaw_ = globalPlanner::rpy_from_quaternion(this->odom_.pose.pose.orientation);
		this->odomReceived_ = true;
	}

	void DEP::visCB(const ros::TimerEvent&){
		if (this->prmNodeVec_.size() != 0){
			this->publishRoadmap();
		}
	}

	void DEP::publishRoadmap(){
		visualization_msgs::MarkerArray roadmapMarkers;
		int countPointNum = 0;
		for (size_t i=0; i<this->prmNodeVec_.size(); ++i){
			std::shared_ptr<PRM::Node> n = this->prmNodeVec_[i];

			// Node point
			visualization_msgs::Marker point;
			point.header.frame_id = "map";
			point.header.stamp = ros::Time::now();
			point.ns = "prm_point";
			point.id = countPointNum;
			point.type = visualization_msgs::Marker::SPHERE;
			point.action = visualization_msgs::Marker::ADD;
			point.pose.position.x = n->pos(0);
			point.pose.position.y = n->pos(1);
			point.pose.position.z = n->pos(2);
			point.lifetime = ros::Duration(0.1);
			point.scale.x = 0.2;
			point.scale.y = 0.2;
			point.scale.z = 0.2;
			point.color.a = 1.0;
			point.color.r = 1.0;
			point.color.g = 0.0;
			point.color.b = 0.0;
			++countPointNum;
			roadmapMarkers.markers.push_back(point);
		}
		this->roadmapPub_.publish(roadmapMarkers);
	}



	// TODO: add map range
	std::shared_ptr<PRM::Node> DEP::randomConfigBBox(const Eigen::Vector3d& region){
		Eigen::Vector3d minRegion, maxRegion;

		minRegion = this->position_ - 0.5 * region;
		maxRegion = this->position_ + 0.5 * region;
		bool valid = false;
		Eigen::Vector3d p;
		while (valid == false){	
			p(0) = globalPlanner::randomNumber(minRegion(0), maxRegion(0));
			p(1) = globalPlanner::randomNumber(minRegion(1), maxRegion(1));
			p(2) = globalPlanner::randomNumber(minRegion(2), maxRegion(2));
			valid = not this->map_->isInflatedOccupied(p) and not this->map_->isUnknown(p);
		}

		std::shared_ptr<PRM::Node> newNode (new PRM::Node(p));
		return newNode;
	}
}