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
		std::vector<double>local_size;	
		if (not this->nh_.getParam(this->ns_ + "/local_region_size", local_size)){
			this->localRegionSize_[0] = 1;
			this->localRegionSize_[1] = 1;
			this->localRegionSize_[2] = 1;
			cout << this->hint_ << ": No local region size param. Use default: [1 1 1]" <<endl;
		}
		else{
			this->localRegionSize_[0] = local_size[0];
			this->localRegionSize_[1] = local_size[1];
			this->localRegionSize_[2] = local_size[2];
			cout << this->hint_ << ": Local Region Size: " << this->localRegionSize_[0] << this->localRegionSize_[1]<< this->localRegionSize_[2]<< endl;
		}
	}

	void DEP::initModules(){
		// initialize roadmap
		this->roadmap_.reset(new PRM::KDTree ());
	}

	void DEP::registerCallback(){
		this->odomSub_ = this->nh_.subscribe(this->odomTopic_, 1000, &DEP::odomCB, this);
	}

	void DEP::makePlan(){
		this->buildRoadMap();

		this->getBestViewCandidates();

		this->findBestPath();
	}

	void DEP::buildRoadMap(){
		// local sample (TODO)
		//std::vector<PRM::Node*> new_nodes;
		// double threshold = 500; // HardCode threshold
		bool saturate = false;
		bool region_saturate = false;
		int count_sample = 0;
		int sample_thresh = 50;
		double distance_thresh = 0.8;
		while (not saturate){
			std::shared_ptr<PRM::Node> n;
			double distance_to_nn = 0;
			// int r = (int) randomNumber(1,10);
			// cout << r << endl;
			if (region_saturate){
				
				int count_failure = 0;
				// Generate new node
				while (true){
					if (count_failure > sample_thresh){
						saturate = true;
						break;
					}
					n = this->randomConfigBBox(this->localRegionSize_);
					// Check how close new node is other nodes
					shared_ptr<PRM::Node> nn = this->roadmap_->nearestNeighbor(n);
					distance_to_nn = (n->pos - nn->pos).norm();
					if (distance_to_nn < distance_thresh){
						++count_failure;
					}
					else{
						this->roadmap_->insert(n);
						++count_sample;
						break;
					}
				}
			}
		}
	}


	
	


		// global sample (TODO)


		// node connection


		// information gain update

	void DEP::getBestViewCandidates(){

	}

	void DEP::findBestPath(){

	}

	void DEP::odomCB(const nav_msgs::OdometryConstPtr& odom){
		this->odom_ = *odom;
		this->pos_ = Eigen::Vector3d (this->odom_.pose.pose.position.x, this->odom_.pose.pose.position.y, this->odom_.pose.pose.position.z);
	}
	//help fucntions
	std::shared_ptr<PRM::Node> DEP::randomConfigBBox(const Eigen::Vector3d& region){
		Eigen::Vector3d min_region, max_region;

		min_region = this->pos_ - 0.5*(region);
		max_region = this->pos_ + 0.5*(region);
		bool valid = false;
		Eigen::Vector3d p;

		while (valid == false){	
			p[0] = randomNumber(min_region[0], max_region[0]);
			p[1] = randomNumber(min_region[1], max_region[1]);
			p[2] = randomNumber(min_region[2], max_region[2]);
			valid = not this->map_->isInflatedOccupied(p) and not this->map_->isUnknown(p);
		}

		std::shared_ptr<PRM::Node> new_node (new PRM::Node(p));

		return new_node;
	}
	std::random_device rd;
	std::mt19937 mt(rd());	
	double DEP::randomNumber(double min, double max){
		std::uniform_real_distribution<double> distribution(min, max);
		return distribution(mt);
	}
}