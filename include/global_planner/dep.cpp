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
		this->registerPub();
		this->registerCallback();
	}

	void DEP::setMap(const std::shared_ptr<mapManager::dynamicMap>& map){
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
			cout << this->hint_ << ": Local Region Size: " << this->localRegionSize_[0] <<" " <<this->localRegionSize_[1]<<" "<< this->localRegionSize_[2]<< endl;
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
			cout << this->hint_ << ": Global Region Size: " << this->globalRegionSize_[0] <<" "<< this->globalRegionSize_[1]<<" "<< this->globalRegionSize_[2]<< endl;
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
			cout << this->hint_ << ": No distance thresh param. Use default: 0.8" << endl;
		}
		else{
			cout << this->hint_ << ": Distance Thresh: " << this->distThresh_ << endl;
		}

		//Camera Parameters	
		if (not this->nh_.getParam(this->ns_ + "/horizontal_FOV", this->horizontalFOV_)){
			this->horizontalFOV_ = 0.8;
			cout << this->hint_ << ": No Horizontal FOV param. Use default: 0.8" << endl;
		}
		else{
			cout << this->hint_ << ": Horizontal FOV: " << this->horizontalFOV_ << endl;
		}
		if (not this->nh_.getParam(this->ns_ + "/vertical_FOV", this->verticalFOV_)){
			this->verticalFOV_ = 0.8;
			cout << this->hint_ << ": No Vertical FOV param. Use default: 0.8" << endl;
		}
		else{
			cout << this->hint_ << ": Vertical FOV: " << this->verticalFOV_ << endl;
		}
		if (not this->nh_.getParam(this->ns_ + "/dmin", this->dmin_)){
			this->dmin_ = 0.0;
			cout << this->hint_ << ": No min depth param. Use default: 0.0" << endl;
		}
		else{
			cout << this->hint_ << ": Min Depth: " << this->dmin_ << endl;
		}
		if (not this->nh_.getParam(this->ns_ + "/dmax", this->dmax_)){
			this->dmax_ = 1.0;
			cout << this->hint_ << ": No max depth param. Use default: 1.0" << endl;
		}
		else{
			cout << this->hint_ << ": Max Depth: " << this->dmax_ << endl;
		}
	}

	void DEP::initModules(){
		// initialize roadmap
		this->roadmap_.reset(new PRM::KDTree ());

		// Yaw candicates array;
		for (int i=0; i<32; ++i){
			this->yaws.push_back(i*2*PI_const/32);
		}
		for (double yaw: this->yaws){
			this->yawNumVoxels[yaw] = 0;
		}
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

	bool DEP::makePlan(){
		if (not this->odomReceived_) return false;
		this->buildRoadMap();
		return true;
		//this->getBestViewCandidates();

		//this->findBestPath();
	}

	bool DEP::sensorRangeCondition(shared_ptr<PRM::Node> n1, shared_ptr<PRM::Node> n2){
		Eigen::Vector3d direction = n2->pos - n1->pos;
		Eigen::Vector3d projection;
		projection(0) = direction.x();
		projection(1) = direction.y();
		projection(2) = 0;
		double verticalAngle = angleBetweenVectors(direction, projection);
		if (verticalAngle < this->verticalFOV_/2){
			return true;
		}
		else{
			return false;
		}
	}

	// TODO create sensor check for 
	// vert, horz FOV, collision, and sensor distance range
	// for yaw angles in vector3d:  cos(yaw), sin(yaw), 0
	// horz angle between yaw angle vector and direction (x y 0) vector for horz FOV
	// Vert angle yaw angle vector and yaw angle vector (c s z) z is direction.z()
	bool DEP::sensorFOVCondition(Eigen::Vector3d n){
		Eigen::Vector3d direction = n - this->position_;
		Eigen::Vector3d horizontalProjection;
		horizontalProjection(0) = direction.x();
		horizontalProjection(1) = direction.y();
		horizontalProjection(2) = 0;
		Eigen::Vector3d yawAngleVector;
		yawAngleVector(0) = cos(this->currYaw_);
		yawAngleVector(1) = sin(this->currYaw_);
		yawAngleVector(2) = 0;
		double horizontalAngle = angleBetweenVectors(yawAngleVector, horizontalProjection);
		if (horizontalAngle > this->horizontalFOV_/2){
			return false;
		}
		Eigen::Vector3d verticalProjection;
		verticalProjection(0) = cos(this->currYaw_);
		verticalProjection(1) = sin(this->currYaw_);
		verticalProjection(2) = direction.z();
		double verticalAngle = angleBetweenVectors(yawAngleVector, verticalProjection);
		if (verticalAngle > this->verticalFOV_/2){
			return false;
		}
		double distance = direction.norm();
		if (distance > this->dmax_){
			return false;
		}
		bool hasCollision = this->map_->isInflatedOccupiedLine(n, this->position_);
		if (hasCollision == true){
			return false;
		}
		return true;
	}

	// map_->getRes
	// loop to create vector of horizon points
	// loop through horizon points for unknown points

	double DEP::calculateUnknown(shared_ptr<PRM::Node> n){
		// Position:
		Eigen::Vector3d p = n->pos;

		int countTotalUnknown = 0;
		for (double z = p(2) - this->dmax_; z < p(2)+ this->dmax_; z =+ this->map_->getRes()){
			for (double y = p(1)- this->dmax_; y < p(1)+ this->dmax_; y =+ this->map_->getRes()){
				for (double x = p(0)- this->dmax_; x < p(0)+ this->dmax_; x =+ this->map_->getRes()){
					Eigen::Vector3d nodePoint;
					nodePoint(0) = x;
					nodePoint(1) = y;
					nodePoint(2) = z;
					Eigen::Vector3d direction = nodePoint - p;
					Eigen::Vector3d face;
					face(0) = direction.x();
					face(1) = direction.y();
					face(2) = 0;
					if (sensorFOVCondition(nodePoint)){
						if (this->map_->isUnknown(nodePoint)){
							countTotalUnknown++;
							for (double yaw: this->yaws){
								Eigen::Vector3d yawDirection;
								yawDirection(0) = cos(yaw);
								yawDirection(1) = sin(yaw);
								yawDirection(2) = 0;
								double angleToYaw = angleBetweenVectors(face, yawDirection);
								if (angleToYaw <= this->horizontalFOV_/2){
									// Give credits to some good unknown
									this->yawNumVoxels[yaw] += 1;
								}
							}
						}
					}
				}
			}
		}
		cout << "+----------------------------+" << endl;
		cout << "Total Unknown: "<< countTotalUnknown << endl;
		cout << "+----------------------------+" << endl;
		n->yawNumVoxels = this->yawNumVoxels;
		return countTotalUnknown;
	}

	bool isNodeRequireUpdate(std::shared_ptr<PRM::Node> n, std::vector<std::shared_ptr<PRM::Node>> path, double& leastDistance){
		double distanceThresh = 2;
		leastDistance = 1000000;
		for (std::shared_ptr<PRM::Node> waypoint: path){
			double currentDistance = (n->pos - waypoint->pos).norm();
			if (currentDistance < leastDistance){
				leastDistance = currentDistance;
			}
		}
		if (leastDistance <= distanceThresh){
			return true;
		}
		else{
			return false;	
		}
		
	}
	void DEP::buildRoadMap(){
		std::vector<std::shared_ptr<PRM::Node>> path;
		//cout << "here" << endl;
		bool saturate = false;
		bool regionSaturate = false;
		int countSample = 0;
		std::shared_ptr<PRM::Node> n;
		while (ros::ok() and not saturate){
			//cout<<"sample"<<endl;
			if (regionSaturate){
				int countFailureGlobal = 0;
				// Generate new node
				while (ros::ok()){
					//cout << "global failure count: " << countFailureGlobal << endl;
					if (countFailureGlobal > this->sampleThresh_){
						saturate = true;
						break;
					}
					n = this->randomConfigBBox(this->globalRegionSize_);
					// Check how close new node is other nodes
					double distToNN;
					if (this->roadmap_->getSize() != 0){
						//cout << "find nearest neighbor" << endl;
						shared_ptr<PRM::Node> nn = this->roadmap_->nearestNeighbor(n);
						//cout << "find nearest neighbor end" << endl;
						distToNN = (n->pos - nn->pos).norm();
					}
					else{
						distToNN = this->distThresh_;
					}
					if (distToNN < this->distThresh_){
						++countFailureGlobal;
					}
					else{
						this->roadmap_->insert(n);
						this->prmNodeVec_.push_back(n);
						++countSample;
						// break;
					}
					ros::spinOnce();
				}
			}
			else{
				if (true){
					int countFailureLocal = 0;
					// Generate new node
					//cout << "local sample" << endl;
					while (ros::ok() and true){
						//cout << "failure number: " << countFailureLocal << endl;
						if (countFailureLocal > this->sampleThresh_){
							regionSaturate = true;
							break;
						}
						n = this->randomConfigBBox(this->localRegionSize_);
						//cout << "get new node" << endl;
						// Check how close new node is other nodes
						double distToNN;

						if (this->roadmap_->getSize() != 0){
							//cout << "find nearest neighbor" << endl;
							shared_ptr<PRM::Node> nn = this->roadmap_->nearestNeighbor(n);
							//cout << "find nearest neighbor end" << endl;
							distToNN = (n->pos - nn->pos).norm();
						}
						else{
							distToNN = this->distThresh_;
						}
						//cout << "distance to nn" << distToNN << endl;
						if (distToNN < this->distThresh_){
							++countFailureLocal;
						}
						else{
							this->roadmap_->insert(n);
							this->prmNodeVec_.push_back(n);
							++countSample;
							//cout << "new sample added" << endl;
							// break;
						}
						ros::spinOnce();
					}
				}
			}
			ros::spinOnce();
		}
		cout << "newly added: " << countSample << " samples" << endl;
		

		// node connection
		for (std::shared_ptr<PRM::Node> n: this->prmNodeVec_){
			std::vector<std::shared_ptr<PRM::Node>> knn = this->roadmap_->kNearestNeighbor(n, 15);
			
			for (std::shared_ptr<PRM::Node> nearestNeighborNode: knn){ // Check collision last if all other conditions are satisfied
				
				double distance2knn = (n->pos - nearestNeighborNode->pos).norm();
				bool rangeCondition = sensorRangeCondition(n, nearestNeighborNode) and sensorRangeCondition(nearestNeighborNode, n);
				
				if (distance2knn < 1.5 and rangeCondition == true){
					bool hasCollision = map_->isInflatedOccupiedLine(n->pos, nearestNeighborNode->pos);
					if (hasCollision == false){
						n->adjNodes.insert(nearestNeighborNode);
						nearestNeighborNode->adjNodes.insert(n);
					} 
				}
			}

			if (n->adjNodes.size() != 0){
				this->roadmap_->addRecord(n);
				double numVoxels = calculateUnknown(n);
				n->numVoxels = numVoxels;
			}
		}
		cout << "finished node connection" <<endl;

		
		// information gain update
		int countUpdateNode = 0;
		int countActualUpdate = 0;
		int totalUnknown = 0;
		int maxUnknown = 0;
		for (std::shared_ptr<PRM::Node> n: this->roadmap_->getRecord()){
			n->g = 1000;
			n->f = 1000;
			n->parent = NULL;
			// Check whether the nodes need to update:
			double leastDistance;
			bool update = isNodeRequireUpdate(n, path, leastDistance);
			if (update and n->newNode==false){
				n->update = true;
				++countUpdateNode;
				// check the update condition: 
				// 1. if node is very close to trajetory: set it to zero
				// 2. if it is already 0, leave it
				// 3. if it is less than threshold (e.g 100), set it to zero
				// 4. if non of those applies, recalculate it
				double cutOffValue = 5;
				double cutOffDistance = 0.5;
				if (n->numVoxels <= cutOffValue or leastDistance <= cutOffDistance){
					n->numVoxels = 0;
					for (double yaw:yaws){
						n->yawNumVoxels[yaw] = 0;
					}
				}
				else{
					++countActualUpdate;
					double numVoxels = calculateUnknown(n);
					n->numVoxels = numVoxels;
				}
			}
			else{
				n->update = false;
			}
			n->newNode = false;

			if (n->numVoxels>maxUnknown){
				maxUnknown = n->numVoxels;
			}
			this->roadmap_->addGoalPQ(n);
			totalUnknown += n->numVoxels;
		}
		this->roadmap_->setTotalUnknown(totalUnknown);
		this->roadmap_->setMaxUnknown(maxUnknown);
		cout << "Total Number of Unknown Voxels is: " << this->roadmap_->getTotalUnknown() << endl;
		cout << "Max Unknown Voxels is: " << this->roadmap_->getMaxUnknown() << endl;
		cout << "Number of nodes needed updated is: " << countUpdateNode << endl;
		cout << "Number of actual nodes needed updated is: " << countActualUpdate << endl;

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
			ros::spinOnce();
		}

		std::shared_ptr<PRM::Node> newNode (new PRM::Node(p));
		return newNode;
	}
}