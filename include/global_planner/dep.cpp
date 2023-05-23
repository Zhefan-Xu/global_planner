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

		// nearest neighbor number
		if (not this->nh_.getParam(this->ns_ + "/nearest_neighbor_number", this->nnNum_)){
			this->nnNum_ = 15;
			cout << this->hint_ << ": No nearest neighbor param. Use default: 15" << endl;
		}
		else{
			cout << this->hint_ << ": Nearest neighbor number is set to: " << this->nnNum_ << endl;
		}

		// node connection max distances
		if (not this->nh_.getParam(this->ns_ + "/max_connect_dist", this->maxConnectDist_)){
			this->maxConnectDist_ = 1.5;
			cout << this->hint_ << ": No max conect distance param. Use default: 1.5m." << endl;
		}
		else{
			cout << this->hint_ << ": Max connect distance is set to: " << this->maxConnectDist_ << endl;
		}

		// number of yaw angles
		int yawNum = 32;
		if (not this->nh_.getParam(this->ns_ + "/num_yaw_angles", yawNum)){
			for (int i=0; i<32; ++i){
				this->yaws_.push_back(i*2*PI_const/32);
			}					
			cout << this->hint_ << ": No number of yaw angles param. Use default: 32." << endl;
		}
		else{
			for (int i=0; i<yawNum; ++i){
				this->yaws_.push_back(i*2*PI_const/32);
			}	
			cout << this->hint_ << ": Number of yaw angles is set to: " << yawNum << endl;
		}

		// minimum threshold of voxels
		if (not this->nh_.getParam(this->ns_ + "/min_voxel_thresh", this->minVoxelThresh_)){
			this->minVoxelThresh_ = 0.1;
			cout << this->hint_ << ": No minimum threshold of voxels param. Use default: 0.1." << endl;
		}
		else{
			cout << this->hint_ << ": Minimum threshold of voxels is set to: " << this->minVoxelThresh_ << endl;
		}

		// minimum number of goal candidates
		if (not this->nh_.getParam(this->ns_ + "/min_goal_candidates", this->minCandidateNum_)){
			this->minCandidateNum_ = 10;
			cout << this->hint_ << ": No minimum number of goal candidates param. Use default: 10." << endl;
		}
		else{
			cout << this->hint_ << ": Minimum number of goal candidates is set to: " << this->minCandidateNum_ << endl;
		}

		// maximum number of goal candidates
		if (not this->nh_.getParam(this->ns_ + "/max_goal_candidates", this->maxCandidateNum_)){
			this->maxCandidateNum_ = 30;
			cout << this->hint_ << ": No maximum number of goal candidates param. Use default: 30." << endl;
		}
		else{
			cout << this->hint_ << ": Maximum number of goal candidates is set to: " << this->maxCandidateNum_ << endl;
		}
	}

	void DEP::initModules(){
		// initialize roadmap
		this->roadmap_.reset(new PRM::KDTree ());
	}

	void DEP::registerPub(){
		// roadmap visualization publisher
		this->roadmapPub_ = this->nh_.advertise<visualization_msgs::MarkerArray>("/dep/roadmap", 10);

		// candidate paths publisher
		this->candidatePathPub_ = this->nh_.advertise<visualization_msgs::MarkerArray>("/dep/candidate_paths", 10);

		// best path publisher
		this->bestPathPub_ = this->nh_.advertise<visualization_msgs::MarkerArray>("/dep/best_paths", 10);
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

		this->updateInformationGain();
		
		this->getBestViewCandidates(this->goalCandidates_);

		cout << "finish best view candidate" << endl;

		this->findCandidatePath(this->goalCandidates_);

		cout << "finish candidate path" << endl;

		this->findBestPath(this->candidatePaths_);

		cout << "found best path" << endl;

		return true;
	}

	bool DEP::sensorRangeCondition(const shared_ptr<PRM::Node>& n1, const shared_ptr<PRM::Node>& n2){
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
	bool DEP::sensorFOVCondition(const Eigen::Vector3d& sample, const Eigen::Vector3d& pos){
		Eigen::Vector3d direction = sample - pos;
		double distance = direction.norm();
		if (distance > this->dmax_){
			return false;
		}
		bool hasCollision = this->map_->isInflatedOccupiedLine(sample, pos);
		if (hasCollision == true){
			return false;
		}
		return true;
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
		bool saturate = false;
		bool regionSaturate = false;
		int countSample = 0;
		std::shared_ptr<PRM::Node> n;
		std::vector<std::shared_ptr<PRM::Node>> newNodes;
		while (ros::ok() and not saturate){
			if (regionSaturate){
				int countFailureGlobal = 0;
				// Generate new node
				while (ros::ok()){
					if (countFailureGlobal > this->sampleThresh_){
						saturate = true;
						break;
					}
					n = this->randomConfigBBox(this->globalRegionSize_);
					// Check how close new node is other nodes
					double distToNN;
					if (this->roadmap_->getSize() != 0){
						shared_ptr<PRM::Node> nn = this->roadmap_->nearestNeighbor(n);
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
						newNodes.push_back(n);
						this->prmNodeVec_.push_back(n);
						++countSample;
					}
				}
			}
			else{
				if (true){
					int countFailureLocal = 0;
					// Generate new node
					while (ros::ok() and true){
						//cout << "failure number: " << countFailureLocal << endl;
						if (countFailureLocal > this->sampleThresh_){
							regionSaturate = true;
							break;
						}
						n = this->randomConfigBBox(this->localRegionSize_);
						// Check how close new node is other nodes
						double distToNN;

						if (this->roadmap_->getSize() != 0){
							std::shared_ptr<PRM::Node> nn = this->roadmap_->nearestNeighbor(n);
							distToNN = (n->pos - nn->pos).norm();
						}
						else{
							distToNN = this->distThresh_;
						}
						if (distToNN < this->distThresh_){
							++countFailureLocal;
						}
						else{
							this->roadmap_->insert(n);
							newNodes.push_back(n);
							this->prmNodeVec_.push_back(n);
							++countSample;
						}
					}
				}
			}
		}
		cout << "newly added: " << countSample << " samples" << endl;
		
		// node connection
		for (std::shared_ptr<PRM::Node> n : newNodes){
			std::vector<std::shared_ptr<PRM::Node>> knn = this->roadmap_->kNearestNeighbor(n, this->nnNum_);
			for (std::shared_ptr<PRM::Node> nearestNeighborNode: knn){ // Check collision last if all other conditions are satisfied
				double distance2knn = (n->pos - nearestNeighborNode->pos).norm();
				bool rangeCondition = sensorRangeCondition(n, nearestNeighborNode) and sensorRangeCondition(nearestNeighborNode, n);
				
				if (distance2knn < this->maxConnectDist_ and rangeCondition == true){
					bool hasCollision = not this->map_->isInflatedFreeLine(n->pos, nearestNeighborNode->pos);
					if (hasCollision == false){
						n->adjNodes.insert(nearestNeighborNode);
						nearestNeighborNode->adjNodes.insert(n);
					}
				}
			}
			n->newNode = true;
		}
		cout << "finished node connection" <<endl;
	}	 

	void DEP::updateInformationGain(){
		// iterate through all current nodes (ignore update by path now)
		// two types of nodes need update:
		// 1. new nodes
		// 2. nodes close to the historical trajectory
		for (std::shared_ptr<PRM::Node> n : this->prmNodeVec_){
			if (n->newNode == true){// 1. new nodes
				std::unordered_map<double, int> yawNumVoxels;
				int unknownVoxelNum = this->calculateUnknown(n, yawNumVoxels);
				n->numVoxels = unknownVoxelNum;
				n->yawNumVoxels = yawNumVoxels;
				n->newNode = false;
			}
			else{// 2. trajectory nodes (TODO)

			}	
		}
	}

	void DEP::getBestViewCandidates(std::vector<std::shared_ptr<PRM::Node>>& goalCandidates){
		goalCandidates.clear();
		bool firstNode = true;
		std::priority_queue<std::shared_ptr<PRM::Node>, std::vector<std::shared_ptr<PRM::Node>>, PRM::GainCompareNode> gainPQ;

		// iterate through all points in the roadmap
		for (std::shared_ptr<PRM::Node> n : this->prmNodeVec_){
			gainPQ.push(n);
		}

		// select candidates from the priority queue
		int maxNumVoxel = 0;
		while (ros::ok()){
			std::shared_ptr<PRM::Node> n = gainPQ.top();
			
			if (firstNode){
				maxNumVoxel = n->numVoxels;
				firstNode = false;
			}

			if (double(n->numVoxels) < double(maxNumVoxel) * this->minVoxelThresh_){
				break;
			}
			goalCandidates.push_back(n);
			gainPQ.pop();
			
			if (int(goalCandidates.size()) >= this->maxCandidateNum_){
				break;
			}
		}

		while (int(goalCandidates.size()) < this->minCandidateNum_){
			if (gainPQ.size() == 0){
				break;
			}

			if (int(goalCandidates.size()) >= this->maxCandidateNum_){
				break;
			}

			std::shared_ptr<PRM::Node> n = gainPQ.top();
			gainPQ.pop();			
			goalCandidates.push_back(n);
		}
		cout << "goal candidate size is: " << goalCandidates.size() << endl;
	}

	void DEP::findCandidatePath(const std::vector<std::shared_ptr<PRM::Node>>& goalCandidates){
		// find nearest node of current location
		std::shared_ptr<PRM::Node> currPos;
		currPos.reset(new PRM::Node (this->position_));
		cout << "start position is: " << this->position_.transpose() << endl;
		std::shared_ptr<PRM::Node> start = this->roadmap_->nearestNeighbor(currPos);
		cout << "find the nearest neighbor: " << start->pos.transpose() << endl;

		std::vector<std::vector<std::shared_ptr<PRM::Node>>> candidatePaths;
		for (std::shared_ptr<PRM::Node> goal : goalCandidates){
			cout << "start A*" << endl;
			std::vector<std::shared_ptr<PRM::Node>> path = PRM::AStar(this->roadmap_, start, goal);
			path.insert(path.begin(), currPos);
			cout << "end A*" << endl;
			candidatePaths.push_back(path);
		}

		this->candidatePaths_ = candidatePaths;
	}

	void DEP::findBestPath(const std::vector<std::vector<std::shared_ptr<PRM::Node>>>& candidatePaths){
		// find path highest unknown
		std::vector<std::shared_ptr<PRM::Node>> bestPath;
		double highestScore = 0;
		for (std::vector<std::shared_ptr<PRM::Node>> path : candidatePaths){
			int unknownVoxel = 0;
			for (int i=0; i <= path.size()-1; i++){
				unknownVoxel += path[i]->numVoxels;
			}
			//cout << "voxel count done" << endl;
			double distance = calculatePathLength(path);
			//cout << "distance calc" <<endl;
			if (unknownVoxel/distance > highestScore){
				highestScore = unknownVoxel/distance;
				bestPath = path;
			}
		}
		this->bestPath_ = bestPath;
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

		if (this->candidatePaths_.size() != 0){
			this->publishCandidatePaths();
		}

		if (this->bestPath_.size()!=0){
			this->publishBestPath();
		}
	}

	void DEP::publishRoadmap(){
		visualization_msgs::MarkerArray roadmapMarkers;

		// PRM nodes and edges
		int countPointNum = 0;
		int countEdgeNum = 0;
		int countVoxelNumText = 0;
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
			point.scale.x = 0.1;
			point.scale.y = 0.1;
			point.scale.z = 0.1;
			point.color.a = 1.0;
			point.color.r = 1.0;
			point.color.g = 0.0;
			point.color.b = 0.0;
			++countPointNum;
			roadmapMarkers.markers.push_back(point);

			// number of voxels for each node
			visualization_msgs::Marker voxelNumText;
			voxelNumText.ns = "num_voxel_text";
			voxelNumText.header.frame_id = "map";
			voxelNumText.id = countVoxelNumText;
			voxelNumText.header.stamp = ros::Time::now();
			voxelNumText.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
			voxelNumText.action = visualization_msgs::Marker::ADD;
			voxelNumText.pose.position.x = n->pos(0);
			voxelNumText.pose.position.y = n->pos(1);
			voxelNumText.pose.position.z = n->pos(2)+0.1;
			voxelNumText.scale.x = 0.1;
			voxelNumText.scale.y = 0.1;
			voxelNumText.scale.z = 0.1;
			voxelNumText.color.a = 1.0;
			voxelNumText.text = std::to_string(n->numVoxels);
			voxelNumText.lifetime = ros::Duration(0.1);
			++countVoxelNumText;
			roadmapMarkers.markers.push_back(voxelNumText);


			// Edges
			visualization_msgs::Marker line;
			line.ns = "edge";
			line.header.frame_id = "map";
			line.type = visualization_msgs::Marker::LINE_LIST;
			line.header.stamp = ros::Time::now();
			for (std::shared_ptr<PRM::Node> adjNode : n->adjNodes){
				geometry_msgs::Point p1, p2;
				p1.x = n->pos(0);
				p1.y = n->pos(1);
				p1.z = n->pos(2);
				p2.x = adjNode->pos(0);
				p2.y = adjNode->pos(1);
				p2.z = adjNode->pos(2);				
				line.points.push_back(p1);
				line.points.push_back(p2);
				line.id = countEdgeNum;
				line.scale.x = 0.05;
				line.scale.y = 0.05;
				line.scale.z = 0.05;
				line.color.r = 0.0;
				line.color.g = 1.0;
				line.color.b = 0.0;
				line.color.a = 1.0;
				line.lifetime = ros::Duration(0.1);
				++countEdgeNum;
				roadmapMarkers.markers.push_back(line);
			}
		}

		int countGoalCandidateNum = 0;
		for (size_t i=0; i<this->goalCandidates_.size(); ++i){
			std::shared_ptr<PRM::Node> n = this->goalCandidates_[i];

			// Goal candidates
			visualization_msgs::Marker goalCandidatePoint;
			goalCandidatePoint.ns = "goal_candidate";
			goalCandidatePoint.header.frame_id = "map";
			goalCandidatePoint.header.stamp = ros::Time::now();
			goalCandidatePoint.id = countGoalCandidateNum;
			goalCandidatePoint.type = visualization_msgs::Marker::SPHERE;
			goalCandidatePoint.action = visualization_msgs::Marker::ADD;
			goalCandidatePoint.pose.position.x = n->pos(0);
			goalCandidatePoint.pose.position.y = n->pos(1);
			goalCandidatePoint.pose.position.z = n->pos(2);
			goalCandidatePoint.lifetime = ros::Duration(0.1);
			goalCandidatePoint.scale.x = 0.2;
			goalCandidatePoint.scale.y = 0.2;
			goalCandidatePoint.scale.z = 0.2;
			goalCandidatePoint.color.a = 1.0;
			goalCandidatePoint.color.r = 1.0;
			goalCandidatePoint.color.g = 0.0;
			goalCandidatePoint.color.b = 1.0;
			++countGoalCandidateNum;
			roadmapMarkers.markers.push_back(goalCandidatePoint);
		}

		this->roadmapPub_.publish(roadmapMarkers);
	}

	void DEP::publishCandidatePaths(){
		visualization_msgs::MarkerArray candidatePathMarkers;
		int countNodeNum = 0;
		int countLineNum = 0;
		for (std::vector<std::shared_ptr<PRM::Node>> path : this->candidatePaths_){
			for (size_t i=0; i<path.size(); ++i){
				std::shared_ptr<PRM::Node> n = path[i];
				visualization_msgs::Marker point;
				point.header.frame_id = "map";
				point.header.stamp = ros::Time::now();
				point.ns = "candidate_path_node";
				point.id = countNodeNum;
				point.type = visualization_msgs::Marker::SPHERE;
				point.action = visualization_msgs::Marker::ADD;
				point.pose.position.x = n->pos(0);
				point.pose.position.y = n->pos(1);
				point.pose.position.z = n->pos(2);
				point.lifetime = ros::Duration(0.1);
				point.scale.x = 0.15;
				point.scale.y = 0.15;
				point.scale.z = 0.15;
				point.color.a = 1.0;
				point.color.r = 1.0;
				point.color.g = 1.0;
				point.color.b = 0.0;
				++countNodeNum;
				candidatePathMarkers.markers.push_back(point);

				if (i<path.size()-1){
					std::shared_ptr<PRM::Node> nNext = path[i+1];
					visualization_msgs::Marker line;
					line.ns = "candidate_path";
					line.header.frame_id = "map";
					line.type = visualization_msgs::Marker::LINE_LIST;
					line.header.stamp = ros::Time::now();
					geometry_msgs::Point p1, p2;
					p1.x = n->pos(0);
					p1.y = n->pos(1);
					p1.z = n->pos(2);
					p2.x = nNext->pos(0);
					p2.y = nNext->pos(1);
					p2.z = nNext->pos(2);				
					line.points.push_back(p1);
					line.points.push_back(p2);
					line.id = countLineNum;
					line.scale.x = 0.1;
					line.scale.y = 0.1;
					line.scale.z = 0.1;
					line.color.r = 0.0;
					line.color.g = 0.0;
					line.color.b = 0.0;
					line.color.a = 1.0;
					line.lifetime = ros::Duration(0.1);
					++countLineNum;
					candidatePathMarkers.markers.push_back(line);				
				}
			}
		}
		this->candidatePathPub_.publish(candidatePathMarkers);		
	}
	
	void DEP::publishBestPath(){
		visualization_msgs::MarkerArray bestPathMarkers;
		int countNodeNum = 0;
		int countLineNum = 0;
		for (size_t i=0; i<this->bestPath_.size(); ++i){
			std::shared_ptr<PRM::Node> n = this->bestPath_[i];
			visualization_msgs::Marker point;
			point.header.frame_id = "map";
			point.header.stamp = ros::Time::now();
			point.ns = "best_path_node";
			point.id = countNodeNum;
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
			point.color.g = 1.0;
			point.color.b = 1.0;
			++countNodeNum;
			bestPathMarkers.markers.push_back(point);

			if (i<this->bestPath_.size()-1){
				std::shared_ptr<PRM::Node> nNext = this->bestPath_[i+1];
				visualization_msgs::Marker line;
				line.ns = "best_path";
				line.header.frame_id = "map";
				line.type = visualization_msgs::Marker::LINE_LIST;
				line.header.stamp = ros::Time::now();
				geometry_msgs::Point p1, p2;
				p1.x = n->pos(0);
				p1.y = n->pos(1);
				p1.z = n->pos(2);
				p2.x = nNext->pos(0);
				p2.y = nNext->pos(1);
				p2.z = nNext->pos(2);				
				line.points.push_back(p1);
				line.points.push_back(p2);
				line.id = countLineNum;
				line.scale.x = 0.2;
				line.scale.y = 0.2;
				line.scale.z = 0.2;
				line.color.r = 1.0;
				line.color.g = 0.0;
				line.color.b = 0.0;
				line.color.a = 1.0;
				line.lifetime = ros::Duration(0.1);
				++countLineNum;
				bestPathMarkers.markers.push_back(line);				
			}
		}
	this->bestPathPub_.publish(bestPathMarkers);		
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
			valid = this->map_->isInflatedFree(p);
		}

		std::shared_ptr<PRM::Node> newNode (new PRM::Node(p));
		return newNode;
	}

	int DEP::calculateUnknown(const shared_ptr<PRM::Node>& n, std::unordered_map<double, int>& yawNumVoxels){
		for (double yaw : this->yaws_){
			yawNumVoxels[yaw] = 0;
		}

		// Position:
		Eigen::Vector3d p = n->pos;

		double zRange = this->dmax_ * tan(this->verticalFOV_/2.0);
		int countTotalUnknown = 0;
		for (double z = p(2) - zRange; z < p(2) + zRange; z += this->map_->getRes()){
			for (double y = p(1) - this->dmax_; y < p(1)+ this->dmax_; y += this->map_->getRes()){
				for (double x = p(0) - this->dmax_; x < p(0) + this->dmax_; x += this->map_->getRes()){
					Eigen::Vector3d nodePoint (x, y, z);
					if (this->map_->isUnknown(nodePoint)){
						if (this->sensorFOVCondition(nodePoint, p)){
							++countTotalUnknown;
							for (double yaw: this->yaws_){
								Eigen::Vector3d yawDirection (cos(yaw), sin(yaw), 0);
								Eigen::Vector3d direction = nodePoint - p;
								Eigen::Vector3d face (direction(0), direction(1), 0);
								double angleToYaw = angleBetweenVectors(face, yawDirection);
								if (angleToYaw <= this->horizontalFOV_/2){
									yawNumVoxels[yaw] += 1;
								}
							}
						}
					}
				}
			}
		}
		return countTotalUnknown;
	}
	double DEP::calculatePathLength(const std::vector<shared_ptr<PRM::Node>> path){
		int idx1 = 0;
		double length = 0;
		for (int idx2=1; idx2<=path.size()-1; ++idx2){
			length += (path[idx2]->pos - path[idx1]->pos).norm();
			++idx1;
		}
		return length;
	}
}