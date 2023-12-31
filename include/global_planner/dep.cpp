/*
*	File: dep.cpp
*	---------------
*   dynamic exploration planner implementation
*/

#include <global_planner/dep.h>
#include <random>


namespace globalPlanner{
	DEP::DEP(const ros::NodeHandle& nh) : nh_(nh){
		this->ns_ = "/DEP";
		this->hint_ = "[DEP]";
		this->initParam();
		this->initModules();
		this->registerPub();
		this->registerCallback();
	}

	void DEP::setMap(const std::shared_ptr<mapManager::occMap>& map){
		this->map_ = map;
	}

	void DEP::loadVelocity(double vel, double angularVel){
		this->vel_ = vel;
		this->angularVel_ = angularVel;
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

		// local sample region min
		std::vector<double> localRegionMinTemp;	
		if (not this->nh_.getParam(this->ns_ + "/local_region_min", localRegionMinTemp)){
			this->localRegionMin_(0) = -5.0;
			this->localRegionMin_(1) = -5.0;
			this->localRegionMin_(2) = -2.0;
			cout << this->hint_ << ": No local region min param. Use default: [-5 -5 -2]" <<endl;
		}
		else{
			this->localRegionMin_(0) = localRegionMinTemp[0];
			this->localRegionMin_(1) = localRegionMinTemp[1];
			this->localRegionMin_(2) = localRegionMinTemp[2];
			cout << this->hint_ << ": Local Region Min: " << this->localRegionMin_[0] <<" " <<this->localRegionMin_[1]<<" "<< this->localRegionMin_[2]<< endl;
		}

		// local sample region max
		std::vector<double> localRegionMaxTemp;	
		if (not this->nh_.getParam(this->ns_ + "/local_region_max", localRegionMaxTemp)){
			this->localRegionMax_(0) = 5.0;
			this->localRegionMax_(1) = 5.0;
			this->localRegionMax_(2) = 2.0;
			cout << this->hint_ << ": No local region max param. Use default: [5 5 2]" <<endl;
		}
		else{
			this->localRegionMax_(0) = localRegionMaxTemp[0];
			this->localRegionMax_(1) = localRegionMaxTemp[1];
			this->localRegionMax_(2) = localRegionMaxTemp[2];
			cout << this->hint_ << ": Local Region Max: " << this->localRegionMax_[0] <<" " <<this->localRegionMax_[1]<<" "<< this->localRegionMax_[2]<< endl;
		}

		// global sample region min
		std::vector<double> globalRegionMinTemp;	
		if (not this->nh_.getParam(this->ns_ + "/global_region_min", globalRegionMinTemp)){
			this->globalRegionMin_(0) = -20.0;
			this->globalRegionMin_(1) = -20.0;
			this->globalRegionMin_(2) = 0.0;
			cout << this->hint_ << ": No global region min param. Use default: [-20 -20 0]" <<endl;
		}
		else{
			this->globalRegionMin_(0) = globalRegionMinTemp[0];
			this->globalRegionMin_(1) = globalRegionMinTemp[1];
			this->globalRegionMin_(2) = globalRegionMinTemp[2];
			cout << this->hint_ << ": Global Region Min: " << this->globalRegionMin_[0] <<" "<< this->globalRegionMin_[1]<<" "<< this->globalRegionMin_[2]<< endl;
		}

		// global sample region max
		std::vector<double> globalRegionMaxTemp;	
		if (not this->nh_.getParam(this->ns_ + "/global_region_max", globalRegionMaxTemp)){
			this->globalRegionMax_(0) = 20.0;
			this->globalRegionMax_(1) = 20.0;
			this->globalRegionMax_(2) = 3.0;
			cout << this->hint_ << ": No global region max param. Use default: [20 20 3]" <<endl;
		}
		else{
			this->globalRegionMax_(0) = globalRegionMaxTemp[0];
			this->globalRegionMax_(1) = globalRegionMaxTemp[1];
			this->globalRegionMax_(2) = globalRegionMaxTemp[2];
			cout << this->hint_ << ": Global Region Max: " << this->globalRegionMax_[0] <<" "<< this->globalRegionMax_[1]<<" "<< this->globalRegionMax_[2]<< endl;
		}

		// Local Sample Threshold Value
		if (not this->nh_.getParam(this->ns_ + "/local_sample_thresh", this->localSampleThresh_)){
			this->localSampleThresh_ = 50;
			cout << this->hint_ << ": No local sample thresh param. Use default: 50" << endl;
		}
		else{
			cout << this->hint_ << ": Local sample Thresh: " << this->localSampleThresh_ << endl;
		}
		
		// Global Sample Threshold Value
		if (not this->nh_.getParam(this->ns_ + "/global_sample_thresh", this->globalSampleThresh_)){
			this->globalSampleThresh_ = 50;
			cout << this->hint_ << ": No global sample thresh param. Use default: 50" << endl;
		}
		else{
			cout << this->hint_ << ": Global sample Thresh: " << this->globalSampleThresh_ << endl;
		}

		// Frontier Sample Threshold Value
		if (not this->nh_.getParam(this->ns_ + "/frontier_sample_thresh", this->frontierSampleThresh_)){
			this->frontierSampleThresh_ = 50;
			cout << this->hint_ << ": No frontier sample thresh param. Use default: 50" << endl;
		}
		else{
			cout << this->hint_ << ": Frontier sample Thresh: " << this->frontierSampleThresh_ << endl;
		}		

		// minimum distance for node sampling
		if (not this->nh_.getParam(this->ns_ + "/dist_thresh", this->distThresh_)){
			this->distThresh_ = 0.8;
			cout << this->hint_ << ": No distance thresh param. Use default: 0.8" << endl;
		}
		else{
			cout << this->hint_ << ": Distance Thresh: " << this->distThresh_ << endl;
		}

		// safety distance for random sampling in xy
		if (not this->nh_.getParam(this->ns_ + "/safe_distance_xy", this->safeDistXY_)){
			this->safeDistXY_ = 0.3;
			cout << this->hint_ << ": No safe distance in XY param. Use default: 0.3" << endl;
		}
		else{
			cout << this->hint_ << ": Safe distance in XY: " << this->safeDistXY_ << endl;
		}

		// safety distance for random sampling in z
		if (not this->nh_.getParam(this->ns_ + "/safe_distance_z", this->safeDistZ_)){
			this->safeDistZ_ = 0.0;
			cout << this->hint_ << ": No safe distance in Z param. Use default: 0.0" << endl;
		}
		else{
			cout << this->hint_ << ": Safe distance in Z: " << this->safeDistZ_ << endl;
		}

		// safety distance check unknown
		if (not this->nh_.getParam(this->ns_ + "/safe_distance_check_unknown", this->safeDistCheckUnknown_)){
			this->safeDistCheckUnknown_ = true;
			cout << this->hint_ << ": No safe distance check unknown param. Use default: true" << endl;
		}
		else{
			cout << this->hint_ << ": Safe distance check unknown: " << this->safeDistCheckUnknown_ << endl;
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

		// frontier nearest neighbor number
		if (not this->nh_.getParam(this->ns_ + "/frontier_nearest_neighbor_number", this->nnNumFrontier_)){
			this->nnNumFrontier_ = 15;
			cout << this->hint_ << ": No frontier nearest neighbor param. Use default: 15" << endl;
		}
		else{
			cout << this->hint_ << ": Frontier nearest neighbor number is set to: " << this->nnNumFrontier_ << endl;
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

		// Information gain update  distance
		if (not this->nh_.getParam(this->ns_ + "/information_gain_update_distance", this->updateDist_)){
			this->updateDist_ = 1.0;
			cout << this->hint_ << ": No information gain update distance param. Use default: 1.0m." << endl;
		}
		else{
			cout << this->hint_ << ": Information gain update distance is set to: " << this->updateDist_ << endl;
		}

		// yaw penalty weight
		if (not this->nh_.getParam(this->ns_ + "/yaw_penalty_weight", this->yawPenaltyWeight_)){
			this->yawPenaltyWeight_ = 1.0;
			cout << this->hint_ << ": No yaw penalty weight param. Use default: 1.0." << endl;
		}
		else{
			cout << this->hint_ << ": Yaw penalty weight is set to: " << this->yawPenaltyWeight_ << endl;
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

		// fronteir vis publisher
		this->frontierVisPub_ = this->nh_.advertise<visualization_msgs::MarkerArray>("/dep/frontier_regions", 10);
	}

	void DEP::registerCallback(){
		// odom subscriber
		this->odomSub_ = this->nh_.subscribe(this->odomTopic_, 1000, &DEP::odomCB, this);
	
		// visualization timer
		this->visTimer_ = this->nh_.createTimer(ros::Duration(0.1), &DEP::visCB, this);

	}

	bool DEP::makePlan(){
		if (not this->odomReceived_) return false;
		// cout << "start detecting frontier" << endl;
		// ros::Time frontierStartTime = ros::Time::now();
		this->detectFrontierRegion(this->frontierPointPairs_);
		// ros::Time frontierEndTime = ros::Time::now();
		// cout << "frontier detection time: " << (frontierEndTime - frontierStartTime).toSec() << endl;


		// cout << "start building roadmap" << endl;
		// ros::Time buildStartTime = ros::Time::now();
		this->buildRoadMap();
		// ros::Time buildEndTime = ros::Time::now();
		// cout << "build roadmap time: " << (buildEndTime - buildStartTime).toSec() << endl;

		// cout << "start pruning nodes" << endl;
		// ros::Time updateStartTime = ros::Time::now();
		this->pruneNodes();

		// cout << "start update information gain" << endl;
		this->updateInformationGain();
		// ros::Time updateEndTime = ros::Time::now();
		// cout << "update time: " << (updateEndTime - updateStartTime).toSec() << endl;

		// cout << "start get goal candidates" << endl;
		// ros::Time pathStartTime = ros::Time::now();
		this->getBestViewCandidates(this->goalCandidates_);

		// cout << "finish best view candidate with size: " << this->goalCandidates_.size() << endl;

		bool findCandidatePathSuccess = this->findCandidatePath(this->goalCandidates_, this->candidatePaths_);

		// cout << "finish candidate path with size: " << this->candidatePaths_.size() << endl;
		if (not findCandidatePathSuccess){
			// cout << "Find candidate paths fails. need generate more samples." << endl;
			return false;
		}

		this->findBestPath(this->candidatePaths_, this->bestPath_);
		// ros::Time pathEndTime = ros::Time::now();
		// cout << "path time: " << (pathEndTime - pathStartTime).toSec() << endl;
		// cout << "found best path with size: " << this->bestPath_.size() << endl;
		return true;
	}

	nav_msgs::Path DEP::getBestPath(){
		nav_msgs::Path bestPath;
		for (int i=0; i<int(this->bestPath_.size()); ++i){
			std::shared_ptr<PRM::Node> currNode = this->bestPath_[i];
			geometry_msgs::PoseStamped p;
			p.pose.position.x = currNode->pos(0);
			p.pose.position.y = currNode->pos(1);
			p.pose.position.z = currNode->pos(2);
			if (i < int(this->bestPath_.size())-1){
				std::shared_ptr<PRM::Node> nextNode = this->bestPath_[i+1];
				Eigen::Vector3d diff = nextNode->pos - currNode->pos;
				double angle = atan2(diff(1), diff(0));
				p.pose.orientation = globalPlanner::quaternion_from_rpy(0, 0, angle);
			}
			bestPath.poses.push_back(p);
		}
		
		// get the best yaw for the last pose
		double bestYaw = this->bestPath_.back()->getBestYaw();
		bestPath.poses.back().pose.orientation = globalPlanner::quaternion_from_rpy(0, 0, bestYaw);

		return bestPath;
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

	// create sensor check for 
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
		leastDistance = std::numeric_limits<double>::max();
		for (std::shared_ptr<PRM::Node>& waypoint: path){
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

	void DEP::detectFrontierRegion(std::vector<std::pair<Eigen::Vector3d, double>>& frontierPointPairs){
		frontierPointPairs.clear();

		Eigen::Vector3d mapMin, mapMax;
		this->map_->getCurrMapRange(mapMin, mapMax);
		int numRow = (mapMax(1) - mapMin(1))/this->map_->getRes() + 1;
		int numCol = (mapMax(0) - mapMin(0))/this->map_->getRes() + 1;

		cv::SimpleBlobDetector::Params params;
		// params.filterByColor = true;
		// params.blobColor = 255;
		// params.filterByConvexity = true;
  //   	params.minConvexity = 0.1;;

		params.filterByColor = true;
		params.blobColor = 255;  // Blobs should be white
		params.filterByArea = true;
		params.minArea = pow(1/this->map_->getRes(), 2);
		params.maxArea = numRow * numCol;
		params.filterByCircularity = false;
		params.minCircularity = 1;
		params.filterByConvexity = true;
		params.minConvexity = 0.1;

		cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);
		std::vector<cv::Mat> imgVec;
		// find height levels to slice the map
		double heightRes = 0.3;
		int col = 0;
		int row = 0;
		for (double h=this->globalRegionMin_(2); h<=this->globalRegionMax_(2); h+=heightRes){
			row = 0;
			cv::Mat im (numRow, numCol, CV_8UC1);
			for (double y=mapMin(1); y<=mapMax(1) and row < numRow; y+=this->map_->getRes()){
				col = 0;
				for (double x=mapMin(0); x<=mapMax(0) and col < numCol; x+=this->map_->getRes()){
					Eigen::Vector3d p (x, y, h);
					if (this->map_->isInflatedOccupied(p)){
						im.at<uchar>(row, col) = 0;
					}
					else if (this->map_->isInflatedFree(p)){
						// im.at<uchar>(row, col) = 255/2;
						// im.at<uchar>(row, col) = 255;
						im.at<uchar>(row, col) = 0;
					}
					else{
						// im.at<uchar>(row, col) = 255/2;
						im.at<uchar>(row, col) = 255;
					}
					++col;
				}
				++row;
			}
			imgVec.push_back(im);
		}



		// detect each image and find the corresponding 3D positions
		double height = this->globalRegionMin_(2);
		for (cv::Mat img : imgVec){
			std::vector<cv::KeyPoint> keypoints;

			cv::Rect rect(0, 0, numRow, numCol);
			cv::rectangle(img, rect, cv::Scalar(0, 0, 0), 3);
			detector->detect(img, keypoints);

			// convert keypoints back to the map coordinate
			for (cv::KeyPoint keypoint : keypoints){
				Eigen::Vector3d p (mapMin(0) + keypoint.pt.x * this->map_->getRes(), mapMin(1) + keypoint.pt.y * this->map_->getRes(), height);
				double dist = keypoint.size * this->map_->getRes();
				frontierPointPairs.push_back({p, dist});
			}

			height += heightRes;

			// cv::Mat imgWithKeypoints;
			// cv::drawKeypoints(img, keypoints, imgWithKeypoints,  cv::Scalar(0, 0, 255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

			// cv::imshow("Blobs with Keypoints", imgWithKeypoints);
			// cv::imwrite("/home/zhefan/Desktop/temp/test.jpg", img);
			// cv::waitKey(0);
		} 
	}

	void DEP::buildRoadMap(){
		bool saturate = false;
		bool regionSaturate = false;
		int countSample = 0;
		std::shared_ptr<PRM::Node> n;
		std::vector<std::shared_ptr<PRM::Node>> newNodes;

		// while does reach sampling threshold (fail time) 
		// 1. sample point from frontier region by weighted sampling
		// 2. find several nearest node
		// 3. for each node, extend random [mindist, maxdist] distance. if success, add new sample 
		std::vector<double> sampleWeights;
		for (int i=0; i<int(this->frontierPointPairs_.size()); ++i){
			double size = this->frontierPointPairs_[i].second;
			sampleWeights.push_back(pow(size, 2));
		}
		int countFrontierFailure = 0;
		while (ros::ok() and countFrontierFailure < this->frontierSampleThresh_ and sampleWeights.size() != 0){
			std::shared_ptr<PRM::Node> fn = this->sampleFrontierPoint(sampleWeights);
			// a. find N nearest neighbors
			std::vector<std::shared_ptr<PRM::Node>> fnNeighbors = this->roadmap_->kNearestNeighbor(fn, this->nnNumFrontier_);

			// b. for each neighbor extend them and check the validity
			if (int(fnNeighbors.size()) > 0){
				int countSampleOnce = 0;
				for (std::shared_ptr<PRM::Node> fnNN : fnNeighbors){
					n = this->extendNode(fnNN, fn);
					if (this->isPosValid(n->pos, this->safeDistXY_, this->safeDistZ_)){
						std::shared_ptr<PRM::Node> nn = this->roadmap_->nearestNeighbor(n);
						double distToNN = (n->pos - nn->pos).norm();
						if (distToNN >= this->distThresh_){
							this->roadmap_->insert(n);
							newNodes.push_back(n);
							// this->prmNodeVec_.push_back(n);
							this->prmNodeVec_.insert(n);
							++countSample;	
							++countSampleOnce;
						}
					}
				}
				if (countSampleOnce == 0){
					++countFrontierFailure;
				}

			}
			else{ // not enough neighbor
				break;
			}
		}
		// cout << "added sample from frontier:  " << countSample << endl;

		while (ros::ok() and not saturate){
			if (regionSaturate){
				int countFailureGlobal = 0;
				// Generate new node
				while (ros::ok()){
					if (countFailureGlobal > this->globalSampleThresh_){
						saturate = true;
						break;
					}
					n = this->randomConfigBBox(this->globalRegionMin_, this->globalRegionMax_);
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
						// this->prmNodeVec_.push_back(n);
						this->prmNodeVec_.insert(n);
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
						if (countFailureLocal > this->localSampleThresh_){
							regionSaturate = true;
							break;
						}
						Eigen::Vector3d localSampleMin = this->position_+this->localRegionMin_;
						Eigen::Vector3d localSampleMax = this->position_+this->localRegionMax_;
						n = this->randomConfigBBox(localSampleMin, localSampleMax);
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
							// this->prmNodeVec_.push_back(n);
							this->prmNodeVec_.insert(n);
							++countSample;
						}
					}
				}
			}
		}


		
		// node connection
		for (std::shared_ptr<PRM::Node>& n : newNodes){
			std::vector<std::shared_ptr<PRM::Node>> knn = this->roadmap_->kNearestNeighbor(n, this->nnNum_);
			for (std::shared_ptr<PRM::Node>& nearestNeighborNode : knn){ // Check collision last if all other conditions are satisfied
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
	}	 

	void DEP::pruneNodes(){
		// record the invalid nodes
		std::unordered_set<std::shared_ptr<PRM::Node>> invalidSet;
		for (std::shared_ptr<PRM::Node> n : this->prmNodeVec_){ // new nodes
			if (not this->isPosValid(n->pos, this->safeDistXY_, this->safeDistZ_)){// 1. new nodes
			// if (this->map_->isInflatedOccupied(n->pos)){// 1. new nodes
				invalidSet.insert(n);
			}	
		}

		// remove invalid nodes
		for (std::shared_ptr<PRM::Node> in : invalidSet){
			this->prmNodeVec_.erase(in);
			this->roadmap_->remove(in);
		}


		//  remove invalid edges
		for (std::shared_ptr<PRM::Node> n : this->prmNodeVec_){
			std::vector<std::shared_ptr<PRM::Node>> eraseVec;
			for (std::shared_ptr<PRM::Node> neighbor : n->adjNodes){
				if (invalidSet.find(neighbor) != invalidSet.end()){
					eraseVec.push_back(neighbor);
				}
			}

			for (std::shared_ptr<PRM::Node> en : eraseVec){
				n->adjNodes.erase(en);
			}
		}
	}

	void DEP::updateInformationGain(){
		// iterate through all current nodes (ignore update by path now)
		// two types of nodes need update:
		// 1. new nodes
		// 2. nodes close to the historical trajectory
		std::unordered_set<std::shared_ptr<PRM::Node>> updateSet;
		for (std::shared_ptr<PRM::Node> n : this->prmNodeVec_){ // new nodes
			if (n->newNode == true){// 1. new nodes
				updateSet.insert(n);
			}	
		}

		for (Eigen::Vector3d& histPos : this->histTraj_){ // traj update nodes
			std::shared_ptr<PRM::Node> histN;
			histN.reset(new PRM::Node(histPos));
			std::vector<std::shared_ptr<PRM::Node>> nns = this->roadmap_->kNearestNeighbor(histN, 10);
			for (std::shared_ptr<PRM::Node>& nn : nns){
				if ((nn->pos - histN->pos).norm() <= this->updateDist_){
					updateSet.insert(nn);
				}
			}
		}

		for (std::shared_ptr<PRM::Node> updateN : updateSet){ // update information gain
			std::unordered_map<double, int> yawNumVoxels;
			int unknownVoxelNum = this->calculateUnknown(updateN, yawNumVoxels);
			updateN->numVoxels = unknownVoxelNum;
			updateN->yawNumVoxels = yawNumVoxels;
			updateN->newNode = false;
		}
		this->histTraj_.clear(); // clear history
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
			if (gainPQ.size() == 0){
				break;
			}


			std::shared_ptr<PRM::Node> n = gainPQ.top();
			
			if (firstNode){
				// if ((n->pos - this->position_).norm() >= 1.0){
				if ((n->pos - this->position_).norm() >= 0.0){
					maxNumVoxel = n->numVoxels;
					firstNode = false;
				}
			}

			if (double(n->numVoxels) < double(maxNumVoxel) * this->minVoxelThresh_){
				break;
			}
			// if ((n->pos - this->position_).norm() >= 1.0){
			if ((n->pos - this->position_).norm() >= 0.0){			
				if (this->isPosValid(n->pos, this->safeDistXY_, this->safeDistZ_)){
					goalCandidates.push_back(n);
					// cout << "Valid goal candidate: " << n->pos.transpose() << " voxel: " << n->numVoxels  << endl;
				}
			}
			gainPQ.pop();
			
			if (int(goalCandidates.size()) >= this->maxCandidateNum_){
				break;
			}
		}

		// cout << "current pos: " << this->position_.transpose() << endl;
		while (int(goalCandidates.size()) < this->minCandidateNum_){
			if (gainPQ.size() == 0){
				break;
			}

			if (int(goalCandidates.size()) >= this->maxCandidateNum_){
				break;
			}

			std::shared_ptr<PRM::Node> n = gainPQ.top();
			gainPQ.pop();
			// if ((n->pos - this->position_).norm() >= 1.0){ 
			if ((n->pos - this->position_).norm() >= 0.0){	
				// cout << "candidate goal: " << n->pos.transpose() << endl;	
				if (this->isPosValid(n->pos, this->safeDistXY_, this->safeDistZ_)){
					goalCandidates.push_back(n);
					// cout << "Valid goal candidate: " << n->pos.transpose() << " voxel: " << n->numVoxels  << endl;
				}			
			}
		}
	}

	bool DEP::findCandidatePath(const std::vector<std::shared_ptr<PRM::Node>>& goalCandidates, std::vector<std::vector<std::shared_ptr<PRM::Node>>>& candidatePaths){
		bool findPath = false;
		// find nearest node of current location
		std::shared_ptr<PRM::Node> currPos;
		currPos.reset(new PRM::Node (this->position_));
		std::shared_ptr<PRM::Node> start = this->roadmap_->nearestNeighbor(currPos);

		candidatePaths.clear();
		for (std::shared_ptr<PRM::Node> goal : goalCandidates){
			std::vector<std::shared_ptr<PRM::Node>> path = PRM::AStar(this->roadmap_, start, goal, this->map_);
			if (int(path.size()) != 0){
				findPath = true;
			}
			else{
				continue;
			}
			path.insert(path.begin(), currPos);
			std::vector<std::shared_ptr<PRM::Node>> pathSc;
			this->shortcutPath(path, pathSc);
			candidatePaths.push_back(pathSc);
		}
		return findPath;
	}

	void DEP::findBestPath(const std::vector<std::vector<std::shared_ptr<PRM::Node>>>& candidatePaths, std::vector<std::shared_ptr<PRM::Node>>& bestPath){
		// find path highest unknown
		bestPath.clear();
		double highestScore = -1;
		for (int n=0; n<int(candidatePaths.size()); ++n){
			std::vector<std::shared_ptr<PRM::Node>> path = candidatePaths[n]; 
			if (int(path.size()) == 0) continue;
			double yawDist = 0;
			double prevYaw = this->currYaw_;
			int unknownVoxel = 0;
			for (int i=0; i<int(path.size())-1; ++i){
				std::shared_ptr<PRM::Node> currNode = path[i];
				std::shared_ptr<PRM::Node> nextNode = path[i+1];
				Eigen::Vector3d diff = nextNode->pos - currNode->pos;
				double angle = atan2(diff(1), diff(0));

				// reevaluate the unknowns for intermediate points
				std::unordered_map<double, int> yawNumVoxels;
				int unknownVoxelNum = this->calculateUnknown(currNode, yawNumVoxels);
				currNode->numVoxels = unknownVoxelNum;
				currNode->yawNumVoxels = yawNumVoxels;


				unknownVoxel += currNode->getUnknownVoxels(angle);
				yawDist += globalPlanner::angleDiff(prevYaw, angle);
				prevYaw = angle;
			}
			// reevaluate the goal node
			std::unordered_map<double, int> yawNumVoxels;
			int unknownVoxelNum = this->calculateUnknown(path.back(), yawNumVoxels);
			path.back()->numVoxels = unknownVoxelNum;
			path.back()->yawNumVoxels = yawNumVoxels;
			unknownVoxel += path.back()->getBestYawVoxel();
			yawDist += globalPlanner::angleDiff(prevYaw, path.back()->getBestYaw());

			double distance = this->calculatePathLength(path);
			// cout << "total is distance is: " << distance << " total yaw distance is: " << yawDist << " voxel: " << path.back()->numVoxels << endl;
			double pathTime = distance/this->vel_ + this->yawPenaltyWeight_ * yawDist/this->angularVel_;
			double score = double(unknownVoxel)/pathTime; 
			// cout << "unknown for path: " << n <<  " is: " << unknownVoxel << " score: " << score << " distance: " << distance << " Time: " << pathTime <<  " Last total unknown: " << path.back()->numVoxels << " last best: " << path.back()->getBestYawVoxel() << endl;
			if (score > highestScore){
				highestScore = score;
				bestPath = path;
			}
		}
		if (highestScore == 0){
			cout << "[DEP]: Current score is 0. The exploration might complete." << endl;
		}
	}


	void DEP::odomCB(const nav_msgs::OdometryConstPtr& odom){
		this->odom_ = *odom;
		this->position_ = Eigen::Vector3d (this->odom_.pose.pose.position.x, this->odom_.pose.pose.position.y, this->odom_.pose.pose.position.z);
		this->currYaw_ = globalPlanner::rpy_from_quaternion(this->odom_.pose.pose.orientation);
		this->odomReceived_ = true;

		if (this->histTraj_.size() == 0){
			this->histTraj_.push_back(this->position_);
		}
		else{
			Eigen::Vector3d lastPos = this->histTraj_.back();
			double dist = (this->position_ - lastPos).norm();
			if (dist >= 0.5){
				if (this->histTraj_.size() >= 100){
					this->histTraj_.pop_front();
					this->histTraj_.push_back(this->position_);
				}
				else{
					this->histTraj_.push_back(this->position_);
				}
			}
		}
	}

	void DEP::visCB(const ros::TimerEvent&){
		if (this->prmNodeVec_.size() != 0){
			this->publishRoadmap();
		}

		if (this->candidatePaths_.size() != 0){
			this->publishCandidatePaths();
		}

		if (this->bestPath_.size() != 0){
			this->publishBestPath();
		}

		if (this->frontierPointPairs_.size() != 0){
			this->publishFrontier();
		}
	}


	bool DEP::isPosValid(const Eigen::Vector3d& p){
		for (double x=p(0)-this->safeDistXY_; x<=p(0)+this->safeDistXY_; x+=this->map_->getRes()){
			for (double y=p(1)-this->safeDistXY_; y<=p(1)+this->safeDistXY_; y+=this->map_->getRes()){
				for (double z=p(2)-this->safeDistZ_; z<=p(2)+this->safeDistZ_; z+=this->map_->getRes()){
					if (this->safeDistCheckUnknown_){
						if (not this->map_->isInflatedFree(Eigen::Vector3d (x, y, z))){
							return false;
						}
					}
					else{
						if (this->map_->isInflatedOccupied(Eigen::Vector3d (x, y, z))){
							return false;
						}
					}
				}
			}
		}
		return true;			
	}


	bool DEP::isPosValid(const Eigen::Vector3d& p, double safeDistXY, double safeDistZ){
		for (double x=p(0)-safeDistXY; x<=p(0)+safeDistXY; x+=this->map_->getRes()){
			for (double y=p(1)-safeDistXY; y<=p(1)+safeDistXY; y+=this->map_->getRes()){
				for (double z=p(2)-safeDistZ; z<=p(2)+safeDistZ; z+=this->map_->getRes()){
					if (this->safeDistCheckUnknown_){
						if (not this->map_->isInflatedFree(Eigen::Vector3d (x, y, z))){
							return false;
						}
					}
					else{
						if (this->map_->isInflatedOccupied(Eigen::Vector3d (x, y, z))){
							return false;
						}
					}
				}
			}
		}
		return true;		
	}

	std::shared_ptr<PRM::Node> DEP::randomConfigBBox(const Eigen::Vector3d& minRegion, const Eigen::Vector3d& maxRegion){
		Eigen::Vector3d mapMinRegion, mapMaxRegion, minSampleRegion, maxSampleRegion;
		this->map_->getCurrMapRange(mapMinRegion, mapMaxRegion);
		// cout << "current map range is: " << mapMinRegion.transpose() << ", " << mapMaxRegion.transpose() << endl;
		minSampleRegion(0) = std::max(mapMinRegion(0), minRegion(0));
		minSampleRegion(1) = std::max(mapMinRegion(1), minRegion(1));
		minSampleRegion(2) = std::max(mapMinRegion(2), minRegion(2));
		maxSampleRegion(0) = std::min(mapMaxRegion(0), maxRegion(0));
		maxSampleRegion(1) = std::min(mapMaxRegion(1), maxRegion(1));
		maxSampleRegion(2) = std::min(mapMaxRegion(2), maxRegion(2));

		minSampleRegion(0) = std::max(minSampleRegion(0), this->globalRegionMin_(0));
		minSampleRegion(1) = std::max(minSampleRegion(1), this->globalRegionMin_(1));
		minSampleRegion(2) = std::max(minSampleRegion(2), this->globalRegionMin_(2));
		maxSampleRegion(0) = std::min(maxSampleRegion(0), this->globalRegionMax_(0));
		maxSampleRegion(1) = std::min(maxSampleRegion(1), this->globalRegionMax_(1));
		maxSampleRegion(2) = std::min(maxSampleRegion(2), this->globalRegionMax_(2));


		bool valid = false;
		Eigen::Vector3d p;
		while (valid == false){	
			p(0) = globalPlanner::randomNumber(minSampleRegion(0), maxSampleRegion(0));
			p(1) = globalPlanner::randomNumber(minSampleRegion(1), maxSampleRegion(1));
			p(2) = globalPlanner::randomNumber(minSampleRegion(2), maxSampleRegion(2));

			valid = this->isPosValid(p, this->safeDistXY_, this->safeDistZ_);

			// valid = this->map_->isInflatedFree(p);
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
		for (double z = p(2) - zRange; z <= p(2) + zRange; z += this->map_->getRes()){
			for (double y = p(1) - this->dmax_; y <= p(1)+ this->dmax_; y += this->map_->getRes()){
				for (double x = p(0) - this->dmax_; x <= p(0) + this->dmax_; x += this->map_->getRes()){
					Eigen::Vector3d nodePoint (x, y, z);
					if (nodePoint(0) < this->globalRegionMin_(0) or nodePoint(0) > this->globalRegionMax_(0) or
						nodePoint(1) < this->globalRegionMin_(1) or nodePoint(1) > this->globalRegionMax_(1) or
						nodePoint(2) < this->globalRegionMin_(2) or nodePoint(2) > this->globalRegionMax_(2)){
						// not in global range
						continue;
					}

					if (this->map_->isUnknown(nodePoint) and not this->map_->isInflatedOccupied(nodePoint)){
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
	double DEP::calculatePathLength(const std::vector<shared_ptr<PRM::Node>>& path){
		int idx1 = 0;
		double length = 0;
		for (size_t idx2=1; idx2<=path.size()-1; ++idx2){
			length += (path[idx2]->pos - path[idx1]->pos).norm();
			++idx1;
		}
		return length;
	}

	void DEP::shortcutPath(const std::vector<std::shared_ptr<PRM::Node>>& path, std::vector<std::shared_ptr<PRM::Node>>& pathSc){
		size_t ptr1 = 0; size_t ptr2 = 2;
		pathSc.push_back(path[ptr1]);

		if (path.size() == 1){
			return;
		}

		if (path.size() == 2){
			pathSc.push_back(path[1]);
			return;
		}

		while (ros::ok()){
			if (ptr2 > path.size()-1){
				break;
			}
			std::shared_ptr<PRM::Node> p1 = path[ptr1];
			std::shared_ptr<PRM::Node> p2 = path[ptr2];
			Eigen::Vector3d pos1 = p1->pos;
			Eigen::Vector3d pos2 = p2->pos;
			bool lineValidCheck;
			// lineValidCheck = not this->map_->isInflatedOccupiedLine(pos1, pos2);
			lineValidCheck = this->map_->isInflatedFreeLine(pos1, pos2);
			// double maxDistance = std::numeric_limits<double>::max();
			// double maxDistance = 3.0;
			// if (lineValidCheck and (pos1 - pos2).norm() <= maxDistance){
			if (lineValidCheck){
				if (ptr2 == path.size()-1){
					pathSc.push_back(p2);
					break;
				}
				++ptr2;
			}
			else{
				pathSc.push_back(path[ptr2-1]);
				if (ptr2 == path.size()-1){
					pathSc.push_back(p2);
					break;
				}
				ptr1 = ptr2-1;
				ptr2 = ptr1+2;
			}
		}		
	}

	int DEP::weightedSample(const std::vector<double>& weights){
		double total = std::accumulate(weights.begin(), weights.end(), 0.0);
		std::vector<double> normalizedWeights;

		 for (const double weight : weights){
		 	normalizedWeights.push_back(weight/total);
		 }

		std::random_device rd;
		std::mt19937 gen(rd());
		std::discrete_distribution<int> distribution(normalizedWeights.begin(), normalizedWeights.end());
		return distribution(gen);
	}


	std::shared_ptr<PRM::Node> DEP::sampleFrontierPoint(const std::vector<double>& sampleWeights){
		// choose the frontier region (random sample by frontier area) 
		int idx = weightedSample(sampleWeights);

		// sample a frontier point in the region
		Eigen::Vector3d frontierCenter = this->frontierPointPairs_[idx].first;
		double frontierSize = this->frontierPointPairs_[idx].second;
		double xmin = std::max(frontierCenter(0) - frontierSize/sqrt(2), this->globalRegionMin_(0));
		double xmax = std::min(frontierCenter(0) + frontierSize/sqrt(2), this->globalRegionMax_(0));
		double ymin = std::max(frontierCenter(1) - frontierSize/sqrt(2), this->globalRegionMin_(1));
		double ymax = std::min(frontierCenter(1) + frontierSize/sqrt(2), this->globalRegionMax_(1));
		double zmin = frontierCenter(2);
		double zmax = frontierCenter(2);
		Eigen::Vector3d frontierPoint;
		frontierPoint(0) = globalPlanner::randomNumber(xmin, xmax);
		frontierPoint(1) = globalPlanner::randomNumber(ymin, ymax);
		frontierPoint(2) = globalPlanner::randomNumber(zmin, zmax);
		std::shared_ptr<PRM::Node> frontierNode (new PRM::Node(frontierPoint));
		return frontierNode;
	}

	std::shared_ptr<PRM::Node> DEP::extendNode(const std::shared_ptr<PRM::Node>& n, const std::shared_ptr<PRM::Node>& target){
		double extendDist = randomNumber(this->distThresh_, this->maxConnectDist_);
		Eigen::Vector3d p = n->pos + (target->pos - n->pos)/(target->pos - n->pos).norm() * extendDist;
		p(0) = std::max(this->globalRegionMin_(0), std::min(p(0), this->globalRegionMax_(0)));
		p(1) = std::max(this->globalRegionMin_(1), std::min(p(1), this->globalRegionMax_(1)));
		p(2) = std::max(this->globalRegionMin_(2), std::min(p(2), this->globalRegionMax_(2)));
		std::shared_ptr<PRM::Node> extendedNode (new PRM::Node(p));
		return extendedNode;
	}

	void DEP::publishRoadmap(){
		visualization_msgs::MarkerArray roadmapMarkers;

		// PRM nodes and edges
		int countPointNum = 0;
		int countEdgeNum = 0;
		int countVoxelNumText = 0;
		for (std::shared_ptr<PRM::Node> n : this->prmNodeVec_){
			// std::shared_ptr<PRM::Node> n = this->prmNodeVec_[i];

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
			point.lifetime = ros::Duration(0.5);
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
			voxelNumText.lifetime = ros::Duration(0.5);
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
				line.lifetime = ros::Duration(0.5);
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
			goalCandidatePoint.lifetime = ros::Duration(0.5);
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
		for (std::vector<std::shared_ptr<PRM::Node>>& path : this->candidatePaths_){
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
					line.lifetime = ros::Duration(0.5);
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
			point.lifetime = ros::Duration(0.5);
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
				line.lifetime = ros::Duration(0.5);
				++countLineNum;
				bestPathMarkers.markers.push_back(line);				
			}
		}
		this->bestPathPub_.publish(bestPathMarkers);		
	}

	void DEP::publishFrontier(){
		visualization_msgs::MarkerArray frontierMarkers;
		int frontierRangeCount = 0;
		for (int i=0; i<int(this->frontierPointPairs_.size()); ++i){
			visualization_msgs::Marker range;

			Eigen::Vector3d p = this->frontierPointPairs_[i].first;
			double dist = this->frontierPointPairs_[i].second;

			range.header.frame_id = "map";
			range.header.stamp = ros::Time::now();
			range.ns = "frontier range";
			range.id = frontierRangeCount;
			range.type = visualization_msgs::Marker::SPHERE;
			range.action = visualization_msgs::Marker::ADD;
			range.pose.position.x = p(0);
			range.pose.position.y = p(1);
			range.pose.position.z = p(2);
			range.lifetime = ros::Duration(0.5);
			range.scale.x = dist;
			range.scale.y = dist;
			range.scale.z = 0.1;
			range.color.a = 0.4;
			range.color.r = 0.0;
			range.color.g = 0.0;
			range.color.b = 1.0;
			++frontierRangeCount;
			frontierMarkers.markers.push_back(range);			
		}
		this->frontierVisPub_.publish(frontierMarkers);
	}

}