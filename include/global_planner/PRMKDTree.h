/*
*	File: PRMKDTree.h
*	---------------
*   PRM KDTree header
*/

#ifndef PRMKDTREE_H
#define PRMKDTREE_H
#include <iostream>
#include <memory>
#include <Eigen/Eigen>
#include <limits>
#include <unordered_set>
#include <queue>


using std::cout; using std::endl;
namespace PRM{
	struct Node{
		Eigen::Vector3d pos;
		double numVoxels;
		std::map<double, int> yawNumVoxels;
		std::shared_ptr<Node> left = NULL;
		std::shared_ptr<Node> right = NULL;
		std::shared_ptr<Node> treeParent = NULL;
		std::shared_ptr<Node> parent = NULL;
		double g = std::numeric_limits<double>::infinity();
		double f = std::numeric_limits<double>::infinity();
		
		bool newNode = false;
		bool update = false;
		std::unordered_set<std::shared_ptr<Node>> adjNodes;

		Node (const Eigen::Vector3d& p){
			this->pos = p;
		}
	};

	struct CompareNode{
		bool operator()(std::shared_ptr<Node> n1, std::shared_ptr<Node> n2){
			return n1->f > n2->f;
		}
	};

	struct GainCompareNode{
		bool operator()(std::shared_ptr<Node> n1, std::shared_ptr<Node> n2){
			return n1->numVoxels < n2->numVoxels;
		}
	};


	class KDTree{
	private:
		int size_;
		std::shared_ptr<Node> root_;
		double leastDistNN_ = std::numeric_limits<double>::infinity(); // temporarily save minimum distance for nearest neighbor search
		std::vector<std::shared_ptr<Node>> notTarget_;
		std::priority_queue<std::shared_ptr<Node>, std::vector<std::shared_ptr<Node>>, GainCompareNode> goalNodes;
		std::vector<std::shared_ptr<Node>> record;
		int totalNumUnknown;
		int maxUnknown;
	public:
		KDTree();
		void clear();
		std::shared_ptr<Node> getRoot();
		int getSize();
		void insert(std::shared_ptr<Node> n);
		std::shared_ptr<Node> nearestNeighbor(std::shared_ptr<Node> n, 
			                                  std::shared_ptr<Node> rootNode=NULL,
			                                  std::shared_ptr<Node> bestNode=NULL,
			                                  int depth=0);
		std::vector<std::shared_ptr<Node>> kNearestNeighbor(std::shared_ptr<Node> n, int num);
		void addRecord(std::shared_ptr<Node>);
		void addGoalPQ(std::shared_ptr<Node>);
		std::priority_queue<std::shared_ptr<Node>, std::vector<std::shared_ptr<Node>>, GainCompareNode> getGoalNodes();
		std::vector<std::shared_ptr<Node>>& getRecord();
		void setTotalUnknown(int totalNumUnknown);
		int getTotalUnknown();
		void setMaxUnknown(int maxUnknown);
		int getMaxUnknown();
	};
}

#endif