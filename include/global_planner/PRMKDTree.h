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


using std::cout; using std::endl;
namespace PRM{
	struct Node{
		Eigen::Vector3d pos;
		double numVoxels;
		std::shared_ptr<Node> left = NULL;
		std::shared_ptr<Node> right = NULL;
		std::shared_ptr<Node> treeParent = NULL;
		
		bool newNode = false;
		std::unordered_set<std::shared_ptr<Node>> adjNodes;

		Node (const Eigen::Vector3d& p){
			this->pos = p;
		}

	};


	class KDTree{
	private:
		int size_;
		std::shared_ptr<Node> root_;
		double leastDistNN_ = std::numeric_limits<double>::infinity(); // temporarily save minimum distance for nearest neighbor search
		std::vector<std::shared_ptr<Node>> notTarget;
		std::vector<std::shared_ptr<Node>> record;

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
		std::vector<std::shared_ptr<Node>>& getRecord();
	};
}

#endif