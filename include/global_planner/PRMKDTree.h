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

using std::cout; using std::endl;
namespace PRM{
	struct Node{
		Eigen::Vector3d pos;
		std::shared_ptr<Node> left = NULL;
		std::shared_ptr<Node> right = NULL;
		std::shared_ptr<Node> treeParent = NULL;

		Node (const Eigen::Vector3d& p){
			this->pos = p;
		}

	};


	class KDTree{
	private:
		int size_;
		std::shared_ptr<Node> root_;


	public:
		KDTree();
		std::shared_ptr<Node> getRoot();
		int getSize();
		void insert(std::shared_ptr<Node> n);
	};
}

#endif