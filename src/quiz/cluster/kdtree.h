/* \author Aaron Brown */
// Quiz on implementing kd tree

#include "../../render/render.h"
#include <cmath>


// Structure to represent node of kd tree
struct Node
{
	std::vector<float> point;
	int id;
	Node* left;
	Node* right;

	Node(std::vector<float> arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL)
	{}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	void insertNode(Node** node,uint depth,std::vector<float> point, int id){
	if (*node == NULL){
		*node = new Node(point,id);
	}
	else{
		int in = depth%2;
		if (point[in] < (*node)->point[in]){
			insertNode(&((*node)->left),depth+1,point,id);
		}
		else{
			insertNode(&((*node)->right),depth+1,point,id);
		}
	}
}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		insertNode(&root,0,point,id);

	}
	void searchNeighbor(Node* node, uint depth, std::vector<float> target, float distanceTol, std::vector<int> &ids){
		uint in = depth%2;
		if (node != NULL){
			if ((node->point[0] <= target[0]+distanceTol) && (node->point[0] >= target[0]-distanceTol) && (node->point[1] <= target[1]+distanceTol) && (node->point[1] >= target[1]-distanceTol)){
				if (sqrt(pow(target[0]-node->point[0],2) + pow(target[1]-node->point[1],2)) <= distanceTol){
					ids.push_back(node->id);
				}
			}
			if (target[in] - distanceTol < node->point[in]){
				searchNeighbor(node->left,depth+1,target,distanceTol,ids);
			}
			if (target[in] + distanceTol > node->point[in]){
				searchNeighbor(node->right,depth+1,target,distanceTol,ids);
			}
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		searchNeighbor(root,0,target,distanceTol,ids);
		return ids;
	}
	

};




