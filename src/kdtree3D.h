/* \author Aaron Brown */
// Quiz on implementing kd tree 
// Modify for KD-Tree 3D

#include "render/render.h"


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

	~Node()
	{
		delete left;
		delete right;
	}
};

struct KdTree
{
	Node* root;

	KdTree()
	: root(NULL)
	{}

	~KdTree()
	{
		delete root;
	}
	
	void insertHelper(Node** node, uint depth, std::vector<float> point, int id)
	{
		if (*node == NULL)
		{
			*node = new Node(point, id);
		}
		else
		{
			uint idx = depth % 3;
			if (point[idx] < ((*node)->point[idx]))
				insertHelper(&((*node)->left), depth + 1, point, id);
			else
				insertHelper(&((*node)->right), depth + 1, point, id);
		}

	}

	void insert(std::vector<float> point, int id)
	{
		// TODO: Fill in this function to insert a new point into the tree
		// the function should create a new node and place correctly with in the root 
		insertHelper(&root, 0, point, id);
	}

	bool distCompare(std::vector<float> point1, std::vector<float> point2, float distTol)
	{
		float x = point1[0] - point2[0];
		float y = point1[1] - point2[1];
		float z = point1[2] - point2[2];
		std::cout << x << std::endl;
		if (sqrt((x * x + y * y + z * z)) <= distTol)
		{	
			//std::cout << sqrt((x * x + y * y + z * z)) << std::endl;
			return true;
		}
		return false;
	}

	void searchHelper(std::vector<float> target, Node* node, int depth, float distTol, std::vector<int>& ids)
	{
		if (node != NULL)
		{	
			if((node->point[0] >= (target[0] - distTol) && node->point[0] <= (target[0] + distTol)) 
			&& (node->point[1] >= (target[1] - distTol) && node->point[1] <= (target[1] + distTol))
			&& (node->point[2] >= (target[2] - distTol) && node->point[2] <= (target[2] + distTol))) {
				if (distCompare(target, node->point, distTol))
				{
					ids.push_back(node->id);
				}
			}

			uint idx = depth % 3;
			if ((target[idx] - distTol) < node->point[idx])
				searchHelper(target, node->left, depth + 1, distTol, ids);

			// if target X + dist tol is still less than the point's X, then don't go down the right side of the tree
			if ((target[idx] + distTol) > node->point[idx])
				searchHelper(target, node->right, depth + 1, distTol, ids);
		}
	}

	// return a list of point ids in the tree that are within distance of target
	std::vector<int> search(std::vector<float> target, float distanceTol)
	{
		std::vector<int> ids;
		searchHelper(target, root, 0, distanceTol, ids);
		return ids;
	}
	

};




