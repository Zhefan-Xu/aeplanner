#include <aeplanner/rrt.h>
#include <algorithm>
#include <geometry_msgs/Point.h>


// HELPER FUNCTION: Extract Best Path
void extractBestBranch(KDTree* t, std::vector<Node*>& best_branch){
	// Get Best Node
	Node* best_node = (*t).getBest();
	Node* ptr = best_node;
	while (ptr != NULL){
		best_branch.push_back(ptr);
		ptr = ptr->parent;
	}
	std::reverse(best_branch.begin(), best_branch.end());
}

// HELPER FUNCTION: Extract Best Path
void extractBestBranchSc(KDTree* t, std::vector<Node*>& best_branch){
	// Get Best Node
	Node* best_node = (*t).getBestSc();
	Node* ptr = best_node;
	while (ptr != NULL){
		best_branch.push_back(ptr);
		ptr = ptr->parent;
	}
	std::reverse(best_branch.begin(), best_branch.end());
}

int evaluatePath(OcTree& tree, std::vector<Node*> &best_branch){
	int total_num_voxels = 0;
	for (Node* n: best_branch){
		double yaw = 0;
		total_num_voxels += calculateUnknown(tree, n, yaw);
		n->yaw = yaw;
	}
	return total_num_voxels;
}


std::vector<Node*> rrt(geometry_msgs::Point start,
					   geometry_msgs::Point goal,
					   double eps,
					   OcTree& tree){
	std::vector<Node*> path;
	point3d start_point (start.x, start.y, start.z);
	point3d goal_point (goal.x, goal.y, goal.z);
	Node* start_node = new Node(start_point); 
	Node* goal_node = new Node(goal_point);
	// cout << start_node->p << endl;
	// cout << goal_node->p << endl;
	KDTree* t = new KDTree();
	t->insert(start_node);
	auto start_time = high_resolution_clock::now(); 
	while (true){
		auto stop_time = high_resolution_clock::now();
		auto duration = duration_cast<microseconds>(stop_time - start_time); 
		if (duration.count()/1e6 > 1){
			cout << "No path!" << endl;
			return path;
		}
		// firstly random take sample
		double rand_num = randomNumber(0, 10);
		Node* q_rand = new Node ();
		if (rand_num > 1){
			q_rand = randomConfig(tree);
		}
		else{
			q_rand->p = goal_node->p;
		}
		// cout << q_rand->p << endl;
		// find its nearest neighbor
		Node* q_near = t->nearestNeighbor(q_rand);
		// adjust length based on the eps
		adjustLength(q_rand, q_near, eps);
		bool has_collision = checkCollision(tree, q_rand, q_near);
		if (not has_collision){
			t->insert(q_rand);
			q_rand->parent = q_near;
			q_rand->dis = q_near->dis + q_rand->p.distance(q_near->p);
			double distance_to_goal = q_rand->p.distance(goal_node->p);
			// cout << distance_to_goal << endl;
			// if (distance_to_goal < 2){
			// 	cout << distance_to_goal << endl;
			// }
			if (distance_to_goal < eps and not checkCollision(tree, q_rand, goal_node)){
			// if (distance_to_goal < eps){ //and not checkCollision(tree, q_rand, goal_node)){
				goal_node->parent = q_rand;
				goal_node->dis = q_rand->dis + distance_to_goal;
				break;
			}
		}

	}
	auto stop_time = high_resolution_clock::now();
	auto duration = duration_cast<microseconds>(stop_time - start_time); 
	// cout << "success time: " << duration.count()/1e6 << endl;
	// backtracking
	Node* ptr = goal_node;
	while (ptr != NULL){
		path.push_back(ptr);
		ptr = ptr->parent;
	}
	std::reverse(path.begin(), path.end());
	// path = shortCutPath(path, tree);
	return path;
}


// Planner Function:
std::vector<Node*> planner(OcTree& tree, 
						  Node* start,
						  int num_sample,
						  int max_sample,
						  double eps,
						  double& best_IG,
						  std::vector<geometry_msgs::Point> &tree_vis_array,
						  KDTree* t,
						  KDTree* cache, 
						  bool &frontier_exploration,
						  std::priority_queue<Node*, std::vector<Node*>, GainCompareNode> &cache_queue,
						  bool write_result = false){
	// Intialize 
	std::vector<Node*> best_branch;

	// Grow RRT Tree
	KDTree* new_t = growRRT(tree, start, num_sample, max_sample, eps, best_IG,  tree_vis_array, cache_queue, t, cache, write_result);
	// Extract the best branch
	if (new_t == NULL){
		return best_branch;
	}
	extractBestBranch(new_t, best_branch);
	// extractBestBranchSc(new_t, best_branch);


	double remove_thresh = 0.8;
	int num_unknown_reevaluate = 0;
	Node* high_gain_node;
	bool success = false;
	if (cache_queue.size() > 0){
		// Check validity
		
		
		while (not success){
			if (cache_queue.size() <= 0){
				break;
			}
			high_gain_node = cache_queue.top();
			double yaw = 0;
			num_unknown_reevaluate = calculateUnknown(tree, high_gain_node, yaw);

			if (num_unknown_reevaluate <= remove_thresh * high_gain_node->num_unknown){
				cache_queue.pop();
			}
			else{
				success = true;
				break;
			}
		}
		print_node(*high_gain_node);
		cout << "num_unknown_reevaluate: " << num_unknown_reevaluate << endl;
	}


	if (not success){
		return best_branch;
	}

	int total_num_voxels = evaluatePath(tree, best_branch);
	cout << "Total unknown voxels: " << total_num_voxels << endl;

	int way_points_num = best_branch.size();
	int factor = 5;
	if (num_unknown_reevaluate > factor*total_num_voxels/way_points_num){
		cout << "we should apply frontier exploration in this iteration!!! " << endl;
		geometry_msgs::Point start_point, goal_point;
		start_point.x = start->p.x(); 
		start_point.y = start->p.y();
		start_point.z = start->p.z();
		goal_point.x = high_gain_node->p.x();
		goal_point.y = high_gain_node->p.y();
		goal_point.z = high_gain_node->p.z();
		std::vector<Node*> branch = rrt(start_point, goal_point, eps, tree);
		cache_queue.pop();

		if (branch.size() != 0){
			best_branch = branch;

			// Set yaw for each nodes:
			for (Node* n: best_branch){
				double yaw = 0;
				int count = calculateUnknown(tree, n, yaw);
				n->yaw = yaw;
			}
			frontier_exploration = true;
		}
		else{
			cout << "But we cannot find a valid path!" << endl;
		}

			
	}



	return best_branch;
} 