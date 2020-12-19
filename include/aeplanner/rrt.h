#include <aeplanner/kdtree.h>
#include <aeplanner/utils.h>
#include <random>
#include <chrono> 
#include <vector>
#include <cmath>
#include <geometry_msgs/Point.h>
#include <fstream>
#include <set>
using namespace std::chrono; 

// PARAMETER
double lambda = 0.5;

// cafe
double env_x_min = -4.5;
double env_x_max = 4.5;
double env_y_min = -11;
double env_y_max = 7;
double env_z_min = 0;
double env_z_max = 2.5;

// Define Drone Size:
double DRONE_X = 0.5;
double DRONE_Y = 0.5;
double DRONE_Z = 0.1;

// // Maze
// double env_x_min = -15;
// double env_x_max = 15;
// double env_y_min = -30;
// double env_y_max = 20;
// double env_z_min = 0.2;
// double env_z_max = 2.5;


// // Define Drone Size:
// double DRONE_X = 0.1;
// double DRONE_Y = 0.1;
// double DRONE_Z = 0.1;

// // Define Drone Size:
// double DRONE_X = 0.5;
// double DRONE_Y = 0.5;
// double DRONE_Z = 0.1;

// // Tunnel
// double env_x_min = -8;
// double env_x_max = 8;
// double env_y_min = 0;
// double env_y_max = 35;
// double env_z_min = 0;
// double env_z_max = 15;


// // Define Drone Size:
// double DRONE_X = 0.5;
// double DRONE_Y = 0.5;
// double DRONE_Z = 0.1;

// // Apartment
// double env_x_min = -15;
// double env_x_max = 15;
// double env_y_min = -15;
// double env_y_max = 20;
// double env_z_min = 0.2;
// double env_z_max = 2.5;


// // Define Drone Size:
// double DRONE_X = 0.2;
// double DRONE_Y = 0.2;
// double DRONE_Z = 0.1;

// // dynamic field
// double env_x_min = -12;
// double env_x_max = 12;
// double env_y_min = -12;
// double env_y_max = 12;
// double env_z_min = 0.2;
// double env_z_max = 2.5;


// // Define Drone Size:
// double DRONE_X = 0.1;
// double DRONE_Y = 0.1;
// double DRONE_Z = 0.1;

// // storage_room field
// double env_x_min = -100;
// double env_x_max = 100;
// double env_y_min = -100;
// double env_y_max = 100;
// double env_z_min = 0.2;
// double env_z_max = 10;


// // Define Drone Size:
// double DRONE_X = 0.1;
// double DRONE_Y = 0.1;
// double DRONE_Z = 0.1;

// MAP RESOLUTION:
double RES = 0.2;
// double RES = 0.05;

// Depth CAMERA
double FOV = 1.8;
double dmin = 0.00001;
double dmax = 1.0;
// double dmax = 3.0;
double ratio = (double) 640/480;    // Width/Height
double W = dmax * tan(FOV/2) * 2;
double H = W/ratio;
double horizontal_max_angle = FOV/2;
double vertical_max_angle = atan2(1 * tan(FOV/2)/ratio, 1);


int step_num = 0;

// Helper Function Write Node info Declare
void writeNodeInfo(Node* n, std::ofstream &file);

// Random Generator
std::random_device rd;
std::mt19937 mt(rd());
// Helper Function: Random Number
double randomNumber(double min, double max){
	std::uniform_real_distribution<double> distribution(min, max);
	return distribution(mt);
}

// Hepler Function: Check Position Validation
bool isValid(const OcTree& tree, point3d p, bool robot_size=false){
	if (robot_size == false){
		OcTreeNode* nptr = tree.search(p);
		if (nptr == NULL){return false;}
		return !tree.isNodeOccupied(nptr);
	}
	else{// we should consider the robot size in this case
		// calculate x, y, z range
		double x_min, x_max, y_min, y_max, z_min, z_max;
		x_min = p.x() - DRONE_X/2;
		x_max = p.x() + DRONE_X/2;
		y_min = p.y() - DRONE_Y/2;
		y_max = p.y() + DRONE_Y/2;
		z_min = p.z() - DRONE_Z/2;
		z_max = p.z() + DRONE_Z/2;

		for (double x=x_min; x<x_max+RES; x+=RES){
			for (double y=y_min; y<y_max+RES; y+=RES){
				for (double z=z_min; z<z_max+RES; z+=RES){
					if (isValid(tree, point3d(x, y, z))){
						continue;
					}
					else{
						return false;
					}
				}
			}
		}
		return true;
	}
}


// Random Sample
// allow_not_valid = true indicates the configuration could be invalid by this generator
Node* randomConfig(const OcTree& tree, bool allow_not_valid=false){ 
	double min_x, max_x, min_y, max_y, min_z, max_z;
	tree.getMetricMax(max_x, max_y, max_z);
	tree.getMetricMin(min_x, min_y, min_z);
	min_x = std::max(min_x, env_x_min);
	max_x = std::min(max_x, env_x_max);
	min_y = std::max(min_y, env_y_min);
	max_y = std::min(max_y, env_y_max);
	min_z = std::max(min_z, env_z_min);
	max_z = std::min(max_z, env_z_max);
	bool valid = false;
	double x, y, z, yaw;
	point3d p;

	while (valid == false){
		p.x() = randomNumber(min_x, max_x);
		// Y>0
		// p.y() = randomNumber(0, max_y);
		p.y() = randomNumber(min_y, max_y);
		// p.z() = randomNumber(min_z, max_z);
		p.z() = randomNumber(min_z, 2.5); // garage and cafe
		yaw = randomNumber(0, 2*pi);
		if (allow_not_valid==true){ 
			break;
		}
		valid = isValid(tree, p, true);
		// valid = isValid(tree, p);
	}

	Node* nptr = new Node(p, yaw);

	return nptr;
}

// Adjust length by extending eps
// q_rand will be changed
void adjustLength(Node* q_rand, Node* q_near, double eps){
	point3d pr = q_rand->p;
	point3d pn = q_near->p;
	double distance = pr.distance(pn); // distance between two node
	point3d diff = pr - pn;
	q_rand->p.x() = pn.x() + (eps/distance) * diff.x();
	q_rand->p.y() = pn.y() + (eps/distance) * diff.y();
	q_rand->p.z() = pn.z() + (eps/distance) * diff.z();
}

// Function: Do Collision Checking
bool checkCollision(OcTree& tree, Node* n1, Node* n2){
	point3d p1 = n1->p;
	point3d p2 = n2->p;
	std::vector<point3d> ray;
	tree.computeRay(p1, p2, ray);
	for (std::vector<point3d>::iterator itr=ray.begin(); itr!=ray.end(); ++itr){
		if (isValid(tree, *itr ,true)){
		// if (isValid(tree, *itr)){
			continue;
		}
		else{
			return true;
		}
	}
	return false;
}

// NEED FIX
int calculateUnknown(const OcTree& tree, Node* n){
	// Position:
	point3d p = n->p;
	double yaw = n->yaw;
	// Possible range
	double xmin, xmax, ymin, ymax, zmin, zmax, distance;
	xmin = p.x() - dmax;
	xmax = p.x() + dmax;
	ymin = p.y() - dmax;
	ymax = p.y() + dmax;
	zmin = p.z() - dmax;
	zmax = p.z() + dmax;
	

	point3d pmin (xmin, ymin, zmin);
	point3d pmax (xmax, ymax, zmax);
	point3d_list node_centers;
	tree.getUnknownLeafCenters(node_centers, pmin, pmax);
	// cout << "Max horizontal Angle: " << horizontal_max_angle << endl;
	// cout << "Max Vertical Angle: " << vertical_max_angle << endl;
	int count = 0;
	int jump_factor = 1;
	int jump_count = 0;
	for (std::list<point3d>::iterator itr=node_centers.begin(); 
		itr!=node_centers.end(); ++itr){
		++jump_count;
		if ((jump_count-1) % jump_factor != 0){
			continue;
		}
		point3d u = *itr;
		bool not_in_scope = u.x() > env_x_max or u.x() < env_x_min or u.y() > env_y_max or u.y() < env_y_min or u.z() > env_z_max or u.z() < env_z_min;
		if (not_in_scope){
			continue;
		}
		OcTreeNode* nptr = tree.search(u);
		if (nptr == NULL){ // Unknown
			distance = p.distance(u);

			if (distance >= dmin and distance <= dmax){
				point3d face (cos(yaw), sin(yaw), 0);
				// This is not correct but just for convienence:
				point3d direction = u - p;
				// divide the direction into horizontal and vertical
				// horizontal
				point3d h_direction = direction;
				h_direction.z() = 0;
				double horizontal_angle = face.angleTo(h_direction);
				
				// vertical 
				point3d v_direction = direction;
				v_direction.x() = cos(yaw);
				v_direction.y() = sin(yaw);
				double vertical_angle = face.angleTo(v_direction);

				// if (abs(horizontal_angle) < horizontal_max_angle and abs(vertical_angle) < vertical_max_angle){
				double angle = face.angleTo(direction);
				if (angle <= FOV/2){
					point3d end;
					bool cast = tree.castRay(p, direction, end, true, distance);
					double cast_distance = p.distance(end);
					// cout << cast << endl;
					if (cast == false){ // No Occupied was hit
						++count;
					}
				}
			}
		}
	}

	return count;
}

// Main Function
KDTree* growRRT(OcTree& tree,
			 Node* start,
			 int num_sample,
			 int max_sample, 
			 double eps,
			 double &best_IG,
			 std::vector<geometry_msgs::Point> &tree_vis_array,
			 KDTree* t = NULL,
			 bool write_results = false
			){
	std::ofstream file;
	if (write_results){	
		std::string name = 	std::string("/home/zhefan/Desktop/Summer Project/experiment/data/5.12/tree_node_info") + std::to_string(step_num) + std::string(".txt");
		file.open(name);
		++step_num;
	}
	// Initialize kd-tree with start configuration
	if (t == NULL){
		t = new KDTree();	
	}
	(*t).insert(start);
	int count = 0;
	double best_sc_gain = 0;
	// (*t).setBest(start); 
	while (count < num_sample or best_IG == 0){ 
		// Random Configuration: (x, y, z, yaw):
		Node* q_rand = randomConfig(tree);

		// Find Nearest Neighbor:
		Node* q_near = (*t).nearestNeighbor(q_rand);


		// Adjust increment length by eps:
		adjustLength(q_rand, q_near, eps);

		// Collision Checking
		bool has_collision = checkCollision(tree, q_rand, q_near);

		if (!has_collision){
			// Add the edge
			q_rand->parent = q_near;
			q_rand->dis += q_near->dis;
			geometry_msgs::Point point_near, point_rand;
			point_near.x = q_near->p.x();
			point_near.y = q_near->p.y();
			point_near.z = q_near->p.z();
			point_rand.x = q_rand->p.x();
			point_rand.y = q_rand->p.y();
			point_rand.z = q_rand->p.z();
			tree_vis_array.push_back(point_near);
			tree_vis_array.push_back(point_rand);
			// Calculate Information Gain Visible
			// TODO: function for calculating information gain
			// calculate unknown
			int num_unknown = calculateUnknown(tree, q_rand);


			double IG = exp(-lambda*(q_near->dis)) * num_unknown;
			q_rand->ig = q_near->ig + IG;
			q_rand->num_unknown = num_unknown;
			// For shortcut part:
			double distance_sc = q_rand->p.distance(start->p);
			q_rand->sc_gain = num_unknown;

			if (q_rand->ig > best_IG){
				best_IG = q_rand->ig;
				(*t).setBest(q_rand); 
			}

			if (q_rand->sc_gain > best_sc_gain){
				best_sc_gain = q_rand->sc_gain;
				(*t).setBestSc(q_rand);
			}
			
			// Insert node 
			(*t).insert(q_rand);	
			if (write_results){
				writeNodeInfo(q_rand, file);
			}		
		}

		++count;
		// if (count > max_sample){
		// 	// cout << count << endl;
		// 	// cout << max_sample << endl;
		// 	// cout << best_IG << endl;
		// 	// cout << "size: " << (*t).getSize() << endl;
		// 	// cout << "Maximum Number Reached" << endl;
		// 	// cout << "Exploration Finished!!!" << endl;
		// 	return NULL; // termination
		// }
	}
	// cout << "tree size: " << (*t).getSize() << endl;
	// cout << "Best IG: " << best_IG << endl;
	// cout << "Best IG sc: " << best_sc_gain << endl;
	// cout << "Best Node: " << (*t).getBest() << endl;
	// cout << "Best Node sc: " << (*t).getBestSc() << endl;
	if (write_results){
		file.close();
	}
	return t;
}

		// auto start3 = high_resolution_clock::now();
		// auto stop3 = high_resolution_clock::now();
		// auto duration3 = duration_cast<microseconds>(stop3 - start3);
		// cout << duration3.count() << endl;

void inflateMap(OcTree& tree, float xsize, float ysize, float zsize, double res){
	// Iterate through all nodes
	for (OcTree::leaf_iterator itr = tree.begin_leafs();
		itr != tree.end_leafs(); ++itr){
		point3d p = itr.getCoordinate();
		OcTreeNode* nptr = tree.search(p);
		bool occ = tree.isNodeOccupied(nptr);
		// If it is occupied
		if (occ == true){
			for (float i=p.x()-xsize; i<=p.x()+xsize; i+=res){
				for (float j=p.y()-ysize; j<=p.y()+ysize; j+=res){
					for (float k=p.z()-zsize; k<=p.z()+zsize; k+=res){
						point3d temp (i, j, k);
						tree.updateNode(temp, true);
					}
				}
			}
		}
	}
}

std::vector<Node*> shortCutPath(std::vector<Node*> path, OcTree& tree){
	// Overall Process:
	// check collision from the last one to previous ones, and shortcut
	// Pointer to the last element of the path
	if (path.size() <= 2){
		return path;
	}
	std::vector<Node*> new_path;
	int ptr1 = path.size() - 1;
	int ptr2 = path.size() - 2; 
	new_path.push_back(path[ptr1]);
	while (true){
		bool has_collision = checkCollision(tree, path[ptr1], path[ptr2]);
		if (!has_collision){
			ptr2 -= 1;
		}
		else{
			ptr1 = ptr2;
			new_path.push_back(path[ptr2+1]); // Because we could only connect to the previous one
		}

		if (ptr2 == 0){
			break;
		}
	}
	new_path.push_back(path[0]);
	std::reverse(new_path.begin(), new_path.end());
	return new_path;
}


// =====================Visualize Information Gain/ No Run time efficiency consdieration============
void visualizeUnknown(const OcTree& tree, Node* n, std::vector<geometry_msgs::Point> &unknown_voxels){
	// tree.setResolution(0.3);
	// Position:
	point3d p = n->p;
	double yaw = n->yaw;
	// Possible range
	double xmin, xmax, ymin, ymax, zmin, zmax, distance;
	xmin = p.x() - dmax;
	xmax = p.x() + dmax;
	ymin = p.y() - dmax;
	ymax = p.y() + dmax;
	zmin = p.z() - dmax;
	zmax = p.z() + dmax;

	point3d pmin (xmin, ymin, zmin);
	point3d pmax (xmax, ymax, zmax);
	point3d_list node_centers;
	tree.getUnknownLeafCenters(node_centers, pmin, pmax);
	// cout << "Max horizontal Angle: " << horizontal_max_angle << endl;
	// cout << "Max Vertical Angle: " << vertical_max_angle << endl;

	int jump_factor = 1;
	// std::list<point3d> sparse_node_centers;
	int count = 0;
	for (std::list<point3d>::iterator itr=node_centers.begin(); 
		itr!=node_centers.end(); ++itr){

		point3d u = *itr;
		bool not_in_scope = u.x() > env_x_max or u.x() < env_x_min or u.y() > env_y_max or u.y() < env_y_min or u.z() > env_z_max or u.z() < env_z_min;
		if (not_in_scope){
			continue;
		}
		OcTreeNode* nptr = tree.search(u);
		if (nptr == NULL){ // Unknown
			distance = p.distance(u);

			if (distance >= dmin and distance <= dmax){
				point3d face (cos(yaw), sin(yaw), 0);
				// This is not correct but just for convienence:
				point3d direction = u - p;
				// divide the direction into horizontal and vertical
				// horizontal
				point3d h_direction = direction;
				h_direction.z() = 0;
				double horizontal_angle = face.angleTo(h_direction);
				
				// vertical 
				point3d v_direction = direction;
				double length_weight = sqrt(u.x()*u.x()+u.y()*u.y()) * cos(horizontal_angle);
				v_direction.x() = cos(yaw) * length_weight;
				v_direction.y() = sin(yaw) * length_weight;
				double vertical_angle = face.angleTo(v_direction);

				// if (abs(horizontal_angle) < horizontal_max_angle and abs(vertical_angle) < vertical_max_angle){
				double angle = face.angleTo(direction);
				if (angle <= FOV/2){
					point3d end;
					bool cast = tree.castRay(p, direction, end, true, distance);
					double cast_distance = p.distance(end);
					// cout << cast << endl;
					if (cast == false){ // No Occupied was hit
						++count;
						geometry_msgs::Point u_point;
						u_point.x = u.x();
						u_point.y = u.y();
						u_point.z = u.z();
						unknown_voxels.push_back(u_point);
					}
				}
			}
		}
	}
}

void writeNodeInfo(Node* n, std::ofstream &file){
	file << n->p.x() << " ";
	file << n->p.y() << " ";
	file << n->p.z() << " ";
	file << n->yaw << " ";
	file << n->num_unknown << "\n";
}


void getNeighbors(point3d p, std::vector<point3d> &neighbors){
	std::vector<double> x_range {-RES, 0, RES};
	std::vector<double> y_range {-RES, 0, RES};
	std::vector<double> z_range {-RES, 0, RES};
	for (double dx: x_range){
		for (double dy: y_range){
			for (double dz: z_range){
				neighbors.push_back(point3d(p.x()+dx, p.y()+dy, p.z()+dz));
			}
		}
	}

	return;
}

void getFrontiers(const OcTree& tree, std::vector<geometry_msgs::Point> &frontier){
	// Frontier is the cell seperate free and unknown
	// need to search 26 directions for each cell if the cell is free
	KeySet record_frontiers;
	for (OcTree::leaf_iterator itr = tree.begin_leafs(), end=tree.end_leafs(); itr != end; ++itr){
		OcTreeNode* nptr = tree.search(itr.getCoordinate());
		bool is_occupied = tree.isNodeOccupied(nptr);
		if (!is_occupied){
			std::vector<point3d> neighbors;
			getNeighbors(itr.getCoordinate(), neighbors);
			for (point3d neighbor: neighbors){
				OcTreeNode* neighbor_ptr = tree.search(neighbor);
				if (neighbor_ptr == NULL){
					record_frontiers.insert(tree.coordToKey(neighbor));	
				}
			}
		}
	}

	// Store data into vector for visualization
	for (KeySet::iterator f_itr = record_frontiers.begin(); f_itr != record_frontiers.end(); ++ f_itr){
		point3d p = tree.keyToCoord(*f_itr);
		geometry_msgs::Point p_point;
		p_point.x = p.x();
		p_point.y = p.y();
		p_point.z = p.z();
		frontier.push_back(p_point);
	}

	return;
}