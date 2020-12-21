#include <ros/ros.h>
#include <aeplanner/planner.h>
#include <nav_msgs/Odometry.h>
#include <octomap_msgs/Octomap.h>
#include <geometry_msgs/Pose.h>
#include <octomap_msgs/conversions.h>
#include <message_filters/sync_policies/approximate_time.h> 
#include <message_filters/time_synchronizer.h>
#include <message_filters/subscriber.h>
#include <tf2/LinearMath/Quaternion.h>
#include "geometry_msgs/Quaternion.h"
#include "tf/transform_datatypes.h"
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <tf2/LinearMath/Quaternion.h>
#include <aeplanner/Goal.h>
#include <chrono> 
#include <iostream>
#include <fstream>

using namespace std::chrono; 
using namespace message_filters;
ros::Publisher goal_pub;
ros::Publisher path_pub;
ros::Publisher tree_vis_pub;
ros::Publisher ig_vis_pub;
ros::Publisher frontier_vis_pub;
std::ofstream replan_data;

// ==========================================================================================
// double res = 0.1;
int num_sample =200, max_sample = 1000;
// int num_sample = 800, max_sample = 2000;
int count_low_info_gain = 0;
double linear_velocity = 0.3;
double eps = 1.0;
// double eps = 0.8;
OcTree* tree_ptr = new OcTree(RES);
AbstractOcTree* abtree;
float current_x, current_y, current_z, current_ox, current_oy, current_oz, current_ow,
	  goal_x, goal_y, goal_z, eps_x, eps_y, eps_z, eps_yaw, eps_q1, eps_q2, eps_q3, eps_q4;
point3d goal_position (-1000, -1000, -1000);
point3d last_position (-1000, -1000, -1000);
point3d first_time = goal_position;
double yaw = 0;
double last_yaw = 0;
bool first_time_entry = true;
std::vector<Node*> branch;
std::vector<geometry_msgs::Point> tree_vis_array;
std::vector<geometry_msgs::Point> ig_vis_array;
std::vector<geometry_msgs::Point> frontier_vis_array;
geometry_msgs::Point next_position;
geometry_msgs::Pose next_pose;
aeplanner::Goal goal;
double best_IG = 0;
nav_msgs::Path path;
visualization_msgs::Marker marker;
visualization_msgs::Marker ig_marker;
visualization_msgs::Marker frontier_marker;
int count_path_segment = 0;
double total_time = 0;
double total_computation_time = 0;
double total_path_length = 0;


// AEP parameter:
bool init_cache = true; // If true, we initialize the cache
KDTree* cache;
std::priority_queue<Node*, std::vector<Node*>, GainCompareNode> cache_queue;

// Give the longest length of the best branch
int max_length = 5;

// ==========================================================================================

// =====================================Result===============================================
int overall_voxels = 0;
int last_time_voxels = 0;
int current_voxels = 0;
int step = 0;
double current_IG = 0;
double next_IG = 0;
double last_tree_size = 0;
double last_size_increment = 100;
double last_two_size_increment = 100;
double last_best_ig = 0;

// std::ofstream output_file("/home/zhefan/Desktop/aeplanner_result/result.txt");
// =======================================END================================================


nav_msgs::Path convertPathMsg(std::vector<Node*> path){
	nav_msgs::Path path_msg;
	vector<geometry_msgs::PoseStamped> msg;
	for (int i = 0; i < path.size(); ++i){
		geometry_msgs::Point p;
		geometry_msgs::PoseStamped ps;
		geometry_msgs::Quaternion q;
		p.x = path[i]->p.x();
		p.y = path[i]->p.y();
		p.z = path[i]->p.z();
		ps.pose.position = p;
		msg.push_back(ps);
	}
	path_msg.poses = msg;
	return path_msg;
}

auto start_time_total = high_resolution_clock::now();
void callback(const nav_msgs::OdometryConstPtr& odom, const octomap_msgs::Octomap::ConstPtr& bmap){
	
	// cout << "Test" << endl;
	// Convert binary tree into octomap
	abtree = octomap_msgs::binaryMsgToMap(*bmap);
	tree_ptr = dynamic_cast<OcTree*>(abtree);
	
	tree_ptr->setResolution(RES);
	
	// Initialize Cache if it is not initialized:
	if (init_cache){
		cache = new KDTree();

		init_cache = false;
	}

	// Get the current state:
	current_x = odom->pose.pose.position.x;
	current_y = odom->pose.pose.position.y;
	current_z = odom->pose.pose.position.z;
	current_ox = odom->pose.pose.orientation.x;
	current_oy = odom->pose.pose.orientation.y;
	current_oz = odom->pose.pose.orientation.z;
	current_ow = odom->pose.pose.orientation.w;
	geometry_msgs::Quaternion quat = odom->pose.pose.orientation;
	tf::Quaternion tf_quat;
	tf::quaternionMsgToTF(quat, tf_quat);
	double current_roll, current_pitch, current_yaw;
	tf::Matrix3x3(tf_quat).getRPY(current_roll, current_pitch, current_yaw);

	// Goal Position/Initialize one if not exist for the first time
	// FIX THIS: WE DONT CHECK YAW
	goal_x = goal_position.x();
	goal_y = goal_position.y();
	goal_z = goal_position.z();
	tf2::Quaternion goal_rot;
	goal_rot.setRPY(0, 0, yaw);

	// Calculate eps: difference between current position and goal position
	eps_x = std::abs(goal_x - current_x);
	eps_y = std::abs(goal_y - current_y);
	eps_z = std::abs(goal_z - current_z);
	// eps_q1 = std::abs(goal_rot[0] - current_ox);
	// eps_q2 = std::abs(goal_rot[1] - current_oy);
	// eps_q3 = std::abs(goal_rot[2] - current_oz);
	// eps_q4 = std::abs(goal_rot[3] - current_ow);
	eps_yaw = std::abs(yaw - (current_yaw));

	bool reach = eps_x < 0.2 and eps_y < 0.2 and eps_z < 0.2 and ((eps_yaw < 0.1) or (std::abs(eps_yaw-2*pi) < 0.1));
	if (reach or goal_position == first_time){
		bool frontier_exploration = false;
		cout << "=========================" << count_path_segment << "=========================" << endl;
		++count_path_segment;
		auto start_time = high_resolution_clock::now();
		tree_vis_array.clear();
		// Initialize Start Node and Tree
		Node* start = new Node(point3d(current_x, current_y, current_z), 0); // Yaw doesn't matter
		KDTree* t = new KDTree();
		if (not frontier_exploration){
		
		// Insert previous best branch
			for (int i=1; i<branch.size(); ++i){
				if (i == branch.size()-1){
					t->setBest(branch[i]);
				}
				if (i == 1){
					branch[i]->parent = NULL;
					// branch[i]->ig = 0; 
				}
				else{
					double yaw = 0;
					branch[i]->ig = branch[i-1]->ig + exp(-lambda*eps) * calculateUnknown(*tree_ptr, branch[i], yaw);
				}
				t->insert(branch[i]);
				// geometry_msgs::Point prev_point, current_point;
				// prev_point.x = branch[i-1]->p.x();
				// prev_point.y = branch[i-1]->p.y();
				// prev_point.z = branch[i-1]->p.z();
				// current_point.x = branch[i]->p.x();
				// current_point.y = branch[i]->p.y();
				// current_point.z = branch[i]->p.z();
				// tree_vis_array.push_back(prev_point);
				// tree_vis_array.push_back(current_point);
			}
		}
		for (int i =1; i<branch.size(); ++i){
			geometry_msgs::Point prev_point, current_point;
			prev_point.x = branch[i-1]->p.x();
			prev_point.y = branch[i-1]->p.y();
			prev_point.z = branch[i-1]->p.z();
			current_point.x = branch[i]->p.x();
			current_point.y = branch[i]->p.y();
			current_point.z = branch[i]->p.z();
			tree_vis_array.push_back(prev_point);
			tree_vis_array.push_back(current_point);
		}
		
		// Call Planner
		best_IG = 0;


		// AEP: Autonomous Exploration Planner
		// If the branch is not good enough (How to define???), then we navigate to the point in cache
		
		branch = planner(*tree_ptr, start, num_sample, max_sample, eps, best_IG, tree_vis_array, t, cache, frontier_exploration, cache_queue, false);
		
		
		auto stop_time = high_resolution_clock::now();
		auto duration = duration_cast<microseconds>(stop_time - start_time);
		total_path_length += eps;
		cout << "resoltuion: " << tree_ptr->getResolution() << endl;
		cout << "Computation Time for this iteration: " << duration.count()/1e6 << " seconds" << endl;
		replan_data << duration.count()/1e6 << endl;
		total_computation_time += duration.count()/1e6;
		cout << "Computation time so far: " << total_computation_time << " seconds" <<endl;
		cout << "Information Gain: " << best_IG << endl;
		cout << "Path Length: " << total_path_length << endl;
		double current_tree_size = tree_ptr->size();
		cout << "Voxel Number: " << current_tree_size << endl;
		double current_size_increment = current_tree_size - last_tree_size;
		last_tree_size = current_tree_size;
		double total_size_increment = current_size_increment + last_size_increment + last_two_size_increment;
		cout << "Total three times increment: " << total_size_increment << endl;

		// if (std::abs(total_size_increment) <= 3){
		// 	cout << "Terminates" << endl;
		// 	// ros::shutdown();
		// 	// return;
		// }

		if (best_IG <= 0){
			count_low_info_gain += 1;
		}
		else{
			count_low_info_gain = 0;
		}

		// double volume = 0;
		// for (OcTree::leaf_iterator it=tree_ptr->begin_leafs(), end=tree_ptr->end_leafs(); it != end; ++it){
		// 		double leaf_size = it.getSize();
		// 		volume += leaf_size * leaf_size * leaf_size;
		// }
		// cout << "Volume: " << volume << endl;
		// if (count_low_info_gain >= 5){
		// 	cout << "Terminates" << endl;
		// 	double volume = 0;
		// 	for (OcTree::leaf_iterator it=tree_ptr->begin_leafs(), end=tree_ptr->end_leafs(); it != end; ++it){
		// 		double leaf_size = it.getSize();
		// 		volume += leaf_size * leaf_size * leaf_size;
		// 	}
		// 	cout << "Volume: " << volume << endl;
		// 	ros::shutdown();
		// 	return;
		// }
		// if (branch.size() == 0){ 
		// 	auto stop_time_total = high_resolution_clock::now();
		// 	auto duration_total = duration_cast<microseconds>(stop_time_total - start_time_total);
		// 	cout << "Total: "<< duration_total.count()/1e6 << " seconds | " << "Exploration Terminates!!!!!!!!!!!" << endl;
		// 	cout << "Number of Path Segment is: " << count_path_segment << endl;
		// 	cout << "Total Path Length is: " << count_path_segment * eps << endl;
		// 	cout << "Total Computation Time is: " << total_computation_time << endl;
		// 	cout << "Avg Computation Time per Segment is: " << total_computation_time/count_path_segment << endl;

		// 	ros::shutdown();
		// 	return;
		// }
		cout << "=========================" << "END" << "=========================" << endl;

		// branch = std::vector<Node*>(branch.begin(), branch.begin()+max_length); 
		// print_node_vector(branch);
		// branch = shortCutPath(branch, *tree_ptr);
		path = convertPathMsg(branch);
		// cout << "tree size: " << t->getSize() << endl;
		// cout << "Best INFO GAIN: " << best_IG << endl;
		// cout << "branch size: " << branch.size() << endl;

		last_position = branch[0]->p;		
		goal_position = branch[1]->p;
		double delta_eps = last_position.distance(goal_position);
		yaw = branch[1]->yaw;

		// get result:
		vector<double> results;
		// 0. # of steps:


		// 1. Next Theoretical Information Gain:
		current_IG = next_IG; // Update current theoretical information gain
		// next_IG = best_IG;
		next_IG = branch[1]->ig;

		// 2. Overall Tree Voxels:
		overall_voxels = tree_ptr->size();

		// 3. Current Voxel increase:
		current_voxels = overall_voxels - last_time_voxels;
		last_time_voxels = overall_voxels;


		// // Write Results
		// results.push_back(step);
		// results.push_back(current_IG);
		// results.push_back(current_voxels);
		// results.push_back(overall_voxels);
		// std::ostream_iterator<double> output_iterator(output_file, " ");
		// std::copy(results.begin(), results.end(), output_iterator);

		// vector<std::string> seperate;
		// seperate.push_back("");
		// std::ostream_iterator<std::string> seperate_iterator(output_file, "\n");
		// std::copy(seperate.begin(), seperate.end(), seperate_iterator);
		// // Update:
		// ++step;

		// Calculate and store the ig voxels
		ig_vis_array.clear();
		visualizeUnknown(*tree_ptr, branch[1], ig_vis_array);
		frontier_vis_array.clear();
		getFrontiers(*tree_ptr, frontier_vis_array);
		// delete t;
		goal.x = goal_position.x();
		goal.y = goal_position.y();
		goal.z = goal_position.z();
		goal.yaw = yaw; 
		goal.linear_velocity = linear_velocity;
		if (first_time_entry){
			double angular_velocity = (yaw-current_yaw)/(delta_eps/linear_velocity);
			goal.angular_velocity = angular_velocity;
			first_time_entry = false;
		}
		else{
			goal.angular_velocity = (yaw-last_yaw)/(delta_eps/linear_velocity);
		}
		goal.is_last = true;
		last_yaw = yaw;
		
	}
	delete abtree;
	// // Publish Goal
	// next_position.x = goal_position.x();
	// next_position.y = goal_position.y();
	// next_position.z = goal_position.z();
	// next_pose.position.x = goal_position.x();
	// next_pose.position.y = goal_position.y();
	// next_pose.position.z = goal_position.z();

	tf2::Quaternion q;
	q.setRPY(0, 0, yaw);
	next_pose.orientation.x = q[0];
	next_pose.orientation.y = q[1];
	next_pose.orientation.z = q[2];
	next_pose.orientation.w = q[3];
	// next_pose.header.stamp = ros::Time::now();
	// next_odom.pose.pose.orientation = q;
	// print_point3d(goal_position);
	// cout << "eps: " << eps_x << ", "<< eps_y << ", " << eps_z << ", " << eps_yaw << endl;
	// cout << "goal yaw: " << yaw << endl;
	// cout << "currnet yaw: " << current_yaw << endl;
	// goal_pub.publish(next_pose);
	// goal_pub.publish(next_odom);
	// Path Msg:
	path.header.frame_id = odom->header.frame_id;
	path_pub.publish(path);


	// Tree Msg: 
	marker.header.frame_id = odom->header.frame_id;
	marker.points = tree_vis_array;
	marker.id = 0;
	marker.type = visualization_msgs::Marker::LINE_LIST;
	marker.scale.x = 0.05;
	marker.scale.y = 0.05;
	marker.scale.z = 0.05;
	marker.color.a = 1.0;
	tree_vis_pub.publish(marker);

	// IG Msg:
	ig_marker.header.frame_id = odom->header.frame_id;
	ig_marker.points = ig_vis_array;
	ig_marker.id = 1;
	ig_marker.type = visualization_msgs::Marker::POINTS;
	ig_marker.scale.x = 0.05;
	ig_marker.scale.y = 0.05;
	ig_marker.scale.z = 0.05;
	ig_marker.color.a = 1.0;
	ig_vis_pub.publish(ig_marker);

	// Frontier Msg:
	frontier_marker.header.frame_id = odom->header.frame_id;
	frontier_marker.points = frontier_vis_array;
	frontier_marker.id = 1;
	frontier_marker.type = visualization_msgs::Marker::POINTS;
	frontier_marker.scale.x = 0.05;
	frontier_marker.scale.y = 0.05;
	frontier_marker.scale.z = 0.05;
	frontier_marker.color.a = 1.0;
	frontier_marker.color.r = 0.0;
	frontier_marker.color.g = 1.0;
	frontier_marker.color.b = 0.0;
	frontier_vis_pub.publish(frontier_marker);
}


int main(int argc, char** argv){
	ros::init(argc, argv, "drone_exploration");
	ros::NodeHandle nh;
	message_filters::Subscriber<nav_msgs::Odometry> odom_sub(nh, "odom", 100);
	message_filters::Subscriber<octomap_msgs::Octomap> map_sub(nh, "octomap_binary", 100);
	typedef sync_policies::ApproximateTime<nav_msgs::Odometry, octomap_msgs::Octomap> MySyncPolicy;
	Synchronizer<MySyncPolicy> sync (MySyncPolicy(100), odom_sub, map_sub);
	sync.registerCallback(boost::bind(&callback, _1, _2));
	// TODO: goal pub need to support yaw
	goal_pub = nh.advertise<aeplanner::Goal>("/goal", 1);
	// goal_pub = nh.advertise<nav_msgs::Odometry>("/get_goal", 10);
	path_pub = nh.advertise<nav_msgs::Path>("path", 1);
	tree_vis_pub = nh.advertise<visualization_msgs::Marker>("/tree_vis_array", 1);
	ig_vis_pub = nh.advertise<visualization_msgs::Marker>("/ig_vis_array", 1);
	frontier_vis_pub = nh.advertise<visualization_msgs::Marker>("/frontier_vis_array", 1);
	ros::Rate loop_rate(10);
	// replan_data.open("/home/zhefan/Desktop/Replan_Data_New/aeplanner/replan_data1.txt");
	while (ros::ok()){
		goal_pub.publish(goal);
		// std::ofstream data;
		// data.open("/home/zhefan/Desktop/Experiment_Stats/apartment/aeplanner/apartment_stats5.txt");
		// auto stop_time_total = high_resolution_clock::now();
		// auto total_duration = duration_cast<microseconds>(stop_time_total - start_time_total);
		// total_time = total_duration.count()/1e6;
		// data << "Total Exploration Time: " << total_time << endl;
		// data << "Total Computation Time: " << total_computation_time << endl;
		// data << "Total Path Length: " << total_path_length << endl;
		ros::spinOnce();
		loop_rate.sleep();
	}
	// ros::spin();
	return 0;
}