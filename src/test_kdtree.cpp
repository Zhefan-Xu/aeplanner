#include <ros/ros.h>
#include <iostream>
#include <aeplanner/kdtree.h>
#include <aeplanner/utils.h>
using std::cout; using std::endl;

void testNode(){
	cout << "================Test Node===================" << endl;
	point3d p1 (1, 2, 3);
	point3d p2 (2, 3, 4);
	point3d p3 (3, 4, 5);
	double yaw1 = 0;
	double yaw2 = 1;
	double yaw3 = 2;
	double ig1 = 10;
	double ig2 = 20;
	double ig3 = 30;
	Node n1 (p1, yaw1);
	Node n2 (p2, yaw2);
	Node n3 (p2, yaw3);
	n1.ig = ig1;
	n2.ig = ig2;
	n3.ig = ig3;
	cout << "Node attributes: " << endl;
	// print_point3d(n1.p);
	// cout << "yaw: " << n1.yaw << endl;
	// cout << "info gain: " <<  n1.ig << endl;
	print_node(n1);
	// Test Parent/Edges           n2->n3 n2->n1
	cout << "Edges: " << endl;
	n1.parent = &(n2);
	n3.parent = &(n2);
	print_point3d(n1.parent->p); // should be (2, 3, 4)
	print_point3d(n3.parent->p); // should be (2, 3, 4)
	cout << (n2.parent == NULL) << endl; // should be true
	cout << n2.parent << endl;
	cout << "============================================" << endl;
	return;
}

void test_kdtree_insert(){
	cout << "============Test KDTree Insert==============" << endl;
	point3d p1 (1, 2, 3);
	point3d p2 (2, 3, 4);
	point3d p3 (3, 4, 5);
	point3d p4 (5, 2, 1);
	point3d p5 (10, 0, 10);
	double yaw1 = 0;
	double yaw2 = 1;
	double yaw3 = 2;
	double yaw4 = 3;
	double yaw5 = 4;
	Node n1 (p1, yaw1);
	Node n2 (p2, yaw2);
	Node n3 (p3, yaw3);
	Node n4 (p4, yaw4);
	Node n5 (p5, yaw5);

	//Initiaize KDTree;
	KDTree t1; 
	t1.insert(&n2);
	t1.insert(&n1);
	t1.insert(&n3);
	t1.insert(&n4);
	t1.insert(&n5);
	cout << "size of kdtree: " << t1.getSize() <<  endl;
	Node* root = t1.getRoot();
	print_node((*root));
	print_node(*(root->left));
	print_node(*(root->right));
	print_node(*(root->right->left));
	print_node(*(root->right->left->right));
	// print_node((*root));
	// print_node(*(root->left));
	// print_node(*(root->right));
	// print_node(*(root->right->left));
	cout << "============================================" << endl;
	return;
}

void test_kdtree_nn(){
	cout << "==========Test KDTree Nearest Neighbor=========" << endl;
	point3d p1 (1, 2, 3);
	point3d p2 (2, 3, 4);
	point3d p3 (3, 4, 5);
	point3d p4 (5, 2, 1);
	point3d p5 (10, 0, 10);
	double yaw1 = 0;
	double yaw2 = 1;
	double yaw3 = 2;
	double yaw4 = 3;
	double yaw5 = 4;
	Node n1 (p1, yaw1);
	Node n2 (p2, yaw2);
	Node n3 (p3, yaw3);
	Node n4 (p4, yaw4);
	Node n5 (p5, yaw5);

	//Initiaize KDTree;
	KDTree t1; 
	t1.insert(&n2);
	t1.insert(&n1);
	t1.insert(&n3);
	t1.insert(&n4);
	t1.insert(&n5);
	cout << "size of kdtree: " << t1.getSize() <<  endl;

	// Find nearest neighbor for n6
	point3d p6 (9, 0, 9); // nearest neighbor should be (10, 0, 10)
	double yaw6 = 5;
	Node n6 (p6, yaw6);
	Node* nn1 = t1.nearestNeighbor(&n6);
	print_node(*nn1);

	// Find nearest neighbor for n7
	point3d p7 (2.1, 2, 3); // nearest neighbor should be (1, 2, 3)
	double yaw7 = 6;
	Node n7 (p7, yaw7);
	Node* nn2 = t1.nearestNeighbor(&n7);
	print_node(*nn2);

	// Find nearest neighbor for n9
	point3d p8 (6, 3, 4);
	point3d pa (6, 1, 0);
	point3d pb (3, 3, 0);
	point3d pc (9, 3, 0);
	point3d p9 (9, 3, 2); // nearest neighbor should be (9, 3, 0)
	double yaw8 = 7;
	double yaw9 = 8;
	Node n8 (p8, yaw8);
	Node na (pa, 0);
	Node nb (pb, 0);
	Node nc (pc, 0);
	Node n9 (p9, yaw9);
	t1.insert(&na);
	t1.insert(&nb);
	t1.insert(&nc);
	Node* nn3 = t1.nearestNeighbor(&n9);
	print_node(*nn3);

	// Test Clear Tree:
	t1.clear();
	cout << "size of tree after clear: " << t1.getSize() << endl;
	cout << "root: " << t1.getRoot() << endl;
	cout << "===============================================" << endl;
}


// Test
int main(int argc, char** argv){
	cout << "Test Receding Horizon Next-Best-View" << endl;
	testNode();
	test_kdtree_insert();
	test_kdtree_nn();
	return 0;
}