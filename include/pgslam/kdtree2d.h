#ifndef _KDTREE2D_H_
#define _KDTREE2D_H_

#include <vector>
#include <Eigen/Eigen>

namespace kd_tree_2d {

class Node {
private:
	Eigen::Vector2d point;
	int dir;
	size_t index;
	Node *left,*right;
public:
	Node (Eigen::Vector2d point, size_t index, int dir);
	~Node ();
	Eigen::Vector2d get_point ();
	size_t get_index();
	void insert (Eigen::Vector2d point, size_t index);
	Node * Nearest (Eigen::Vector2d point);
};

class KDTree2D {
private:
	Node *root;
	void insert (Eigen::Vector2d point, size_t index);
public:
	KDTree2D ();
	void Construct (const std::vector<Eigen::Vector2d> &points);
	Eigen::Vector2d Nearest (Eigen::Vector2d point);
	size_t NearestIndex (Eigen::Vector2d point);
	~KDTree2D ();
};


} // namespace kd_tree_2d

#endif
