#include <pgslam/kdtree2d.h>
#include <iostream>

using namespace kd_tree_2d;

Node::Node (Eigen::Vector2d point, size_t index, int dir)
{
	this->point = point;
	this->index = index;
	left = nullptr;
	right = nullptr;
	this->dir = dir;
}

Eigen::Vector2d Node::get_point ()
{
	return point;
}

size_t Node::get_index ()
{
	return index;
}

void Node::insert (Eigen::Vector2d point, size_t index)
{
	if (point(dir) <= this->point(dir)) {
		if (left==nullptr) {
			left = new Node (point, index, (dir+1)%2);
		} else {
			left->insert (point, index);
		}
	}
	else {
		if (right==nullptr) {
			right = new Node (point, index, (dir+1)%2);
		} else {
			right->insert (point, index);
		}
	}
}

Node * Node::Nearest (Eigen::Vector2d point)
{
	Node *near_side, *far_side;
	Node *nearside_best=nullptr;
	Node *farside_best=nullptr;
	if (left==nullptr && right==nullptr)
		return this;
	if (point(dir) <= this->point(dir)) {
		near_side = left;
		far_side = right;
	} else {
		near_side = right;
		far_side = left;
	}

	if (near_side!=nullptr) {
		nearside_best = near_side->Nearest (point);
	}

	if ( nearside_best==nullptr || (nearside_best->point - point).norm() > fabs(point(dir)-this->point(dir))) {
		if (far_side!=nullptr) {
			farside_best = far_side->Nearest (point);
		}
	}
	Node * best = this;
	if (nearside_best!=nullptr && (nearside_best->point-point).norm() < (best->point-point).norm()) {
		best = nearside_best;
	}
	if (farside_best!=nullptr && (farside_best->point-point).norm() < (best->point-point).norm()) {
		best = farside_best;
	}
	return best;
}

Node::~Node ()
{
	if (left!=nullptr)
		delete left;
	if (right!=nullptr)
		delete right;
}

KDTree2D::KDTree2D ()
{
	root = nullptr;
}

void KDTree2D::insert (Eigen::Vector2d point, size_t index)
{
	if (root==nullptr) {
		root = new Node (point,index,0);
	} else {
		root->insert (point,index);
	}
}

void KDTree2D::Construct (const std::vector<Eigen::Vector2d> &points)
{
	std::vector<size_t> index;
	for (size_t i=0; i<points.size(); i++)
		index.push_back (i);
	srand (time(NULL));
	for (size_t i=0; i<index.size()/2; i++) {
		size_t a = rand()%index.size();
		size_t b = rand()%index.size();
		size_t t = index[a];
		index[a] = index[b];
		index[b] = t;
	}
	for (size_t i=0; i<index.size(); i++) {
		insert (points[index[i]],index[i]);
	}
}

Eigen::Vector2d KDTree2D::Nearest (Eigen::Vector2d point)
{
	assert (root!=nullptr);
	auto result = root->Nearest (point);
	if (result==nullptr) {
		std::cout << "error" << std::endl;
	}
	else {
		return result->get_point();
	}
}

size_t KDTree2D::NearestIndex (Eigen::Vector2d point)
{
	assert (root!=nullptr);
	auto result = root->Nearest (point);
	if (result==nullptr) {
		std::cout << "error" << std::endl;
	}
	else {
		return result->get_index();
	}
}

KDTree2D::~KDTree2D ()
{
	delete root;
}
