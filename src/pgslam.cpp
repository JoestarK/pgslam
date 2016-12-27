#include <pgslam/pgslam.h>

#include <iomanip>
#include <iostream>
#include <sstream>
#include <float.h>

#include <Eigen/Eigen>

using namespace pgslam;

Pose2D::Pose2D ()
{
	this->x = 0;
	this->y = 0;
	this->theta = 0;
}

Pose2D::Pose2D (double x, double y, double theta)
{
	this->x = x;
	this->y = y;
	this->theta = theta;
	while (this->theta<-M_PI) this->theta += 2 * M_PI;
	while (this->theta> M_PI) this->theta -= 2 * M_PI;
}

void Pose2D::set_x(double x)
{
	this->x = x;
}
void Pose2D::set_y(double y)
{
	this->y = y;
}
void Pose2D::set_theta(double theta)
{
	this->theta = theta;
	while (this->theta<-M_PI) this->theta += 2 * M_PI;
	while (this->theta> M_PI) this->theta -= 2 * M_PI;
}

Pose2D Pose2D::operator + (Pose2D p)
{
	Eigen::Vector2d v1(x,y);
	Eigen::Vector2d v2(p.x,p.y);
	Eigen::Rotation2D<double> rot(theta);
	v2 = rot * v2;
	Eigen::Vector2d v = v1 + v2;
	return Pose2D (v.x(), v.y(), theta+p.theta);
}

Pose2D Pose2D::inverse ()
{
	Eigen::Vector2d v(-x,-y);
	Eigen::Rotation2D<double> rot(-theta);
	v = rot * v;
	return Pose2D (v.x(), v.y(), -theta);
}

Pose2D Pose2D::operator - (Pose2D p)
{
	return p.inverse() + *this;
}

Eigen::Vector2d Pose2D::pos ()
{
	return Eigen::Vector2d (x,y);
}

double Pose2D::get_theta ()
{
	return theta;
}

std::string Pose2D::to_string ()
{
	std::stringstream ss;
	ss << "x:";
	ss << std::setw (7) << setiosflags(std::ios::fixed) << std::setprecision (4);
	ss << x;
	ss << " y:";
	ss << std::setw (7) << setiosflags(std::ios::fixed) << std::setprecision (4);
	ss << y;
	ss << " theta:";
	ss << std::setw (7) << setiosflags(std::ios::fixed) << std::setprecision (4);
	ss << theta;
	return ss.str();
}

Echo::Echo (double range, double angle, double intensity, int64_t time_stamp)
{
	this->range = range;
	this->angle = angle;
	this->intensity = intensity;
	this->time_stamp = time_stamp;
}

double Echo::get_range ()
{
	return range;
}

double Echo::get_angle ()
{
	return angle;
}

double Echo::get_intensity ()
{
	return intensity;
}

int64_t Echo::get_time_stamp ()
{
	return time_stamp;
}

Eigen::Vector2d Echo::get_point ()
{
	double x = range * cos(angle);
	double y = range * sin(angle);
	return Eigen::Vector2d (x,y);
}

LaserScan::LaserScan (std::vector<Echo> echos)
{
	for (size_t i=0; i<echos.size(); i++)
		points_self.push_back (echos[i].get_point());
	world_transformed_flag = false;

	match_threshold = 0.1;
	dist_threshold = 1.0;
}

LaserScan::LaserScan (std::vector<Echo> echos, Pose2D pose)
{
	for (size_t i=0; i<echos.size(); i++)
		points_self.push_back (echos[i].get_point());
	this->pose = pose;
	world_transformed_flag = false;

	match_threshold = 0.1;
	dist_threshold = 1.0;
}

Pose2D LaserScan::get_pose ()
{
	return pose;
}

void LaserScan::set_pose (Pose2D pose)
{
	this->pose = pose;
	world_transformed_flag = false;
}

const std::vector<Eigen::Vector2d> & LaserScan::get_points ()
{
	UpdateToWorld ();
	return points_world;
}

void LaserScan::UpdateToWorld ()
{
	if (world_transformed_flag) return;

	std::vector<Eigen::Vector2d>().swap (points_world);

	max_x = 0.0;
	min_x = 0.0;
	max_y = 0.0;
	min_y = 0.0;

	Eigen::Rotation2D<double> rot (pose.theta);
	Eigen::Vector2d move = pose.pos();
	for (size_t i=0; i<points_self.size(); i++) {
		Eigen::Vector2d p = rot * points_self[i] + move;
		points_world.push_back (p);
		if (p.x()>max_x) max_x = p.x();
		if (p.x()<min_x) min_x = p.x();
		if (p.y()>max_y) max_y = p.y();
		if (p.y()<min_y) min_y = p.y();
	}

	world_transformed_flag = true;
}

int LaserScan::nearest (std::vector<Eigen::Vector2d> &v, Eigen::Vector2d p)
{
	double dist = DBL_MAX;
	int index_rough=-1,index;
	int speed_up_num = 10;
	for (int i=0; i<v.size(); i+=speed_up_num) {
		if ((v[i]-p).norm()<dist) {
			dist = (v[i]-p).norm();
			index_rough = i;
		}
	}
	index = index_rough;
	for (int j=index_rough-speed_up_num; j<index_rough+speed_up_num; j++) {
		int i = (j+v.size())%v.size();
		if ((v[i]-p).norm()<dist) {
			dist = (v[i]-p).norm();
			index = i;
		}
	}
	return index;
}

std::vector<Eigen::Vector2d> LaserScan::transform (const std::vector<Eigen::Vector2d> &v, Pose2D pose)
{
	std::vector<Eigen::Vector2d> v2 (v.size());
	Eigen::Rotation2D<double> rot (pose.theta);
	Eigen::Vector2d move (pose.x, pose.y);
	for (int i=0; i<v.size(); i++) {
		v2[i] = rot * v[i];
		v2[i] += move;
	}
	return v2;
}

Pose2D LaserScan::icp(std::vector<Eigen::Vector2d> scan_ref, std::vector<Eigen::Vector2d> scan, double *ratio, Pose2D reference_pose)
{
	int insert_num = 10;
	auto scan_cache = scan_ref;
	scan_ref.resize(scan_cache.size()*insert_num);
	for (size_t i=0; i<scan_cache.size()-1; i++) { // TODO size>2
		for (size_t j=0; j<insert_num; j++) {
			scan_ref[insert_num*i+j] = (scan_cache[i+1] - scan_cache[i]) / insert_num * j + scan_cache[i];
		}
	}

	std::vector<Eigen::Vector2d> scan_origin = scan;
	Pose2D pose = reference_pose;
	for (int i=0; i<20; i++) {
		scan = transform (scan_origin, pose);

		std::vector<Eigen::Vector2d>    near = scan;                 // store the closest point
		std::vector< std::vector<int> >	trace_back(scan_ref.size()); // to trace some points have a same closest point
		std::vector<bool>		mask(scan.size());           // true: effective point; false: non-effective point;

		// search and save nearest point
		int match_count = 0;
		for (int i=0; i<scan.size(); i++) {
			Eigen::Vector2d point = scan[i];

			int index = nearest (scan_ref,scan[i]);
			if (index==-1) {
				if (ratio!=nullptr)
					*ratio = 0.0;
				return Pose2D();
			}
			trace_back[index].push_back(i);
			Eigen::Vector2d closest = scan_ref[index];

			double distance = (point-closest).norm();
			if (distance < match_threshold)
				match_count ++;
			if (distance < dist_threshold) {
				near[i] = closest;
				mask[i] = true;
			} else {
				mask[i] = false;
			}
		}
		if (ratio!=nullptr)
			*ratio = (double)match_count/scan.size();

		// disable the points have one same nearest point
		for (int i=0; i<trace_back.size(); i++)
			if (trace_back[i].size()>3)
				for (int j=0; j<trace_back[i].size(); j++) {
					mask[trace_back[i][j]] = false;
					near [trace_back[i][j]] = scan[trace_back[i][j]];
				}

		// disable the farest 10% point
		std::vector<double> max_distance;
		std::vector<int>    max_index;
		max_distance.resize (scan.size()/10,0.0);
		max_index.resize (scan.size()/10,0);
		for (int i=0; i<scan.size(); i++) {
			double distance = (scan[i]-near[i]).norm();
			for (int j=1; j<max_distance.size(); j++) {
				if (distance>max_distance[j]) {
					max_distance[j-1] = max_distance[j];
					max_index[j-1] = max_index[j];
					if (j==max_distance.size()-1) {
						max_distance[j] = distance;
						max_index[j] = i;
					}
				}
				else {
					max_distance[j-1] = distance;
					max_index[j-1] = i;
					break;
				}
			}
		}
		for (int i=1; i<max_index.size(); i++)
			mask[max_index[i]] = false;

		// calc center
		Eigen::Vector2d center(0,0);
		int count = 0;
		for (int i=0; i<scan.size(); i++)
			if (mask[i]) {
				center += scan[i];
				count ++;
			}
		if (count==0) {
			std::cout << "Error: no valid point, return reference pose." << std::endl;
			if (ratio!=nullptr)
				*ratio = 0.0;
			return reference_pose;
		}
		center /= count;

		// calc the tramsform
		Eigen::Vector2d move = Eigen::Vector2d(0.0,0.0);
		double rot = 0.0;
		for (int i=0; i<scan.size(); i++) {
			if (mask[i]==false) continue;
			Eigen::Vector2d delta = near[i] - scan[i];
			double length = delta.norm();
			if (length>0) {
				delta.normalize();
				delta *= (length<0.05)?length:sqrt(length*20)/20;
			}
			move += delta;
			Eigen::Vector2d p = scan[i] - center;
			Eigen::Vector2d q = near[i] - center;
			if (p.norm()<DBL_EPSILON*2) continue;

			rot += (p.x()*q.y() - p.y()*q.x()) / p.norm() / sqrt(p.norm());
		}
		move /= count;
		rot /= count;

		// speed up
		move *= 2.0;
		rot *= 1.0;

		pose.x += move.x();
		pose.y += move.y();
		pose.theta += rot;
	}
	return pose;
}

Pose2D LaserScan::ICP (const LaserScan &scan_, double *ratio)
{
	LaserScan scan = scan_;
	return icp (points_self, scan.points_self, ratio, scan.pose-pose);
}

double LaserScan::get_max_x_in_world ()
{
	UpdateToWorld ();
	return max_x;
}

double LaserScan::get_min_x_in_world ()
{
	UpdateToWorld ();
	return min_x;
}

double LaserScan::get_max_y_in_world ()
{
	UpdateToWorld ();
	return max_y;
}

double LaserScan::get_min_y_in_world ()
{
	UpdateToWorld ();
	return min_y;
}

GraphSlam::GraphSlam ()
{
	slam = new isam::Slam();
}

bool GraphSlam::check (int id)
{
	// size enough
	if (id<pose_nodes.size())
		if (pose_nodes[id]==NULL) {	// already been removed
			pose_nodes[id] = new isam::Pose2d_Node();
			return true;
		}
		else return false;	// still there

	// create new space
	int size = pose_nodes.size();
	pose_nodes.resize(id+1);
	for (int i=size; i<id+1; i++)
		pose_nodes[i] = new isam::Pose2d_Node();
	return true;
}

void GraphSlam::remove (size_t node_id)
{
	slam->remove_node (pose_nodes[node_id]);
	delete pose_nodes[node_id];
	pose_nodes[node_id] = NULL;
	slam->batch_optimization();
}

void GraphSlam::clear ()
{
	delete slam;
	slam = new isam::Slam();
	std::vector<isam::Pose2d_Node*>().swap(pose_nodes);
}

std::vector< std::pair<size_t, Pose2D> > GraphSlam::get_nodes ()
{
	std::vector< std::pair<size_t, Pose2D> > pose_id;
	// store in response
	for (size_t i=0; i<pose_nodes.size(); i++) {
		if (pose_nodes[i]==NULL) continue;
		Pose2D pose;
		pose.set_x(pose_nodes[i]->value().x());
		pose.set_y(pose_nodes[i]->value().y());
		pose.set_theta(pose_nodes[i]->value().t());
		pose_id.push_back (std::pair<size_t, Pose2D>(i,pose));
	}
	return pose_id;
}

std::vector< std::pair<Eigen::Vector2d, Eigen::Vector2d> > GraphSlam::get_factors ()
{
	std::vector< std::pair<Eigen::Vector2d, Eigen::Vector2d> > factors;
	const std::list<isam::Factor*> factors_ = slam->get_factors();
	for (std::list<isam::Factor*>::const_iterator it=factors_.begin(); it!=factors_.end(); it++) {
		std::vector<isam::Node*> nodes = (*it)->nodes();
		if (nodes.size()==1) continue;
		Eigen::Vector2d first;
		first.x() = ((isam::Pose2d_Node*)nodes[0])->value().x();
		first.y() = ((isam::Pose2d_Node*)nodes[0])->value().y();
		Eigen::Vector2d second;
		second.x() = ((isam::Pose2d_Node*)nodes[1])->value().x();
		second.y() = ((isam::Pose2d_Node*)nodes[1])->value().y();
		factors.push_back (std::pair<Eigen::Vector2d,Eigen::Vector2d>(first,second));
	}
	return factors;
}

void GraphSlam::AddPose2dFactor (size_t node_id, Pose2D pose_ros, double cov)
{
	isam::Pose2d pose (pose_ros.pos().x(), pose_ros.pos().y(), pose_ros.get_theta());
	if (cov<=0) {
		cov = 1.0;
	}

	// check new node
	bool ret = check (node_id);
	if (ret) slam->add_node (pose_nodes[node_id]);

	// add factor
	isam::Noise noise = isam::Information (cov * isam::eye(3));
	isam::Pose2d_Factor * factor = new isam::Pose2d_Factor(pose_nodes[node_id], pose, noise);
	slam->add_factor(factor);

	slam->batch_optimization();
}

void GraphSlam::AddPose2dPose2dFactor (size_t node_id_ref, size_t node_id, Pose2D pose_ros, double cov)
{
	isam::Pose2d pose (pose_ros.pos().x(), pose_ros.pos().y(), pose_ros.get_theta());
	if (cov<=0) {
		cov = 1.0;
	}

	// check new node
	bool ret = check (node_id_ref);
	if (ret) slam->add_node (pose_nodes[node_id_ref]);
	ret = check (node_id);
	if (ret) slam->add_node (pose_nodes[node_id]);

	// add factor
	isam::Noise noise = isam::Information (cov * isam::eye(3));
	isam::Pose2d_Pose2d_Factor * factor = new isam::Pose2d_Pose2d_Factor(pose_nodes[node_id_ref], pose_nodes[node_id], pose, noise);
	slam->add_factor(factor);

	slam->batch_optimization();
}

Slam::Slam ()
{
	keyscan_threshold = 0.4;
	factor_threshold = 0.6;
	//TODO factor > keyscan*2
	pose_update_callback = nullptr;
	map_update_callback = nullptr;
}

Pose2D Slam::get_pose ()
{
	return pose;
}

const std::vector<LaserScan> & Slam::get_scans ()
{
	return scans;
}

std::vector< std::pair<Eigen::Vector2d, Eigen::Vector2d> > Slam::get_factors ()
{
	return graph_slam.get_factors();
}

Pose2D Slam::EncoderToPose2D (double left, double right, double tread)
{
	double theta = (right-left) / tread;
	double theta_2 = theta / 2.0;
	double arc = (right+left) / 2.0;
	double radius = arc / theta;
	double secant = 2 * sin (theta_2) * radius;
	if (theta == 0)
		secant = arc;
	double x = secant * cos (theta_2);
	double y = secant * sin (theta_2);

	return Pose2D (x,y,theta);
}

void Slam::UpdatePoseWithPose (Pose2D pose)
{
	this->pose = this->pose + pose;
}

void Slam::UpdatePoseWithEncoder (double left, double right, double tread)
{
	pose = pose + EncoderToPose2D (left,right,tread);
	if (pose_update_callback != nullptr)
		pose_update_callback ();
}

void Slam::UpdatePoseWithLaserScan (const LaserScan &scan_)
{
	LaserScan scan = scan_;
	scan.set_pose (pose);

	// first scan
	if (scans.empty()) {
		scans.push_back (scan);
		graph_slam.AddPose2dFactor (0,pose,1);
		std::cout << "add key scan " << scans.size() << ": " << pose.to_string() << std::endl;
		if (map_update_callback != nullptr)
			map_update_callback ();
		return;
	}

	// search for the closest scan
	LaserScan *closest_scan;
	double min_dist = DBL_MAX;
	for (size_t i=0; i<scans.size(); i++) {
		double dist = (scans[i].get_pose().pos() - scan.get_pose().pos()).norm();
		double delta_theta = fabs(scans[i].get_pose().get_theta() - scan.get_pose().get_theta());;
		while (delta_theta<-M_PI) delta_theta += 2 * M_PI;
		while (delta_theta> M_PI) delta_theta -= 2 * M_PI;
		delta_theta *= keyscan_threshold / (M_PI_4 * 3.0);

		dist = sqrt(dist*dist + delta_theta*delta_theta);
		if (dist < min_dist) {
			min_dist = dist;
			closest_scan = &(scans[i]);
		}
	}

	if (min_dist < keyscan_threshold) {
		// update pose
		double ratio;
		Pose2D pose_delta = closest_scan->ICP (scan, &ratio);
		pose = closest_scan->get_pose() + pose_delta;
	}
	else {
		// add key scan
		for (size_t i=0; i<scans.size(); i++) {
			double distance = (pose.pos() - scans[i].get_pose().pos()).norm();
			if (distance<factor_threshold) {
				double ratio;
				Pose2D pose_delta = scans[i].ICP (scan, &ratio);
				graph_slam.AddPose2dPose2dFactor (i, scans.size(), pose_delta, ratio);
			}
		}

		auto nodes = graph_slam.get_nodes ();
		for (size_t i=0; i<nodes.size(); i++) {
			// update pose of node
			for (int j=0; j<scans.size(); j++) {
				if (j != nodes[i].first) continue;
				scans[j].set_pose (nodes[i].second);
				break;
			}
			// add the new scan with pose
			if (nodes[i].first == scans.size()) {
				pose = nodes[i].second;
				scan.set_pose (nodes[i].second);
				scans.push_back (scan);
			}

		}
		std::cout << "add key scan " << scans.size() << ": " << pose.to_string() << std::endl;

		if (map_update_callback != nullptr)
			map_update_callback ();
	}
	if (pose_update_callback != nullptr)
		pose_update_callback ();
}

void Slam::RegisterPoseUpdateCallback (void(*f)(void))
{
	pose_update_callback = f;
}
void Slam::RegisterMapUpdateCallback (void(*f)(void))
{
	map_update_callback = f;
}

