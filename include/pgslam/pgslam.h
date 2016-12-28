#ifndef _PGSLAM_H_
#define _PGSLAM_H_

#include <vector>
#include <string>

#include <Eigen/Eigen>
#include <isam/isam.h>

namespace pgslam {

class Pose2D {
friend class LaserScan;
private:
	double x;
	double y;
	double theta;
public:
	Pose2D ();
	Pose2D (double x, double y, double theta);
	void set_x(double x);
	void set_y(double y);
	void set_theta(double theta);
	Pose2D operator + (Pose2D p);
	Pose2D operator - (Pose2D p);
	Pose2D inverse ();
	Eigen::Vector2d pos ();
	double get_theta ();
	std::string to_string ();
};

class Echo {
private:
	double range;
	double angle;
	double intensity;
	int64_t time_stamp;
public:
	Echo (double range, double angle, double intensity, int64_t time_stamp);
	double get_range ();
	double get_angle ();
	double get_intensity ();
	int64_t get_time_stamp ();
	Eigen::Vector2d get_point ();
};

class LaserScan {
private:
	std::vector<Eigen::Vector2d> points_self;
	std::vector<Eigen::Vector2d> points_world;
	Pose2D pose;
	bool world_transformed_flag;
	void UpdateToWorld ();
	double max_x;
	double min_x;
	double max_y;
	double min_y;

	double match_threshold;
	double dist_threshold;

	std::vector<Eigen::Vector2d> transform (const std::vector<Eigen::Vector2d> &v, Pose2D pose);
	Pose2D icp(std::vector<Eigen::Vector2d> scan_ref, std::vector<Eigen::Vector2d> scan, double *ratio, Pose2D reference_pose);
	int nearest (std::vector<Eigen::Vector2d> &v, Eigen::Vector2d p);
public:
	LaserScan (std::vector<Echo> echos);
	LaserScan (std::vector<Echo> echos, Pose2D pose);
	Pose2D get_pose ();
	void set_pose (Pose2D pose);
	const std::vector<Eigen::Vector2d> & get_points ();
	Pose2D ICP (const LaserScan &scan, double *ratio);
	double get_max_x_in_world ();
	double get_min_x_in_world ();
	double get_max_y_in_world ();
	double get_min_y_in_world ();

};

class GraphSlam {
private:
	isam::Slam * slam;
	std::vector<isam::Pose2d_Node*> pose_nodes;
	bool check (int id);

public:
	GraphSlam ();
	void AddPose2dFactor (size_t node_id, Pose2D pose_ros, double cov);
	void AddPose2dPose2dFactor (size_t node_id_ref, size_t node_id, Pose2D pose_ros, double cov);
	void remove (size_t node_id);
	std::vector< std::pair<size_t, Pose2D> > get_nodes ();
	std::vector< std::pair<Eigen::Vector2d, Eigen::Vector2d> > get_factors ();
	void clear ();
};

class Slam {
private:
	std::vector<LaserScan> scans;
	Pose2D pose;
	double keyscan_threshold;
	double factor_threshold;
	GraphSlam graph_slam;
	void(*pose_update_callback)(void);
	void(*map_update_callback)(void);
	Pose2D EncoderToPose2D (double left, double right, double tread);
public:
	Slam ();
	void set_keyscan_threshold (double keyscan_threshold);
	void set_factor_threshold (double factor_threshold);
	void UpdatePoseWithPose (Pose2D pose);
	void UpdatePoseWithEncoder (double left, double right, double tread);
	void UpdatePoseWithLaserScan (const LaserScan &scan);
	Pose2D get_pose ();
	const std::vector<LaserScan> & get_scans ();
	std::vector< std::pair<Eigen::Vector2d, Eigen::Vector2d> > get_factors ();
	void RegisterPoseUpdateCallback (void(*f)(void));
	void RegisterMapUpdateCallback (void(*f)(void));
};

} // namespace pose_graph_slam

#endif

