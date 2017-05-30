/*
 * Copyright 2017 Yu Kunlin <yukunlin@mail.ustc.edu.cn>
 */
#ifndef PGSLAM_H_
#define PGSLAM_H_

#include <Eigen/Eigen>
#ifdef USE_ISAM
#  include <isam/isam.h>
#endif

#include <vector>
#include <string>
#include <utility>
#include <functional>

namespace pgslam {

class Pose2D {
 public:
  Pose2D();
  Pose2D(double x, double y, double theta);
  void set_x(double x);
  void set_y(double y);
  void set_theta(double theta);
  double x() const;
  double y() const;
  double theta() const;
  Pose2D operator *(Pose2D p) const;
  Pose2D inverse() const;
  Eigen::Vector2d pos() const;
  Eigen::Rotation2D<double> ToRotation() const;
  Eigen::Transform<double, 2, Eigen::Affine> ToTransform() const;
  std::string ToJson() const;
 private:
  double x_;
  double y_;
  double theta_;
  friend class LaserScan;
};

class Echo {
 private:
  double range_;
  double angle_;
  double intensity_;
  int64_t time_stamp_;
 public:
  Echo(double range, double angle, double intensity, int64_t time_stamp);
  double get_range();
  double get_angle();
  double get_intensity();
  int64_t get_time_stamp();
  Eigen::Vector2d get_point();
};

class LaserScan {
 public:
  explicit LaserScan(std::vector<Echo> echos);
  LaserScan(std::vector<Echo> echos, Pose2D pose);
  Pose2D get_pose() const;
  void set_pose(Pose2D pose);
  const Eigen::Matrix2Xd& get_points();
  Pose2D ICP(const LaserScan &scan, double *ratio);
  double get_max_x_in_world();
  double get_min_x_in_world();
  double get_max_y_in_world();
  double get_min_y_in_world();

 private:
  void UpdateToWorld();

 private:
  Eigen::Matrix2Xd points_;
  Eigen::Matrix2Xd points_world_;
  Pose2D pose_;
  bool world_transformed_flag_;
  double max_x_;
  double min_x_;
  double max_y_;
  double min_y_;

  double match_threshold_;
  double dist_threshold_;
};


#ifdef USE_ISAM
class GraphSlam {
 public:
  GraphSlam();
  void AddPose2dFactor(size_t node_id, Pose2D pose_ros, double cov);
  void AddPose2dPose2dFactor(size_t node_id_ref,
      size_t node_id, Pose2D pose_ros, double cov);
  void remove(size_t node_id);
  std::vector<std::pair<size_t, Pose2D>> get_nodes();
  std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> get_factors();
  void clear();
  void Optimization();

 private:
  bool check(size_t id);

 private:
  isam::Slam * slam_;
  std::vector<isam::Pose2d_Node*> pose_nodes_;
};
#endif

class Slam {
 public:
  Slam();
  void set_keyscan_threshold(double keyscan_threshold);
  void set_factor_threshold(double factor_threshold);
  void UpdatePoseWithPose(Pose2D pose);
  void UpdatePoseWithEncoder(double left, double right, double tread);
  void UpdatePoseWithLaserScan(const LaserScan &scan);
  Pose2D get_pose() const;
  const std::vector<LaserScan> & get_scans();
#ifdef USE_ISAM
  std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> get_factors();
#endif
  void RegisterPoseUpdateCallback(std::function<void(Pose2D)> f);
  void RegisterMapUpdateCallback(std::function<void(void)> f);

 private:
  Pose2D EncoderToPose2D(double left, double right, double tread);

 private:
  std::vector<LaserScan> scans_;
  Pose2D pose_;
  double keyscan_threshold_;
  double factor_threshold_;
#ifdef USE_ISAM
  GraphSlam graph_slam_;
#endif
  std::function<void(Pose2D)> pose_update_callback;
  std::function<void(void)> map_update_callback;
};

}  // namespace pgslam

#endif  // PGSLAM_H_
