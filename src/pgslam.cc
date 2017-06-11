/*
 * Copyright 2017 Yu Kunlin <yukunlin@mail.ustc.edu.cn>
 */
#include <pgslam/pgslam.h>
#include <pgslam/kdtree2d.h>

#include <float.h>
#include <sys/time.h>
#include <Eigen/Eigen>

#include <iomanip>
#include <iostream>
#include <sstream>

using pgslam::Pose2D;
using pgslam::Echo;
using pgslam::LaserScan;
using pgslam::GraphSlam;
using pgslam::Slam;

Pose2D::Pose2D() {
  this->x_ = 0;
  this->y_ = 0;
  this->theta_ = 0;
}

Pose2D::Pose2D(double x, double y, double theta) {
  x_ = x;
  y_ = y;
  theta_ = theta;
  while (theta_ < -M_PI) theta_ += 2 * M_PI;
  while (theta_ >  M_PI) theta_ -= 2 * M_PI;
}

void Pose2D::set_x(double x) { x_ = x; }

void Pose2D::set_y(double y) { y_ = y; }

void Pose2D::set_theta(double theta) {
  theta_ = theta;
  while (theta_ < -M_PI) theta_ += 2 * M_PI;
  while (theta_ >  M_PI) theta_ -= 2 * M_PI;
}

double Pose2D::x() const { return x_; }
double Pose2D::y() const { return y_; }
double Pose2D::theta () const { return theta_; }

Pose2D Pose2D::operator *(Pose2D p) const {
  Eigen::Vector2d v = p.pos() + p.ToRotation() * pos();
  return Pose2D(v.x(), v.y(), theta_ + p.theta_);
}

Pose2D Pose2D::inverse() const {
  Eigen::Vector2d v = ToRotation().inverse() * pos();
  return Pose2D (-v.x(), -v.y(), -theta_);
}

Eigen::Vector2d Pose2D::pos() const {
  return Eigen::Vector2d(x_, y_);
}

Eigen::Rotation2D<double> Pose2D::ToRotation() const {
  return Eigen::Rotation2D<double>(theta_);
}

Eigen::Transform<double, 2, Eigen::Affine> Pose2D::ToTransform() const {
  Eigen::Transform<double, 2, Eigen::Affine> transform;
  transform.setIdentity();
  transform.translate(pos());
  transform.rotate(ToRotation());
  return transform;
}

std::string Pose2D::ToJson() const {
  std::stringstream ss;
  ss << setiosflags(std::ios::fixed) << std::setprecision(4);
  ss << "{\"x\":" << x_;
  ss << ",\"y\":"  << y_;
  ss << ",\"theta\":" << theta_ << "}";
  return ss.str();
}

Echo::Echo(double range, double angle, double intensity, int64_t time_stamp) {
  range_ = range;
  angle_ = angle;
  intensity_ = intensity;
  time_stamp_ = time_stamp;
}

double Echo::range() const { return range_; }
double Echo::angle() const { return angle_; }
double Echo::intensity() const { return intensity_; }
int64_t Echo::time_stamp() const { return time_stamp_; }

Eigen::Vector2d Echo::point() const {
  double x = range_ * cos(angle_);
  double y = range_ * sin(angle_);
  return Eigen::Vector2d(x, y);
}

LaserScan::LaserScan(std::vector<Echo> echos) {
  points_.resize(Eigen::NoChange, echos.size());
  for (size_t i = 0; i < echos.size(); i++)
    points_.col(i) = echos[i].point();
  world_transformed_flag_ = false;

  match_threshold_ = 0.1;
  dist_threshold_ = 1.0;
}

LaserScan::LaserScan(std::vector<Echo> echos, Pose2D pose) {
  points_.resize(Eigen::NoChange, echos.size());
  for (size_t i = 0; i < echos.size(); i++)
    points_.col(i) = echos[i].point();
  pose_ = pose;
  world_transformed_flag_ = false;

  match_threshold_ = 0.1;
  dist_threshold_ = 1.0;
}

Pose2D LaserScan::pose() const {
  return pose_;
}

void LaserScan::set_pose(Pose2D pose) {
  pose_ = pose;
  world_transformed_flag_ = false;
}

const Eigen::Matrix2Xd& LaserScan::points() {
  UpdateToWorld();
  return points_world_;
}

void LaserScan::UpdateToWorld() {
  if (world_transformed_flag_) return;

  points_world_.resize(Eigen::NoChange, points_.cols());

  max_x_ = 0.0;
  min_x_ = 0.0;
  max_y_ = 0.0;
  min_y_ = 0.0;

  auto t = pose_.ToTransform();
  for (size_t i = 0; i < points_.cols(); i++) {
    Eigen::Vector2d p = t * points_.col(i);
    points_world_.col(i) = p;
    if (p.x() > max_x_) max_x_ = p.x();
    if (p.x() < min_x_) min_x_ = p.x();
    if (p.y() > max_y_) max_y_ = p.y();
    if (p.y() < min_y_) min_y_ = p.y();
  }

  world_transformed_flag_ = true;
}

Pose2D LaserScan::ICP(const LaserScan &scan, double *ratio) {
  Pose2D reference_pose = scan.pose() * pose_.inverse();

  // interpolate
  size_t interpolate_num = 7;
  Eigen::Matrix2Xd points_ref;
  points_ref.resize(Eigen::NoChange, points_.cols() * interpolate_num);
  for (size_t i = 0; i < points_.cols() - 1; i++) {
    for (size_t j = 0; j < interpolate_num; j++) {
      Eigen::Vector2d curr = points_.col(i + 0);
      Eigen::Vector2d next = points_.col(i + 1);
      double gain = static_cast<double>(j) / interpolate_num;
      points_ref.col(interpolate_num * i + j) = (next - curr) * gain + curr;
    }
  }

  // construct kd tree
  kd_tree_2d::KDTree2D tree;
  tree.Construct(points_ref);

  // iterate
  Pose2D pose = reference_pose;
  for (int i = 0; i < 100; i++) {
    Eigen::Matrix2Xd points = pose.ToTransform() * scan.points_;

    // store the closest point
    Eigen::Matrix2Xd near = points;
    // to trace some points have a same closest point
    std::vector<std::vector<int>> trace_back(points_ref.cols());
    // true: effective point; false: non-effective point;
    std::vector<bool> mask(points.cols());

    // search and save nearest point
    int match_count = 0;
    for (size_t i = 0; i < points.cols(); i++) {
      Eigen::Vector2d point = points.col(i);

      size_t index = tree.NearestIndex(points.col(i));

      trace_back[index].push_back(i);
      Eigen::Vector2d closest = points_ref.col(index);

      double distance = (point - closest).norm();
      if (distance < match_threshold_)
        match_count++;
      if (distance < dist_threshold_) {
        near.col(i) = closest;
        mask[i] = true;
      } else {
        mask[i] = false;
      }
    }
    if (ratio != nullptr)
      *ratio = static_cast<double>(match_count) / points.cols();

    // disable the points have one same nearest point
    for (size_t i = 0; i < trace_back.size(); i++)
      if (trace_back[i].size() > 3) {
        for (size_t j = 0; j < trace_back[i].size(); j++) {
          mask[trace_back[i][j]] = false;
          near.col(trace_back[i][j]) = points.col(trace_back[i][j]);
        }
      }

    // disable the farest 10% point
    std::vector<double> max_distance;
    std::vector<int>    max_index;
    max_distance.resize(points.cols() / 10, 0.0);
    max_index.resize(points.cols() / 10, 0);
    for (size_t i = 0; i < points.cols(); i++) {
      double distance = (points.col(i) - near.col(i)).norm();
      for (size_t j = 1; j < max_distance.size(); j++) {
        if (distance > max_distance[j]) {
          max_distance[j-1] = max_distance[j];
          max_index[j-1] = max_index[j];
          if (j == max_distance.size()-1) {
            max_distance[j] = distance;
            max_index[j] = i;
          }
        } else {
          max_distance[j-1] = distance;
          max_index[j-1] = i;
          break;
        }
      }
    }
    for (size_t i = 1; i < max_index.size(); i++)
      mask[max_index[i]] = false;

    // calc center
    Eigen::Vector2d center(0, 0);
    int count = 0;
    for (size_t i = 0; i < points.cols(); i++)
      if (mask[i]) {
        center += points.col(i);
        count++;
      }
    if (count == 0) {
      std::cout << "Error: no valid point, return reference pose." << std::endl;
      if (ratio != nullptr)
        *ratio = 0.0;
      return reference_pose;
    }
    center /= count;

    // calc the tramsform
    Eigen::Vector2d move = Eigen::Vector2d(0.0, 0.0);
    double rot = 0.0;
    for (size_t i = 0; i < points.cols(); i++) {
      if (mask[i] == false) continue;
      Eigen::Vector2d delta = near.col(i) - points.col(i);
      double length = delta.norm();
      if (length > 0) {
        delta.normalize();
        delta *= (length < 0.05) ? length : sqrt(length * 20) / 20;
      }
      move += delta;
      Eigen::Vector2d p = points.col(i) - center;
      Eigen::Vector2d q = near.col(i) - center;
      if (p.norm() < DBL_EPSILON * 2) continue;

      rot += (p.x() * q.y() - p.y() * q.x()) / p.norm() / sqrt(p.norm());
    }
    move /= count;
    rot /= count;

    // speed up
    move *= 2.0;
    rot *= 1.0;

    // update pose
    Pose2D pose_delta = pose * Pose2D(move.x(), move.y(), rot) * pose.inverse();
    pose = pose_delta * pose;
    if (pose_delta.pos().norm() < 0.001 && pose_delta.theta() < 0.001)
      break;
  }
  return pose;
}

double LaserScan::max_x_in_world() {
  UpdateToWorld();
  return max_x_;
}

double LaserScan::min_x_in_world() {
  UpdateToWorld();
  return min_x_;
}

double LaserScan::max_y_in_world() {
  UpdateToWorld();
  return max_y_;
}

double LaserScan::min_y_in_world() {
  UpdateToWorld();
  return min_y_;
}

#ifdef USE_ISAM
GraphSlam::GraphSlam() {
  slam_ = new isam::Slam();
}

bool GraphSlam::check(size_t id) {
  // size enough
  if (id < pose_nodes_.size()) {
    if (pose_nodes_[id] == NULL) {  // already been removed
      pose_nodes_[id] = new isam::Pose2d_Node();
      return true;
    } else {
      return false;  // still there
    }
  }

  // create new space
  int size = pose_nodes_.size();
  pose_nodes_.resize(id + 1);
  for (size_t i = size; i < id + 1; i++)
    pose_nodes_[i] = new isam::Pose2d_Node();
  return true;
}

void GraphSlam::remove(size_t node_id) {
  slam_->remove_node(pose_nodes_[node_id]);
  delete pose_nodes_[node_id];
  pose_nodes_[node_id] = NULL;
  slam_->batch_optimization();
}

void GraphSlam::clear() {
  delete slam_;
  slam_ = new isam::Slam();
  std::vector<isam::Pose2d_Node*>().swap(pose_nodes_);
}

std::vector<std::pair<size_t, Pose2D>> GraphSlam::nodes() {
  std::vector<std::pair<size_t, Pose2D>> pose_id;
  // store in response
  for (size_t i = 0; i < pose_nodes_.size(); i++) {
    if (pose_nodes_[i] == NULL) continue;
    Pose2D pose;
    pose.set_x(pose_nodes_[i]->value().x());
    pose.set_y(pose_nodes_[i]->value().y());
    pose.set_theta(pose_nodes_[i]->value().t());
    pose_id.push_back(std::pair<size_t, Pose2D>(i, pose));
  }
  return pose_id;
}

std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> GraphSlam::factors() {
  std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> factors;
  const std::list<isam::Factor*> factors_ = slam_->get_factors();
  for (std::list<isam::Factor*>::const_iterator it = factors_.begin();
      it != factors_.end(); it++) {
    std::vector<isam::Node*> nodes = (*it)->nodes();
    if (nodes.size() == 1) continue;
    Eigen::Vector2d first;
    first.x() = ((isam::Pose2d_Node*)nodes[0])->value().x();
    first.y() = ((isam::Pose2d_Node*)nodes[0])->value().y();
    Eigen::Vector2d second;
    second.x() = ((isam::Pose2d_Node*)nodes[1])->value().x();
    second.y() = ((isam::Pose2d_Node*)nodes[1])->value().y();
    factors.push_back(std::make_pair(first, second));
  }
  return factors;
}

void GraphSlam::AddPose2dFactor(size_t node_id, Pose2D pose_ros, double cov) {
  isam::Pose2d pose(pose_ros.x(), pose_ros.y(), pose_ros.theta());
  if (cov <= 0) {
    cov = 1.0;
  }

  // check new node
  bool ret = check(node_id);
  if (ret) slam_->add_node(pose_nodes_[node_id]);

  // add factor
  isam::Noise noise = isam::Information(cov * isam::eye(3));
  isam::Pose2d_Factor * factor =
    new isam::Pose2d_Factor(pose_nodes_[node_id], pose, noise);
  slam_->add_factor(factor);
  // slam_->batch_optimization();
}

void GraphSlam::AddPose2dPose2dFactor(size_t node_id_ref,
    size_t node_id, Pose2D pose_ros, double cov) {
  isam::Pose2d pose(pose_ros.x(), pose_ros.y(), pose_ros.theta());
  if (cov <= 0) {
    cov = 1.0;
  }

  // check new node
  bool ret = check(node_id_ref);
  if (ret) slam_->add_node(pose_nodes_[node_id_ref]);
  ret = check(node_id);
  if (ret) slam_->add_node(pose_nodes_[node_id]);

  // add factor
  isam::Noise noise = isam::Information(cov * isam::eye(3));
  isam::Pose2d_Pose2d_Factor * factor =
    new isam::Pose2d_Pose2d_Factor(pose_nodes_[node_id_ref],
        pose_nodes_[node_id], pose, noise);
  slam_->add_factor(factor);
  // slam_->batch_optimization();
}

void GraphSlam::Optimization() {
  slam_->batch_optimization();
}
#endif

Slam::Slam() {
  keyscan_threshold_ = 0.4;
  factor_threshold_ = 0.9;
}

void Slam::set_keyscan_threshold(double keyscan_threshold) {
  this->keyscan_threshold_ = keyscan_threshold;
  if (keyscan_threshold_ * 2 > factor_threshold_)
    factor_threshold_ = keyscan_threshold_ * 2;
}

void Slam::set_factor_threshold(double factor_threshold) {
  this->factor_threshold_ = factor_threshold;
  if (keyscan_threshold_ * 2 > factor_threshold_)
    keyscan_threshold_ = factor_threshold_/2;
}

Pose2D Slam::pose() const {
  return pose_;
}

const std::vector<LaserScan> & Slam::scans() {
  return scans_;
}

#ifdef USE_ISAM
std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>> Slam::factors() {
  return graph_slam_.factors();
}
#endif

Pose2D Slam::EncoderToPose2D(double left, double right, double tread) {
  double theta = (right - left) / tread;
  double theta_2 = theta / 2.0;
  double arc = (right + left) / 2.0;
  double radius = arc / theta;
  double secant = 2 * sin(theta_2) * radius;
  if (theta == 0)
    secant = arc;
  double x = secant * cos(theta_2);
  double y = secant * sin(theta_2);

  return Pose2D(x, y, theta);
}

void Slam::UpdatePoseWithPose(Pose2D pose) {
  this->pose_ = pose * this->pose_;
}

void Slam::UpdatePoseWithEncoder(double left, double right, double tread) {
  pose_ = EncoderToPose2D(left, right, tread) * pose_;
  if (pose_update_callback)
    pose_update_callback(pose_);
}

void Slam::UpdatePoseWithLaserScan(const LaserScan &_scan) {
  LaserScan scan = _scan;
  scan.set_pose(pose_);

  // first scan
  if (scans_.empty()) {
    scans_.push_back(scan);
#ifdef USE_ISAM
    graph_slam_.AddPose2dFactor(0, pose_, 1);
#endif
    std::cout << "add key scan " << scans_.size() << ": "
      << pose_.ToJson() << std::endl;
    if (map_update_callback)
      map_update_callback();
    return;
  }

  // search for the closest scan
  LaserScan *closest_scan = &(scans_[0]);
  double min_dist = DBL_MAX;
  for (size_t i = 0; i < scans_.size(); i++) {
    double dist = (scans_[i].pose().pos() -
        scan.pose().pos()).norm();
    double delta_theta = fabs(scans_[i].pose().theta() -
        scan.pose().theta());;
    while (delta_theta < -M_PI) delta_theta += 2 * M_PI;
    while (delta_theta >  M_PI) delta_theta -= 2 * M_PI;
    delta_theta *= keyscan_threshold_ / (M_PI_4 * 3.0);

    dist = sqrt(dist * dist + delta_theta * delta_theta);
    if (dist < min_dist) {
      min_dist = dist;
      closest_scan = &(scans_[i]);
    }
  }


  if (min_dist < keyscan_threshold_) {
    // update pose
    double ratio;
    Pose2D pose_delta = closest_scan->ICP(scan, &ratio);
    pose_ = pose_delta * closest_scan->pose();
  } else {
    // add key scan
#ifdef USE_ISAM
    size_t constrain_count = 0;
    for (size_t i = 0; i < scans_.size(); i++) {
      double distance = (pose_.pos() - scans_[i].pose().pos()).norm();
      if (distance < factor_threshold_) {
        constrain_count++;
        double ratio;
        Pose2D pose_delta = scans_[i].ICP(scan, &ratio);
        graph_slam_.AddPose2dPose2dFactor(i, scans_.size(), pose_delta, ratio);
      }
    }
    if (constrain_count > 1)
      graph_slam_.Optimization();

    auto nodes = graph_slam_.nodes();
    for (size_t i = 0; i < nodes.size(); i++) {
      // update pose of node
      for (size_t j = 0; j < scans_.size(); j++) {
        if (j != nodes[i].first) continue;
        scans_[j].set_pose(nodes[i].second);
        break;
      }
      // add the new scan with pose
      if (nodes[i].first == scans_.size()) {
        pose_ = nodes[i].second;
        scan.set_pose(nodes[i].second);
        scans_.push_back(scan);
      }
    }
#else
    scans_.push_back(scan);
#endif
    std::cout << "add key scan " << scans_.size() << ": "
      << pose_.ToJson() << std::endl;

    if (map_update_callback)
      map_update_callback();
  }
  if (pose_update_callback)
    pose_update_callback(pose_);
}

void Slam::RegisterPoseUpdateCallback(std::function<void(Pose2D)> f) {
  pose_update_callback = f;
}
void Slam::RegisterMapUpdateCallback(std::function<void(void)> f) {
  map_update_callback = f;
}

