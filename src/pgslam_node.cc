/*
 * Copyright 2017 Yu Kunlin <yukunlin@mail.ustc.edu.cn>
 */
#include <Eigen/Eigen>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>

#include <pgslam/pgslam.h>

#include <string>

ros::Publisher node_pub;
ros::Publisher factor_pub;
ros::Publisher map_pub;
nav_msgs::OccupancyGrid global_map;
tf::TransformListener * plistener;

pgslam::Slam slam;

std::string map_frame  = "map";
std::string odom_frame = "odom";
std::string base_frame = "base_link";
double keyscan_threshold = 0.4;
double factor_threshold = 0.9;

void draw_graph() {
  visualization_msgs::Marker points;
  points.header.frame_id = map_frame;
  points.header.stamp = ros::Time::now();
  points.ns = "points_and_lines";
  points.action = visualization_msgs::Marker::ADD;
  points.pose.orientation.w = 1.0;
  points.id = 0;
  points.type = visualization_msgs::Marker::POINTS;
  points.scale.x = 0.05;
  points.scale.y = 0.05;
  points.color.g = 0.5f;
  points.color.a = 1.0;

  auto scans = slam.get_scans();
  scans[0].set_pose(pgslam::Pose2D());
  for (int i = 0; i < scans.size(); i++) {
    geometry_msgs::Point p;
    p.x = scans[i].get_pose().pos().x();
    p.y = scans[i].get_pose().pos().y();
    p.z = 0;
    points.points.push_back(p);
  }
  node_pub.publish(points);

  visualization_msgs::Marker line_list;
  line_list.header.frame_id = map_frame;
  line_list.header.stamp = ros::Time::now();
  line_list.ns = "points_and_lines";
  line_list.action = visualization_msgs::Marker::ADD;
  line_list.pose.orientation.w = 1.0;
  line_list.id = 1;
  line_list.type = visualization_msgs::Marker::LINE_LIST;
  line_list.scale.x = 0.01;
  line_list.color.r = 0.5;
  line_list.color.a = 1.0;

#ifdef USE_ISAM
  auto factors = slam.get_factors();
  for (size_t i = 0; i < factors.size(); i++) {
    geometry_msgs::Point first;
    first.x = factors[i].first.x();
    first.y = factors[i].first.y();
    line_list.points.push_back(first);
    geometry_msgs::Point second;
    second.x = factors[i].second.x();
    second.y = factors[i].second.y();
    line_list.points.push_back(second);
  }
  factor_pub.publish(line_list);
#endif
}

void draw_map() {
  double resolution = 0.05;
  double draw_range = 6.0;
  ros::param::get("~resolution", resolution);
  ros::param::get("~draw_range", draw_range);

  double max_x = 0.0;
  double min_x = 0.0;
  double max_y = 0.0;
  double min_y = 0.0;
  auto scans = slam.get_scans();
  for (size_t i = 0; i < scans.size(); i++) {
    if (max_x < scans[i].get_max_x_in_world())
      max_x = scans[i].get_max_x_in_world();
    if (min_x > scans[i].get_min_x_in_world())
      min_x = scans[i].get_min_x_in_world();
    if (max_y < scans[i].get_max_y_in_world())
      max_y = scans[i].get_max_y_in_world();
    if (min_y > scans[i].get_min_y_in_world())
      min_y = scans[i].get_min_y_in_world();
  }
  double max_x_i = static_cast<int>((max_x + 1.0) * 10) / 10.0;
  double min_x_i = static_cast<int>((min_x - 1.0) * 10) / 10.0;
  double max_y_i = static_cast<int>((max_y + 1.0) * 10) / 10.0;
  double min_y_i = static_cast<int>((min_y - 1.0) * 10) / 10.0;
  int width  = (max_x_i - min_x_i) / resolution;
  int height = (max_y_i - min_y_i) / resolution;

  // new an eigen map
  Eigen::MatrixXi emap(width, height);
  for (int i = 0; i < width; i++)
    for (int j = 0; j < height; j++)
      emap(i, j) = -1;

  // draw map into eigen map
  Eigen::Vector2d source(min_x_i, min_y_i);
  for (int i = 0; i < scans.size(); i++) {  // for every scan
    Eigen::Vector2d origin = scans[i].get_pose().pos();
    auto points = scans[i].get_points();
    for (int j = 1; j < points.size(); j++) {  // for every point
      Eigen::Vector2d v = points[j] - origin;
      if (v.norm() > draw_range)
        v = v.normalized() * (draw_range + resolution);
      int steps = v.norm() / resolution;
      Eigen::Vector2d step = v / steps / resolution;
      Eigen::Vector2d current = (origin - source) / resolution;
      for (int i = 0; i < steps; i++) {  // for every step
        if (emap(current.x(), current.y()) == -1) {
          emap(current.x(), current.y()) = 30;  // white
        } else {
          emap(current.x(), current.y()) =
            static_cast<int>(emap(current.x(), current.y()) * 0.8);
        }
        current += step;
      }
      if (v.norm() < draw_range) {
        if (emap(current.x(), current.y()) == -1) {
          emap(current.x(), current.y()) = 100;  // black
        } else {
          emap(current.x(), current.y()) =
            static_cast<int>(emap(current.x(), current.y()) * 0.2);
          emap(current.x(), current.y()) += 100 * 0.8;
        }
      }
    }
  }

  // copy eigen map to ros map
  global_map.header.stamp = ros::Time::now();
  global_map.header.frame_id = map_frame;
  global_map.info.resolution = resolution;
  global_map.info.map_load_time = ros::Time::now();

  global_map.info.width = width;
  global_map.info.height = height;

  global_map.info.origin.position.x = min_x_i;
  global_map.info.origin.position.y = min_y_i;
  global_map.info.origin.position.z = 0;
  global_map.info.origin.orientation.w = 1.0;

  global_map.data.resize(width * height);

  for (int i = 0; i < width; i++)
    for (int j = 0; j < height; j++)
      global_map.data[j * width + i] = emap(i, j);

  // publish
  map_pub.publish(global_map);
}

pgslam::Pose2D
ListenPose2D(std::string target_frame, std::string source_frame) {
  pgslam::Pose2D pose;
  // listen
  tf::StampedTransform transform;
  std::string s;
  try {
    plistener->waitForTransform(target_frame,
        source_frame, ros::Time(0), ros::Duration(2.0),
        ros::Duration(0.01), &s);
    plistener->lookupTransform(target_frame,
        source_frame, ros::Time(0), transform);
  } catch (tf::TransformException ex) {
    ROS_ERROR("slam lookupTransform error: %s %s", ex.what(), s.c_str());
    return pose;
  }

  // calc pose
  pose.set_x(transform.getOrigin().x());
  pose.set_y(transform.getOrigin().y());

  double roll, pitch, yaw;
  transform.getBasis().getRPY(roll, pitch, yaw);

  pose.set_theta(yaw);

  return pose;
}

void BroadcastMapAndGraph() {
  draw_graph();
  draw_map();
}

void BroadcastPose() {
  pgslam::Pose2D map_pose = slam.get_pose();
  pgslam::Pose2D odom_pose = ListenPose2D(odom_frame, base_frame);
  pgslam::Pose2D delta = map_pose + odom_pose.inverse();

  tf::StampedTransform transform;
  transform.setOrigin(tf::Vector3(delta.pos().x(), delta.pos().y(), 0.0));
  tf::Quaternion q;
  q.setRPY(0, 0, delta.theta());
  transform.setRotation(q);

  static tf::TransformBroadcaster br;
  br.sendTransform(tf::StampedTransform(transform,
        ros::Time::now(), map_frame, odom_frame));
}

pgslam::LaserScan
RosLaserScan_T_PGSlamLaserScan(const sensor_msgs::LaserScan& msg) {
  std::vector<pgslam::Echo> echos;
  size_t i = 0;
  for (double angle = msg.angle_min; angle <= msg.angle_max;
      angle+=msg.angle_increment, i++) {
    echos.push_back(pgslam::Echo(msg.ranges[i], angle, msg.intensities[i], 0));
  }
  return pgslam::LaserScan(echos);
}

void scanCallback(const sensor_msgs::LaserScan& msg) {
  static pgslam::Pose2D odom_old;
  pgslam::Pose2D odom_new = ListenPose2D(odom_frame, base_frame);
  pgslam::Pose2D odom_delta = odom_new - odom_old;
  odom_old = odom_new;
  slam.UpdatePoseWithPose(odom_delta);
  slam.UpdatePoseWithLaserScan(RosLaserScan_T_PGSlamLaserScan(msg));
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "pgslam");
  ros::NodeHandle node;

  ros::Subscriber scan_sub = node.subscribe("/scan", 10, scanCallback);

  node_pub   = node.advertise<visualization_msgs::Marker>("graph_node", 1);
  factor_pub = node.advertise<visualization_msgs::Marker>("graph_factor", 1);
  map_pub    = node.advertise<nav_msgs::OccupancyGrid>("/map", 1);

  slam.RegisterMapUpdateCallback(BroadcastMapAndGraph);
  slam.RegisterPoseUpdateCallback(BroadcastPose);

  plistener = new tf::TransformListener();

  ros::param::get("~map_frame", map_frame);
  ros::param::get("~odom_frame", odom_frame);
  ros::param::get("~base_frame", base_frame);
  ros::param::get("~keyscan_threshold", keyscan_threshold);
  ros::param::get("~factor_threshold", factor_threshold);

  slam.set_keyscan_threshold(keyscan_threshold);
  slam.set_factor_threshold(factor_threshold);

  ros::spin();

  return 0;
}

