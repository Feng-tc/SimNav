#include "ease_planner/ease_planner.h"

#include <cmath>

#include <ros/package.h>

namespace ease_planner_ns {

EasePlanner::EasePlanner(ros::NodeHandle& nh, ros::NodeHandle& private_nh)
    : nh_(nh), private_nh_(private_nh) {
  ReadParameters();
}

void EasePlanner::initialize() {
  start_exploration_sub_ = nh_.subscribe<std_msgs::Bool>(
      sub_start_exploration_topic_, 5, &EasePlanner::StartExplorationCallback, this);
  state_estimation_sub_ = nh_.subscribe<nav_msgs::Odometry>(
      sub_state_estimation_topic_, 5, &EasePlanner::StateEstimationCallback, this);

  waypoint_pub_ = nh_.advertise<geometry_msgs::PointStamped>(pub_waypoint_topic_, 2);
  exploration_finish_pub_ =
      nh_.advertise<std_msgs::Bool>(pub_exploration_finish_topic_, 2, true);
  exploring_phase_pub_ =
      nh_.advertise<std_msgs::Int32>(pub_exploring_phase_topic_, 1, true);
  map_clearing_pub_ = nh_.advertise<std_msgs::Float32>("/map_clearing", 1);
  waypoint_marker_pub_ =
      nh_.advertise<visualization_msgs::MarkerArray>("/ease_planner/waypoints", 1, true);

  PublishExploringPhase();
  execution_timer_ =
      nh_.createTimer(ros::Duration(0.1), &EasePlanner::ExecuteTimer, this);
  ROS_INFO("ease_planner initialized (auto_start=%s, arrival_dist=%.2f)",
           kAutoStart_ ? "true" : "false", kPhaseArrivalDist_);
}

void EasePlanner::ReadParameters() {
  private_nh_.param<std::string>("sub_start_exploration_topic_",
                                 sub_start_exploration_topic_, "/start_exploration");
  private_nh_.param<std::string>("sub_state_estimation_topic_",
                                 sub_state_estimation_topic_, "/Odometry_gazebo");
  private_nh_.param<std::string>("pub_waypoint_topic_", pub_waypoint_topic_, "/way_point");
  private_nh_.param<std::string>("pub_exploration_finish_topic_",
                                 pub_exploration_finish_topic_, "exploration_finish");
  private_nh_.param<std::string>("pub_exploring_phase_topic_",
                                 pub_exploring_phase_topic_, "/exploring_phase");

  private_nh_.param("kAutoStart", kAutoStart_, true);
  private_nh_.param("kUsePhase1Door", kUsePhase1Door_, true);
  private_nh_.param("kPhaseArrivalDist", kPhaseArrivalDist_, 1.0);
  private_nh_.param("kMapClearingDist", kMapClearingDist_, 8.0);
  private_nh_.param<std::string>("kMainEntranceDoorId", kMainEntranceDoorId_, "main_entrance");
  private_nh_.param<std::string>("kElevatorId", kElevatorId_, "elevator_main");
  private_nh_.param("kElevatorServedFloorCount", kElevatorServedFloorCount_, 3);
  if (kElevatorServedFloorCount_ < 1) {
    kElevatorServedFloorCount_ = 1;
  }

  LoadPointParam(private_nh_, "kPhase1Waypoint", &phase1_waypoint_, 0.0, 2.5, 0.6);
  LoadFloorWaypoints(private_nh_);

  LoadPointParam(private_nh_, "kElevator1_1Waypoint", &elevator_trips_[0].outside,
                 0.0, 2.5, 0.6);
  LoadPointParam(private_nh_, "kElevator1_2Waypoint", &elevator_trips_[0].inside,
                 2.2, 2.5, 0.6);
  LoadPointParam(private_nh_, "kElevator1_3Waypoint", &elevator_trips_[0].exit,
                 0.0, 2.5, 3.2);
  private_nh_.param("kElevator1TargetFloor", elevator_trips_[0].call_to_floor, 1);
  private_nh_.param("kElevator1WaitFloor", elevator_trips_[0].wait_at_floor, 0);

  LoadPointParam(private_nh_, "kElevator2_1Waypoint", &elevator_trips_[1].outside,
                 0.0, 2.5, 3.2);
  LoadPointParam(private_nh_, "kElevator2_2Waypoint", &elevator_trips_[1].inside,
                 2.2, 2.5, 3.2);
  LoadPointParam(private_nh_, "kElevator2_3Waypoint", &elevator_trips_[1].exit,
                 0.0, 2.5, 5.8);
  private_nh_.param("kElevator2TargetFloor", elevator_trips_[1].call_to_floor, 2);
  private_nh_.param("kElevator2WaitFloor", elevator_trips_[1].wait_at_floor, 1);

  ROS_INFO("ease_planner profile '%s': 1F=%zu 2F=%zu 3F=%zu",
           waypoint_profile_.c_str(), floor_waypoints_[0].size(),
           floor_waypoints_[1].size(), floor_waypoints_[2].size());
}

void EasePlanner::LoadWaypointListFromYaml(
    const YAML::Node& node, std::vector<geometry_msgs::Point>* out) {
  out->clear();
  if (!node || !node.IsSequence()) {
    return;
  }
  for (const auto& item : node) {
    if (!item.IsSequence() || item.size() < 3) {
      continue;
    }
    geometry_msgs::Point point;
    point.x = item[0].as<double>();
    point.y = item[1].as<double>();
    point.z = item[2].as<double>();
    out->push_back(point);
  }
}

void EasePlanner::LoadFloorWaypoints(const ros::NodeHandle& nh) {
  nh.param("kWaypointProfile", waypoint_profile_, std::string("full_tour"));

  const std::string profile_path = ros::package::getPath("ease_planner") +
                                   "/config/profiles/" + waypoint_profile_ +
                                   ".yaml";
  try {
    const YAML::Node root = YAML::LoadFile(profile_path);
    LoadWaypointListFromYaml(root["kFloor1Waypoints"], &floor_waypoints_[0]);
    LoadWaypointListFromYaml(root["kFloor2Waypoints"], &floor_waypoints_[1]);
    LoadWaypointListFromYaml(root["kFloor3Waypoints"], &floor_waypoints_[2]);
    ROS_INFO("Loaded waypoint profile '%s' from %s", waypoint_profile_.c_str(),
             profile_path.c_str());
    return;
  } catch (const YAML::Exception& e) {
    ROS_ERROR("Failed to load waypoint profile '%s' from %s: %s",
              waypoint_profile_.c_str(), profile_path.c_str(), e.what());
  }

  ROS_WARN("Falling back to kFloor*Waypoints from parameter server");
  LoadWaypointList(nh, "kFloor1Waypoints", &floor_waypoints_[0]);
  LoadWaypointList(nh, "kFloor2Waypoints", &floor_waypoints_[1]);
  LoadWaypointList(nh, "kFloor3Waypoints", &floor_waypoints_[2]);
}

double EasePlanner::XmlRpcToDouble(const XmlRpc::XmlRpcValue& value) {
  if (value.getType() == XmlRpc::XmlRpcValue::TypeInt) {
    return static_cast<int>(value);
  }
  if (value.getType() == XmlRpc::XmlRpcValue::TypeDouble) {
    return static_cast<double>(value);
  }
  return 0.0;
}

bool EasePlanner::LoadPointParam(const ros::NodeHandle& nh, const std::string& key,
                                 geometry_msgs::Point* point, double dx, double dy,
                                 double dz) {
  std::vector<double> xyz;
  if (nh.getParam(key, xyz) && xyz.size() >= 3) {
    point->x = xyz[0];
    point->y = xyz[1];
    point->z = xyz[2];
    return true;
  }
  point->x = dx;
  point->y = dy;
  point->z = dz;
  if (nh.hasParam(key)) {
    ROS_WARN("%s must have at least 3 elements [x, y, z]; using fallback", key.c_str());
  }
  return false;
}

void EasePlanner::LoadWaypointList(const ros::NodeHandle& nh, const std::string& key,
                                   std::vector<geometry_msgs::Point>* out) {
  out->clear();
  XmlRpc::XmlRpcValue list;
  if (!nh.getParam(key, list)) {
    ROS_INFO("Param %s not set, using empty waypoint list", key.c_str());
    return;
  }
  if (list.getType() != XmlRpc::XmlRpcValue::TypeArray) {
    ROS_ERROR("%s must be a list of [x, y, z]", key.c_str());
    return;
  }
  for (int i = 0; i < list.size(); ++i) {
    if (list[i].getType() != XmlRpc::XmlRpcValue::TypeArray || list[i].size() < 3) {
      ROS_ERROR("%s[%d] must be [x, y, z]", key.c_str(), i);
      continue;
    }
    geometry_msgs::Point point;
    point.x = XmlRpcToDouble(list[i][0]);
    point.y = XmlRpcToDouble(list[i][1]);
    point.z = XmlRpcToDouble(list[i][2]);
    out->push_back(point);
  }
}

void EasePlanner::StartExplorationCallback(const std_msgs::Bool::ConstPtr& msg) {
  start_exploration_ = msg->data;
}

void EasePlanner::StateEstimationCallback(const nav_msgs::Odometry::ConstPtr& msg) {
  robot_position_ = msg->pose.pose.position;
  has_robot_position_ = true;
}

void EasePlanner::ExecuteTimer(const ros::TimerEvent& /*event*/) {
  Execute();
}

void EasePlanner::PublishExploringPhase() {
  std_msgs::Int32 msg;
  msg.data = exploring_phase_;
  exploring_phase_pub_.publish(msg);
}

void EasePlanner::PublishWaypoint(const geometry_msgs::Point& point) {
  geometry_msgs::PointStamped waypoint;
  waypoint.header.frame_id = "map";
  waypoint.header.stamp = ros::Time::now();
  waypoint.point = point;
  waypoint_pub_.publish(waypoint);
}

void EasePlanner::PublishPhase1Waypoint() {
  if (!phase1_pointcloud_reset_) {
    std_msgs::Float32 clearing_msg;
    clearing_msg.data = static_cast<float>(kMapClearingDist_);
    map_clearing_pub_.publish(clearing_msg);
    ROS_INFO("Phase1 first waypoint: published /map_clearing (dist=%.1f m)",
             kMapClearingDist_);
    phase1_pointcloud_reset_ = true;
  }
  PublishWaypoint(phase1_waypoint_);
}

void EasePlanner::PublishVisualization() {
  visualization_msgs::MarkerArray markers;
  visualization_msgs::Marker delete_all;
  delete_all.header.frame_id = "map";
  delete_all.header.stamp = ros::Time::now();
  delete_all.ns = "ease_waypoints";
  delete_all.action = visualization_msgs::Marker::DELETEALL;
  markers.markers.push_back(delete_all);

  int id = 0;
  for (int floor = 0; floor < 3; ++floor) {
    visualization_msgs::Marker spheres;
    spheres.header.frame_id = "map";
    spheres.header.stamp = ros::Time::now();
    spheres.ns = "ease_waypoints";
    spheres.id = id++;
    spheres.type = visualization_msgs::Marker::SPHERE_LIST;
    spheres.action = visualization_msgs::Marker::ADD;
    spheres.scale.x = 0.35;
    spheres.scale.y = 0.35;
    spheres.scale.z = 0.35;
    spheres.pose.orientation.w = 1.0;
    if (floor == 0) {
      spheres.color.r = 0.2;
      spheres.color.g = 0.8;
      spheres.color.b = 0.2;
    } else if (floor == 1) {
      spheres.color.r = 0.2;
      spheres.color.g = 0.5;
      spheres.color.b = 1.0;
    } else {
      spheres.color.r = 0.9;
      spheres.color.g = 0.6;
      spheres.color.b = 0.1;
    }
    spheres.color.a = 0.8;
    for (const auto& point : floor_waypoints_[floor]) {
      spheres.points.push_back(point);
    }
    if (!spheres.points.empty()) {
      markers.markers.push_back(spheres);
    }
  }

  geometry_msgs::Point current;
  bool has_current = false;
  if (exploring_phase_ == 1) {
    current = phase1_waypoint_;
    has_current = true;
  } else if (exploring_phase_ == 2 || exploring_phase_ == 4 || exploring_phase_ == 6) {
    if (waypoint_index_ < static_cast<int>(floor_waypoints_[floor_index_].size())) {
      current = floor_waypoints_[floor_index_][waypoint_index_];
      has_current = true;
    }
  } else if (exploring_phase_ == 3 || exploring_phase_ == 5) {
    current = elevator_waypoint_;
    has_current = true;
  }

  if (has_current) {
    visualization_msgs::Marker highlight;
    highlight.header.frame_id = "map";
    highlight.header.stamp = ros::Time::now();
    highlight.ns = "ease_waypoints";
    highlight.id = 100;
    highlight.type = visualization_msgs::Marker::SPHERE;
    highlight.action = visualization_msgs::Marker::ADD;
    highlight.pose.position = current;
    highlight.pose.orientation.w = 1.0;
    highlight.scale.x = 0.6;
    highlight.scale.y = 0.6;
    highlight.scale.z = 0.6;
    highlight.color.r = 1.0;
    highlight.color.g = 0.1;
    highlight.color.b = 0.1;
    highlight.color.a = 0.95;
    markers.markers.push_back(highlight);
  }

  waypoint_marker_pub_.publish(markers);
}

void EasePlanner::PublishFinished() {
  std_msgs::Bool msg;
  msg.data = true;
  exploration_finish_pub_.publish(msg);
}

bool EasePlanner::ArrivedXY(const geometry_msgs::Point& target) const {
  const double dx = robot_position_.x - target.x;
  const double dy = robot_position_.y - target.y;
  return std::hypot(dx, dy) < kPhaseArrivalDist_;
}

bool EasePlanner::RemainingFloorsHaveWaypoints(int from_floor_index) const {
  for (int i = from_floor_index; i < 3; ++i) {
    if (!floor_waypoints_[i].empty()) {
      return true;
    }
  }
  return false;
}

bool EasePlanner::WillUseElevator() const {
  return RemainingFloorsHaveWaypoints(1);
}

void EasePlanner::MarkFinished() {
  if (exploration_finished_) {
    return;
  }
  exploration_finished_ = true;
  PublishFinished();
  ROS_INFO("ease_planner: tour finished");
}

void EasePlanner::StartPhase1() {
  exploring_phase_ = 1;
  PublishExploringPhase();
  if (!kUsePhase1Door_) {
    ROS_INFO("Phase1 door disabled, skipping to floor-1 waypoints");
    StartWaypointTour(0);
    return;
  }
  phase1_door_opened_ = false;
  if (TryOpenPhase1Entry(30.0)) {
    phase1_door_opened_ = true;
    PublishPhase1Waypoint();
    ROS_INFO("exploringPhase=1, entry open%s, target [%.2f, %.2f, %.2f]",
             WillUseElevator() ? " (main door + elevator prep)" : "",
             phase1_waypoint_.x, phase1_waypoint_.y, phase1_waypoint_.z);
  } else {
    ROS_WARN("exploringPhase=1, waiting for main entrance door%s before moving",
             WillUseElevator() ? " and elevator setup" : "");
  }
}

void EasePlanner::HandlePhase1() {
  if (!phase1_door_opened_) {
    if (TryOpenPhase1Entry(5.0)) {
      phase1_door_opened_ = true;
    } else {
      ROS_WARN_THROTTLE(2.0, "Phase1: waiting for main entrance door%s",
                        WillUseElevator() ? " and elevator setup" : "");
      return;
    }
  }
  PublishPhase1Waypoint();
  if (ArrivedXY(phase1_waypoint_)) {
    StartWaypointTour(0);
  }
}

void EasePlanner::StartWaypointTour(int floor_index) {
  floor_index_ = floor_index;
  waypoint_index_ = 0;
  exploring_phase_ = 2 + floor_index * 2;
  PublishExploringPhase();
  if (floor_waypoints_[floor_index].empty()) {
    ROS_INFO("Phase%d: floor %d has no waypoints, skipping tour", exploring_phase_,
             floor_index + 1);
    FinishFloor(floor_index);
    return;
  }
  const auto& first = floor_waypoints_[floor_index][0];
  ROS_INFO("exploringPhase -> %d, floor %d waypoint 1/%zu [%.2f, %.2f, %.2f]",
           exploring_phase_, floor_index + 1, floor_waypoints_[floor_index].size(),
           first.x, first.y, first.z);
  PublishWaypoint(first);
}

void EasePlanner::HandleWaypointTour() {
  if (waypoint_index_ >= static_cast<int>(floor_waypoints_[floor_index_].size())) {
    FinishFloor(floor_index_);
    return;
  }
  const auto& target = floor_waypoints_[floor_index_][waypoint_index_];
  PublishWaypoint(target);
  if (!ArrivedXY(target)) {
    return;
  }
  ROS_INFO("Phase%d: arrived at floor %d waypoint %d/%zu", exploring_phase_,
           floor_index_ + 1, waypoint_index_ + 1, floor_waypoints_[floor_index_].size());
  ++waypoint_index_;
  if (waypoint_index_ >= static_cast<int>(floor_waypoints_[floor_index_].size())) {
    FinishFloor(floor_index_);
    return;
  }
  const auto& next = floor_waypoints_[floor_index_][waypoint_index_];
  ROS_INFO("Phase%d: next waypoint %d/%zu [%.2f, %.2f, %.2f]", exploring_phase_,
           waypoint_index_ + 1, floor_waypoints_[floor_index_].size(), next.x, next.y,
           next.z);
  PublishWaypoint(next);
}

void EasePlanner::FinishFloor(int floor_index) {
  if (floor_index >= 2 || !RemainingFloorsHaveWaypoints(floor_index + 1)) {
    MarkFinished();
    return;
  }
  StartElevator(floor_index);
}

void EasePlanner::StartElevator(int trip_index) {
  elevator_trip_index_ = trip_index;
  elevator_ride_requested_ = false;
  exploring_phase_ = 3 + trip_index * 2;
  PublishExploringPhase();

  const ElevatorTrip& trip = elevator_trips_[trip_index];
  elevator_waypoint_ = trip.outside;
  if (elevator_ready_[trip_index]) {
    elevator_step_ = 1;
    ROS_INFO(
        "exploringPhase -> %d (elevator ready), step 1 wp1 [%.2f, %.2f, %.2f], "
        "wp2 [%.2f, %.2f, %.2f], wp3 [%.2f, %.2f, %.2f], target floor %d",
        exploring_phase_, trip.outside.x, trip.outside.y, trip.outside.z, trip.inside.x,
        trip.inside.y, trip.inside.z, trip.exit.x, trip.exit.y, trip.exit.z,
        trip.call_to_floor);
    PublishWaypoint(elevator_waypoint_);
  } else {
    elevator_step_ = 0;
    ROS_INFO(
        "exploringPhase -> %d, wp1 [%.2f, %.2f, %.2f], wp2 [%.2f, %.2f, %.2f], "
        "wp3 [%.2f, %.2f, %.2f], target floor %d",
        exploring_phase_, trip.outside.x, trip.outside.y, trip.outside.z, trip.inside.x,
        trip.inside.y, trip.inside.z, trip.exit.x, trip.exit.y, trip.exit.z,
        trip.call_to_floor);
  }
}

void EasePlanner::HandleElevator() {
  const ElevatorTrip& trip = elevator_trips_[elevator_trip_index_];
  switch (elevator_step_) {
    case 0: {
      if (elevator_ready_[elevator_trip_index_]) {
        elevator_step_ = 1;
        elevator_waypoint_ = trip.outside;
        ROS_INFO("Phase%d step 1: approach outside waypoint [%.2f, %.2f, %.2f]",
                 exploring_phase_, elevator_waypoint_.x, elevator_waypoint_.y,
                 elevator_waypoint_.z);
        PublishWaypoint(elevator_waypoint_);
        break;
      }
      if (TrySetupElevator(elevator_trip_index_, 5.0, 30.0)) {
        elevator_step_ = 1;
        elevator_waypoint_ = trip.outside;
        ROS_INFO("Phase%d step 1 (fallback setup): approach outside waypoint [%.2f, %.2f, %.2f]",
                 exploring_phase_, elevator_waypoint_.x, elevator_waypoint_.y,
                 elevator_waypoint_.z);
        PublishWaypoint(elevator_waypoint_);
      } else {
        ROS_WARN_THROTTLE(2.0,
                          "Phase%d step 0 (fallback): waiting for elevator setup "
                          "(all doors + floor %d)",
                          exploring_phase_, trip.wait_at_floor);
      }
      break;
    }
    case 1: {
      elevator_waypoint_ = trip.outside;
      PublishWaypoint(elevator_waypoint_);
      if (ArrivedXY(elevator_waypoint_)) {
        elevator_step_ = 2;
        elevator_waypoint_ = trip.inside;
        ROS_INFO("Phase%d step 2: enter car waypoint [%.2f, %.2f, %.2f]",
                 exploring_phase_, elevator_waypoint_.x, elevator_waypoint_.y,
                 elevator_waypoint_.z);
      }
      break;
    }
    case 2: {
      elevator_waypoint_ = trip.inside;
      PublishWaypoint(elevator_waypoint_);
      if (ArrivedXY(elevator_waypoint_)) {
        elevator_step_ = 3;
        ROS_INFO("Phase%d step 3: calling elevator to floor %d", exploring_phase_,
                 trip.call_to_floor);
      }
      break;
    }
    case 3: {
      PublishWaypoint(elevator_waypoint_);
      if (!elevator_ride_requested_) {
        if (CallElevatorToFloor(trip.call_to_floor, true, 30.0)) {
          elevator_ride_requested_ = true;
          elevator_step_ = 4;
          ROS_INFO("Phase%d step 3: elevator called to floor %d, waiting for arrival",
                   exploring_phase_, trip.call_to_floor);
        } else {
          ROS_WARN_THROTTLE(2.0, "Phase%d step 3: waiting to call elevator to floor %d",
                            exploring_phase_, trip.call_to_floor);
        }
      }
      break;
    }
    case 4: {
      const double z_threshold = trip.exit.z - 0.5;
      if (robot_position_.z >= z_threshold) {
        elevator_step_ = 5;
        elevator_waypoint_ = trip.exit;
        ROS_INFO("Phase%d step 4: elevator arrived (z=%.2f), goto [%.2f, %.2f, %.2f]",
                 exploring_phase_, robot_position_.z, elevator_waypoint_.x,
                 elevator_waypoint_.y, elevator_waypoint_.z);
        PublishWaypoint(elevator_waypoint_);
      } else {
        ROS_WARN_THROTTLE(2.0,
                          "Phase%d step 4: waiting for elevator (robot z=%.2f, need >= %.2f)",
                          exploring_phase_, robot_position_.z, z_threshold);
      }
      break;
    }
    case 5: {
      elevator_waypoint_ = trip.exit;
      PublishWaypoint(elevator_waypoint_);
      if (ArrivedXY(elevator_waypoint_)) {
        ROS_INFO("Phase%d: arrived at exit waypoint [%.2f, %.2f, %.2f]", exploring_phase_,
                 elevator_waypoint_.x, elevator_waypoint_.y, elevator_waypoint_.z);
        StartWaypointTour(elevator_trip_index_ + 1);
      }
      break;
    }
    default:
      break;
  }
}

void EasePlanner::Execute() {
  if (!kAutoStart_ && !start_exploration_) {
    ROS_INFO_THROTTLE(2.0, "Waiting for start signal");
    return;
  }
  PublishExploringPhase();
  PublishVisualization();

  if (!initialized_) {
    initialized_ = true;
    StartPhase1();
    return;
  }
  if (exploration_finished_) {
    PublishFinished();
    return;
  }
  if (!has_robot_position_) {
    ROS_WARN_THROTTLE(2.0, "Waiting for odometry on %s",
                      sub_state_estimation_topic_.c_str());
    return;
  }

  ROS_INFO_THROTTLE(1.0,
                    "ease_planner | phase=%d pos=(%.2f,%.2f,%.2f) floor=%d wp=%d elev_step=%d",
                    exploring_phase_, robot_position_.x, robot_position_.y, robot_position_.z,
                    floor_index_ + 1, waypoint_index_ + 1, elevator_step_);

  switch (exploring_phase_) {
    case 1:
      HandlePhase1();
      break;
    case 2:
    case 4:
    case 6:
      HandleWaypointTour();
      break;
    case 3:
    case 5:
      HandleElevator();
      break;
    default:
      break;
  }
}

bool EasePlanner::SetDoorById(const std::string& door_id, bool open,
                              double service_wait_timeout) {
  if (!set_door_state_client_.exists()) {
    if (!ros::service::waitForService("/set_door_state", service_wait_timeout)) {
      ROS_WARN_THROTTLE(5.0, "set_door_state service not available");
      return false;
    }
    set_door_state_client_ =
        nh_.serviceClient<building_generator_interfaces::SetDoorState>("/set_door_state");
  }

  building_generator_interfaces::SetDoorState srv;
  srv.request.door_id = door_id;
  srv.request.open = open;
  if (!set_door_state_client_.call(srv)) {
    ROS_WARN("set_door_state service call failed for \"%s\"", door_id.c_str());
    return false;
  }
  if (srv.response.accepted) {
    ROS_INFO("Door \"%s\" %s", door_id.c_str(), open ? "opened" : "closed");
    return true;
  }
  ROS_WARN("Door \"%s\" %s rejected: %s", door_id.c_str(), open ? "open" : "close",
           srv.response.message.c_str());
  return false;
}

bool EasePlanner::SetMainEntranceDoor(bool open, double service_wait_timeout) {
  return SetDoorById(kMainEntranceDoorId_, open, service_wait_timeout);
}

bool EasePlanner::OpenAllElevatorDoors(double service_wait_timeout) {
  for (int i = 0; i < kElevatorServedFloorCount_; ++i) {
    const std::string door_id = "elevator_floor_" + std::to_string(i);
    if (!SetDoorById(door_id, true, service_wait_timeout)) {
      return false;
    }
  }
  ROS_INFO("Opened all %d elevator doors (elevator_floor_0 .. elevator_floor_%d)",
           kElevatorServedFloorCount_, kElevatorServedFloorCount_ - 1);
  return true;
}

bool EasePlanner::CallElevatorToFloor(int floor, bool open_doors,
                                      double service_wait_timeout) {
  if (!call_elevator_client_.exists()) {
    if (!ros::service::waitForService("/call_elevator", service_wait_timeout)) {
      ROS_WARN_THROTTLE(5.0, "call_elevator service not available");
      return false;
    }
    call_elevator_client_ =
        nh_.serviceClient<building_generator_interfaces::CallElevator>("/call_elevator");
  }

  building_generator_interfaces::CallElevator srv;
  srv.request.elevator_id = kElevatorId_;
  srv.request.target_floor = floor;
  srv.request.open_doors = open_doors;
  if (!call_elevator_client_.call(srv)) {
    ROS_WARN("call_elevator service call failed");
    return false;
  }
  if (srv.response.accepted) {
    ROS_INFO("Elevator \"%s\" called to floor %d (current_floor=%d, state=%s)",
             kElevatorId_.c_str(), floor, srv.response.current_floor,
             srv.response.state.c_str());
    return true;
  }
  ROS_WARN("Elevator \"%s\" call to floor %d rejected: %s", kElevatorId_.c_str(), floor,
           srv.response.message.c_str());
  return false;
}

bool EasePlanner::TrySetupElevator(int trip_index, double door_wait, double elevator_wait) {
  if (elevator_ready_[trip_index]) {
    return true;
  }
  const bool doors_ok = OpenAllElevatorDoors(door_wait);
  const bool elevator_ok =
      CallElevatorToFloor(elevator_trips_[trip_index].wait_at_floor, true, elevator_wait);
  if (doors_ok && elevator_ok) {
    elevator_ready_[trip_index] = true;
  }
  return elevator_ready_[trip_index];
}

bool EasePlanner::TryOpenPhase1Entry(double service_wait_timeout) {
  if (!SetMainEntranceDoor(true, service_wait_timeout)) {
    return false;
  }
  if (WillUseElevator() && !TrySetupElevator(0, 5.0, 30.0)) {
    return false;
  }
  return true;
}

}  // namespace ease_planner_ns
