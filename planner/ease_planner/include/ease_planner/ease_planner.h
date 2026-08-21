#pragma once

#include <string>
#include <vector>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <visualization_msgs/MarkerArray.h>
#include <xmlrpcpp/XmlRpcValue.h>

#include <building_generator_interfaces/CallElevator.h>
#include <building_generator_interfaces/SetDoorState.h>

namespace ease_planner_ns {

struct ElevatorTrip {
  geometry_msgs::Point outside;
  geometry_msgs::Point inside;
  geometry_msgs::Point exit;
  int call_to_floor = 1;
  int wait_at_floor = 0;
};

class EasePlanner {
 public:
  EasePlanner(ros::NodeHandle& nh, ros::NodeHandle& private_nh);
  void initialize();

 private:
  void ReadParameters();
  void StartExplorationCallback(const std_msgs::Bool::ConstPtr& msg);
  void StateEstimationCallback(const nav_msgs::Odometry::ConstPtr& msg);
  void ExecuteTimer(const ros::TimerEvent& event);
  void Execute();

  void PublishExploringPhase();
  void PublishWaypoint(const geometry_msgs::Point& point);
  void PublishPhase1Waypoint();
  void PublishVisualization();
  void PublishFinished();

  void StartPhase1();
  void HandlePhase1();
  void StartWaypointTour(int floor_index);
  void HandleWaypointTour();
  void FinishFloor(int floor_index);
  void StartElevator(int trip_index);
  void HandleElevator();
  void MarkFinished();

  bool RemainingFloorsHaveWaypoints(int from_floor_index) const;
  bool WillUseElevator() const;
  bool ArrivedXY(const geometry_msgs::Point& target) const;

  bool SetDoorById(const std::string& door_id, bool open, double service_wait_timeout);
  bool SetMainEntranceDoor(bool open, double service_wait_timeout);
  bool OpenAllElevatorDoors(double service_wait_timeout);
  bool CallElevatorToFloor(int floor, bool open_doors, double service_wait_timeout);
  bool TrySetupElevator(int trip_index, double door_wait, double elevator_wait);
  bool TryOpenPhase1Entry(double service_wait_timeout);

  static double XmlRpcToDouble(const XmlRpc::XmlRpcValue& value);
  static bool LoadPointParam(const ros::NodeHandle& nh, const std::string& key,
                             geometry_msgs::Point* point, double dx, double dy, double dz);
  static void LoadWaypointList(const ros::NodeHandle& nh, const std::string& key,
                               std::vector<geometry_msgs::Point>* out);

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  std::string sub_start_exploration_topic_;
  std::string sub_state_estimation_topic_;
  std::string pub_waypoint_topic_;
  std::string pub_exploration_finish_topic_;
  std::string pub_exploring_phase_topic_;

  bool kAutoStart_ = true;
  bool kUsePhase1Door_ = true;
  double kPhaseArrivalDist_ = 1.0;
  double kMapClearingDist_ = 8.0;
  std::string kMainEntranceDoorId_ = "main_entrance";
  std::string kElevatorId_ = "elevator_main";
  int kElevatorServedFloorCount_ = 3;

  geometry_msgs::Point phase1_waypoint_;
  std::vector<geometry_msgs::Point> floor_waypoints_[3];
  ElevatorTrip elevator_trips_[2];

  int exploring_phase_ = 1;
  int floor_index_ = 0;
  int waypoint_index_ = 0;
  int elevator_trip_index_ = 0;
  int elevator_step_ = 0;
  bool elevator_ready_[2] = {false, false};
  bool elevator_ride_requested_ = false;
  geometry_msgs::Point elevator_waypoint_;

  bool initialized_ = false;
  bool start_exploration_ = false;
  bool exploration_finished_ = false;
  bool phase1_door_opened_ = false;
  bool phase1_pointcloud_reset_ = false;
  bool has_robot_position_ = false;
  geometry_msgs::Point robot_position_;

  ros::Timer execution_timer_;
  ros::Subscriber start_exploration_sub_;
  ros::Subscriber state_estimation_sub_;
  ros::Publisher waypoint_pub_;
  ros::Publisher exploration_finish_pub_;
  ros::Publisher exploring_phase_pub_;
  ros::Publisher map_clearing_pub_;
  ros::Publisher waypoint_marker_pub_;
  ros::ServiceClient set_door_state_client_;
  ros::ServiceClient call_elevator_client_;
};

}  // namespace ease_planner_ns
