// Copyright 2020 Tier IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "scenario_selector/scenario_selector_node.hpp"

#include <lanelet2_extension/utility/message_conversion.hpp>
#include <lanelet2_extension/utility/query.hpp>

#include <lanelet2_core/geometry/BoundingBox.h>
#include <lanelet2_core/geometry/Lanelet.h>
#include <lanelet2_core/geometry/LineString.h>
#include <lanelet2_core/geometry/Point.h>
#include <lanelet2_core/geometry/Polygon.h>

#include <deque>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace
{
template <class T>
void onData(const T & data, T * buffer)
{
  *buffer = data;
}

std::shared_ptr<lanelet::ConstPolygon3d> findNearestParkinglot(
  const std::shared_ptr<lanelet::LaneletMap> & lanelet_map_ptr,
  const lanelet::BasicPoint2d & current_position)
{
  const auto all_parking_lots = lanelet::utils::query::getAllParkingLots(lanelet_map_ptr);

  const auto linked_parking_lot = std::make_shared<lanelet::ConstPolygon3d>();
  const auto result = lanelet::utils::query::getLinkedParkingLot(
    current_position, all_parking_lots, linked_parking_lot.get());

  if (result) {
    return linked_parking_lot;
  } else {
    return {};
  }
}

std::shared_ptr<lanelet::ConstPolygon3d> findNearestExternalArea(
  const std::shared_ptr<lanelet::LaneletMap> & lanelet_map_ptr,
  const lanelet::BasicPoint2d & current_position)
{
  const auto all_external_areas = lanelet::utils::query::getAllPolygonsByType(lanelet_map_ptr, "external_area");

  const auto linked_external_area = std::make_shared<lanelet::ConstPolygon3d>();
  const auto result = lanelet::utils::query::getLinkedParkingLot(
    current_position, all_external_areas, linked_external_area.get());

  if (result) {
    return linked_external_area;
  } else {
    return {};
  }
}

bool isInLane(
  const std::shared_ptr<lanelet::LaneletMap> & lanelet_map_ptr,
  const geometry_msgs::msg::Point & current_pos)
{
  const auto & p = current_pos;
  const lanelet::Point3d search_point(lanelet::InvalId, p.x, p.y, p.z);

  std::vector<std::pair<double, lanelet::Lanelet>> nearest_lanelets =
    lanelet::geometry::findNearest(lanelet_map_ptr->laneletLayer, search_point.basicPoint2d(), 1);

  if (nearest_lanelets.empty()) {
    return false;
  }

  const auto nearest_lanelet = nearest_lanelets.front().second;

  return lanelet::geometry::within(search_point, nearest_lanelet.polygon3d());
}

bool isInParkingLot(
  const std::shared_ptr<lanelet::LaneletMap> & lanelet_map_ptr,
  const geometry_msgs::msg::Pose & current_pose)
{
  const auto & p = current_pose.position;
  const lanelet::Point3d search_point(lanelet::InvalId, p.x, p.y, p.z);

  const auto nearest_parking_lot =
    findNearestParkinglot(lanelet_map_ptr, search_point.basicPoint2d());

  if (!nearest_parking_lot) {
    return false;
  }

  return lanelet::geometry::within(search_point, nearest_parking_lot->basicPolygon());
}

bool isInExternalArea(
  const std::shared_ptr<lanelet::LaneletMap> & lanelet_map_ptr,
  const geometry_msgs::msg::Pose & current_pose)
{
  const auto & p = current_pose.position;
  const lanelet::Point3d search_point(lanelet::InvalId, p.x, p.y, p.z);

  const auto nearest_external_area =
    findNearestExternalArea(lanelet_map_ptr, search_point.basicPoint2d());

  if (!nearest_external_area) {
    return false;
  }

  return lanelet::geometry::within(search_point, nearest_external_area->basicPolygon());
}

bool isNearTrajectoryEnd(
  const autoware_auto_planning_msgs::msg::Trajectory::ConstSharedPtr trajectory,
  const geometry_msgs::msg::Pose & current_pose, const double th_dist)
{
  if (!trajectory || trajectory->points.empty()) {
    return false;
  }

  const auto & p1 = current_pose.position;
  const auto & p2 = trajectory->points.back().pose.position;

  const auto dist = std::hypot(p1.x - p2.x, p1.y - p2.y);

  return dist < th_dist;
}

bool isStopped(
  const std::deque<geometry_msgs::msg::TwistStamped::ConstSharedPtr> & twist_buffer,
  const double th_stopped_velocity_mps)
{
  for (const auto & twist : twist_buffer) {
    if (std::abs(twist->twist.linear.x) > th_stopped_velocity_mps) {
      return false;
    }
  }
  return true;
}

size_t findNearestExternalIndex(const std::shared_ptr<lanelet::LaneletMap> & lanelet_map_ptr, 
    const autoware_auto_planning_msgs::msg::Trajectory::ConstSharedPtr msg, size_t start_index, double search_distance, double margin_distance)
{
  size_t target_index = msg->points.size();

  double sum = 0.0;
  bool found = false;
  double target_distance = std::numeric_limits<double>::infinity();
  lanelet::Point3d old_point;
  auto front = msg->points[start_index];
  old_point.x() = front.pose.position.x;
  old_point.y() = front.pose.position.y;

  for(auto index=start_index; index < msg->points.size(); index++) {
    const auto pos = msg->points[index].pose.position;
    lanelet::Point3d point;
    point.x() = pos.x;
    point.y() = pos.y;
    double interval = std::hypot(point.x() - old_point.x(), point.y() - old_point.y());
    sum += interval;
    if(sum > search_distance) {
      break;
    }

    old_point.x() = point.x();
    old_point.y() = point.y();

    if (!found) {
      const auto nearest_external_area = findNearestExternalArea(lanelet_map_ptr, point.basicPoint2d());
      if (nearest_external_area) {        
        if(lanelet::geometry::within(point, nearest_external_area->basicPolygon())) {
          found = true;
          target_distance = sum + margin_distance;
        }
      }
    }    

    if (sum >= target_distance) {
      // Check forward or backward index is proper.
      target_index = (sum - target_distance) <= interval / 2 ?  index : index - 1;
      break;
    }
  }
  return target_index;
}

std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> jointTrajectory(
  const autoware_auto_planning_msgs::msg::Trajectory::ConstSharedPtr base_trajectory,
  const autoware_auto_planning_msgs::msg::Trajectory::ConstSharedPtr additional_trajectory, 
  bool use_smooth_extend, const double smooth_dumping_factor=0.3,  const double dumping_limit=0.01,
  const double dist_threshold=1.0, const double yaw_threshold = tier4_autoware_utils::pi / 2.0
){
  std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> output_points{};

  if(additional_trajectory->points.size() <= 1) {
    return output_points;
  }

  const auto back_point = base_trajectory -> points.back();

  const auto segment_idx = motion_utils::findNearestSegmentIndex(
        additional_trajectory -> points, back_point.pose, dist_threshold, yaw_threshold);

  if(segment_idx) {
    for(auto point : base_trajectory->points) {
      output_points.push_back(point);
    }
    auto back_longitudinal_velocity = back_point.longitudinal_velocity_mps;
    const auto& add = additional_trajectory->points;
    for(auto it=add.begin() + (*segment_idx + 1); it != add.end(); ++it) {
      if(use_smooth_extend) {
        auto new_point = *it;
        auto diff = new_point.longitudinal_velocity_mps - back_longitudinal_velocity;
        if(abs(diff) > dumping_limit) {
          back_longitudinal_velocity += smooth_dumping_factor * diff;
          new_point.longitudinal_velocity_mps = back_longitudinal_velocity;
        }
        output_points.push_back(new_point);
      } else {
        output_points.push_back(*it);
      }
    }
  } 
  return output_points;

}

}  // namespace


size_t ScenarioSelectorNode::searchExternalIndex(const autoware_auto_planning_msgs::msg::Trajectory::ConstSharedPtr msg)
{
  auto start_index = motion_utils::findNearestIndex(msg->points, current_pose_->pose.pose.position);

  auto external_index = findNearestExternalIndex(lanelet_map_ptr_, msg, start_index, search_limit_,
                                                 area_margin_length_);
  return external_index;
}

autoware_auto_planning_msgs::msg::Trajectory::ConstSharedPtr 
ScenarioSelectorNode::cutTrajectoryByExternal(const autoware_auto_planning_msgs::msg::Trajectory::ConstSharedPtr msg)
{  
  auto search_index = searchExternalIndex(msg);
  if (msg->points.size() > search_index) {
    std::vector<autoware_auto_planning_msgs::msg::TrajectoryPoint> output_points{};
    for(size_t i=0; i < search_index; i++) {
      output_points.push_back(msg->points[i]);
    }
    auto tmp_trajectory = motion_utils::convertToTrajectory(output_points);
    tmp_trajectory.header = msg->header;
    autoware_auto_planning_msgs::msg::Trajectory::ConstSharedPtr  
      trajectory(new autoware_auto_planning_msgs::msg::Trajectory(tmp_trajectory));
    return trajectory;
  } else {
    return msg;
  }

}

autoware_auto_planning_msgs::msg::Trajectory::ConstSharedPtr 
ScenarioSelectorNode::extendTrajectoryByLane(const autoware_auto_planning_msgs::msg::Trajectory::ConstSharedPtr msg)
{
  if(!lane_driving_trajectory_) {
    return msg;
  }
  auto time_diff = (rclcpp::Time(msg->header.stamp) - rclcpp::Time(lane_driving_trajectory_->header.stamp)).seconds();
  if(time_diff > th_old_trajectory_time_sec_ ) {
    return msg;
  }

  auto output_points = jointTrajectory(msg, lane_driving_trajectory_, use_smooth_extend_);

  if(output_points.size() > msg->points.size()) {
    auto tmp_trajectory = motion_utils::convertToTrajectory(output_points);
    tmp_trajectory.header = msg->header;
    autoware_auto_planning_msgs::msg::Trajectory::ConstSharedPtr  
      trajectory(new autoware_auto_planning_msgs::msg::Trajectory(tmp_trajectory));
    return trajectory;  
  } else {
    return msg;
  }
}

autoware_auto_planning_msgs::msg::Trajectory::ConstSharedPtr
ScenarioSelectorNode::getScenarioTrajectory(const std::string & scenario)
{
  if (scenario == tier4_planning_msgs::msg::Scenario::LANEDRIVING) {
    return lane_driving_trajectory_;
  }
  if (scenario == tier4_planning_msgs::msg::Scenario::PARKING) {
    return parking_trajectory_;
  }
  if (scenario == tier4_planning_msgs::msg::Scenario::EXTERNAL) {
    return external_trajectory_;
  }
  RCLCPP_ERROR_STREAM(this->get_logger(), "invalid scenario argument: " << scenario);
  return lane_driving_trajectory_;
}

std::string ScenarioSelectorNode::selectScenarioByPosition()
{
  const auto is_in_lane = isInLane(lanelet_map_ptr_, current_pose_->pose.pose.position);
  const auto is_goal_in_lane = isInLane(lanelet_map_ptr_, route_->goal_pose.position);
  const auto is_in_parking_lot = isInParkingLot(lanelet_map_ptr_, current_pose_->pose.pose);
  const auto is_in_external_area = (use_external_) ? isInExternalArea(lanelet_map_ptr_, current_pose_->pose.pose) : false;

  if (current_scenario_ == tier4_planning_msgs::msg::Scenario::EMPTY) {
    if(is_in_external_area){
      return tier4_planning_msgs::msg::Scenario::EXTERNAL;
    } else if (is_in_lane && is_goal_in_lane) {
      return tier4_planning_msgs::msg::Scenario::LANEDRIVING;
    } else if (is_in_parking_lot) {
      return tier4_planning_msgs::msg::Scenario::PARKING;
    } else {
      return tier4_planning_msgs::msg::Scenario::LANEDRIVING;
    }
  }

  if (current_scenario_ == tier4_planning_msgs::msg::Scenario::LANEDRIVING) {
    if (is_in_parking_lot && !is_goal_in_lane) {
      return tier4_planning_msgs::msg::Scenario::PARKING;
    } else if (is_in_external_area && !is_external_completed_) {
      return tier4_planning_msgs::msg::Scenario::EXTERNAL;
    }
  } else if (current_scenario_ == tier4_planning_msgs::msg::Scenario::PARKING) {
    if (is_parking_completed_ && is_in_lane) {
      is_parking_completed_ = false;
      return tier4_planning_msgs::msg::Scenario::LANEDRIVING;
    }
  } else if (current_scenario_ == tier4_planning_msgs::msg::Scenario::EXTERNAL) {
      if(!is_in_external_area || is_external_completed_) {
        if (is_in_lane && is_goal_in_lane) {
          return tier4_planning_msgs::msg::Scenario::LANEDRIVING;
        } else if (is_in_parking_lot) {
          return tier4_planning_msgs::msg::Scenario::PARKING;
        }
      } 
  }

  if(!is_in_external_area && use_external_) {
    // Completed flag off. 
    is_external_completed_ = false;
  }
  return current_scenario_;
}

void ScenarioSelectorNode::updateCurrentScenario()
{
  const auto prev_scenario = current_scenario_;
  const auto is_stopped = isStopped(twist_buffer_, th_stopped_velocity_mps_);
  // Never change scenario in safe mode if not stopped
  if(use_safe_mode_  && (!is_stopped))
  {
    return;
  }

  auto new_scenario = selectScenarioByPosition();
  if (new_scenario == tier4_planning_msgs::msg::Scenario::EXTERNAL) {
    current_scenario_ = new_scenario;
  }

  const auto scenario_trajectory = getScenarioTrajectory(current_scenario_);
  const auto is_near_trajectory_end =
    isNearTrajectoryEnd(scenario_trajectory, current_pose_->pose.pose, th_arrived_distance_m_);
  if ((is_near_trajectory_end && is_stopped) || !use_safe_mode_) {
    current_scenario_ = new_scenario;
  }

  if (current_scenario_ != prev_scenario) {
    RCLCPP_INFO_STREAM(
      this->get_logger(), "scenario changed: " << prev_scenario << " -> " << current_scenario_);
  }
}

void ScenarioSelectorNode::onMap(
  const autoware_auto_mapping_msgs::msg::HADMapBin::ConstSharedPtr msg)
{
  lanelet_map_ptr_ = std::make_shared<lanelet::LaneletMap>();
  lanelet::utils::conversion::fromBinMsg(
    *msg, lanelet_map_ptr_, &traffic_rules_ptr_, &routing_graph_ptr_);
  route_handler_ = std::make_shared<route_handler::RouteHandler>(*msg);
}

void ScenarioSelectorNode::onRoute(
  const autoware_planning_msgs::msg::LaneletRoute::ConstSharedPtr msg)
{
  // When the route id is the same (e.g. rerouting with modified goal) keep the current scenario.
  // Otherwise, reset the scenario.
  if (!route_handler_ || route_handler_->getRouteUuid() != msg->uuid) {
    current_scenario_ = tier4_planning_msgs::msg::Scenario::EMPTY;
  }

  route_ = msg;
}

void ScenarioSelectorNode::onOdom(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
  current_pose_ = msg;
  auto twist = std::make_shared<geometry_msgs::msg::TwistStamped>();
  twist->header = msg->header;
  twist->twist = msg->twist.twist;

  twist_ = twist;
  twist_buffer_.push_back(twist);

  // Delete old data in buffer
  while (true) {
    const auto time_diff =
      rclcpp::Time(msg->header.stamp) - rclcpp::Time(twist_buffer_.front()->header.stamp);

    if (time_diff.seconds() < th_stopped_time_sec_) {
      break;
    }

    twist_buffer_.pop_front();
  }
}

void ScenarioSelectorNode::onParkingState(const std_msgs::msg::Bool::ConstSharedPtr msg)
{
  is_parking_completed_ = msg->data;
}

void ScenarioSelectorNode::onExternalState(const std_msgs::msg::Bool::ConstSharedPtr msg)
{
  is_external_completed_ = msg->data;
}

bool ScenarioSelectorNode::isDataReady()
{
  if (!current_pose_) {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Waiting for current pose.");
    return false;
  }

  if (!lanelet_map_ptr_) {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Waiting for lanelet map.");
    return false;
  }

  if (!route_) {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Waiting for route.");
    return false;
  }

  if (!twist_) {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Waiting for twist.");
    return false;
  }

  // Check route handler is ready
  route_handler_->setRoute(*route_);
  if (!route_handler_->isHandlerReady()) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000, "Waiting for route handler.");
    return false;
  }

  return true;
}

void ScenarioSelectorNode::onTimer()
{
  if (!isDataReady()) {
    return;
  }

  // Initialize Scenario
  if (current_scenario_ == tier4_planning_msgs::msg::Scenario::EMPTY) {
    current_scenario_ = selectScenarioByPosition();
  }

  updateCurrentScenario();
  tier4_planning_msgs::msg::Scenario scenario;
  scenario.current_scenario = current_scenario_;

  if (current_scenario_ == tier4_planning_msgs::msg::Scenario::PARKING) {
    scenario.activating_scenarios.push_back(current_scenario_);
  }

  pub_scenario_->publish(scenario);
}

void ScenarioSelectorNode::onLaneDrivingTrajectory(
  const autoware_auto_planning_msgs::msg::Trajectory::ConstSharedPtr msg)
{
  lane_driving_trajectory_ = msg;

  if (current_scenario_ != tier4_planning_msgs::msg::Scenario::LANEDRIVING) {
    return;
  }

  if (use_safe_mode_ && use_external_ && (!is_external_completed_)) {
    auto traj = cutTrajectoryByExternal(lane_driving_trajectory_);
    publishTrajectory(traj);
  } else {
    publishTrajectory(msg);
  }
}

void ScenarioSelectorNode::onParkingTrajectory(
  const autoware_auto_planning_msgs::msg::Trajectory::ConstSharedPtr msg)
{
  parking_trajectory_ = msg;

  if (current_scenario_ != tier4_planning_msgs::msg::Scenario::PARKING) {
    return;
  }

  publishTrajectory(msg);
}

void ScenarioSelectorNode::onExternalTrajectory(
  const autoware_auto_planning_msgs::msg::Trajectory::ConstSharedPtr msg)
{
  external_trajectory_ = msg;

  if (current_scenario_ != tier4_planning_msgs::msg::Scenario::EXTERNAL) {
    return;
  }

  if (!use_safe_mode_ && use_external_extend_) {
    auto traj = extendTrajectoryByLane(external_trajectory_);
    publishTrajectory(traj);
  } else {
    publishTrajectory(msg);
  }  
}

void ScenarioSelectorNode::publishTrajectory(
  const autoware_auto_planning_msgs::msg::Trajectory::ConstSharedPtr msg)
{
  const auto now = this->now();
  const auto delay_sec = (now - msg->header.stamp).seconds();
  if (delay_sec <= th_max_message_delay_sec_) {
    pub_trajectory_->publish(*msg);
  } else {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), std::chrono::milliseconds(1000).count(),
      "trajectory is delayed: scenario = %s, delay = %f, th_max_message_delay = %f",
      current_scenario_.c_str(), delay_sec, th_max_message_delay_sec_);
  }
}

ScenarioSelectorNode::ScenarioSelectorNode(const rclcpp::NodeOptions & node_options)
: Node("scenario_selector", node_options),
  current_scenario_(tier4_planning_msgs::msg::Scenario::EMPTY),
  update_rate_(this->declare_parameter<double>("update_rate")),
  th_max_message_delay_sec_(this->declare_parameter<double>("th_max_message_delay_sec")),
  th_arrived_distance_m_(this->declare_parameter<double>("th_arrived_distance_m")),
  th_stopped_time_sec_(this->declare_parameter<double>("th_stopped_time_sec")),
  th_stopped_velocity_mps_(this->declare_parameter<double>("th_stopped_velocity_mps")),
  search_limit_(this->declare_parameter<double>("search_limit")),
  area_margin_length_(this->declare_parameter<double>("area_margin_length")),
  th_old_trajectory_time_sec_(this->declare_parameter<double>("th_old_trajectory_time_sec")),
  use_safe_mode_(this->declare_parameter<bool>("use_safe_mode")),
  use_external_(this->declare_parameter<bool>("use_external")),
  use_external_extend_(this->declare_parameter<bool>("use_external_extend")),
  use_smooth_extend_(this->declare_parameter<bool>("use_smooth_extend")),
  is_parking_completed_(false),
  is_external_completed_(false)
{
  // Input
  sub_lane_driving_trajectory_ =
    this->create_subscription<autoware_auto_planning_msgs::msg::Trajectory>(
      "input/lane_driving/trajectory", rclcpp::QoS{1},
      std::bind(&ScenarioSelectorNode::onLaneDrivingTrajectory, this, std::placeholders::_1));

  sub_parking_trajectory_ = this->create_subscription<autoware_auto_planning_msgs::msg::Trajectory>(
    "input/parking/trajectory", rclcpp::QoS{1},
    std::bind(&ScenarioSelectorNode::onParkingTrajectory, this, std::placeholders::_1));

  sub_external_trajectory_ = this->create_subscription<autoware_auto_planning_msgs::msg::Trajectory>(
    "input/external/trajectory", rclcpp::QoS{1},
    std::bind(&ScenarioSelectorNode::onExternalTrajectory, this, std::placeholders::_1));

  sub_lanelet_map_ = this->create_subscription<autoware_auto_mapping_msgs::msg::HADMapBin>(
    "input/lanelet_map", rclcpp::QoS{1}.transient_local(),
    std::bind(&ScenarioSelectorNode::onMap, this, std::placeholders::_1));
  sub_route_ = this->create_subscription<autoware_planning_msgs::msg::LaneletRoute>(
    "input/route", rclcpp::QoS{1}.transient_local(),
    std::bind(&ScenarioSelectorNode::onRoute, this, std::placeholders::_1));
  sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "input/odometry", rclcpp::QoS{100},
    std::bind(&ScenarioSelectorNode::onOdom, this, std::placeholders::_1));
  sub_parking_state_ = this->create_subscription<std_msgs::msg::Bool>(
    "is_parking_completed", rclcpp::QoS{100},
    std::bind(&ScenarioSelectorNode::onParkingState, this, std::placeholders::_1));
  sub_external_state_ = this->create_subscription<std_msgs::msg::Bool>(
    "is_external_completed", rclcpp::QoS{100},
    std::bind(&ScenarioSelectorNode::onExternalState, this, std::placeholders::_1));

  // Output
  pub_scenario_ =
    this->create_publisher<tier4_planning_msgs::msg::Scenario>("output/scenario", rclcpp::QoS{1});
  pub_trajectory_ = this->create_publisher<autoware_auto_planning_msgs::msg::Trajectory>(
    "output/trajectory", rclcpp::QoS{1});

  // Timer Callback
  const auto period_ns = rclcpp::Rate(static_cast<double>(update_rate_)).period();

  timer_ = rclcpp::create_timer(
    this, get_clock(), period_ns, std::bind(&ScenarioSelectorNode::onTimer, this));
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(ScenarioSelectorNode)
