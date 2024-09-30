#include "behaviortree_ros2/bt_action_node.hpp"
#include "behaviortree_ros2/plugins.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include <moveit/move_group_interface/move_group_interface.h>
// #include <rclcpp/rclcpp.hpp>

#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <moveit_msgs/action/move_group.hpp>

#include <moveit_msgs/msg/robot_state.hpp>
#include <moveit_msgs/msg/move_it_error_codes.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit_msgs/msg/motion_plan_request.hpp>
#include <moveit_msgs/msg/planning_options.hpp>

using namespace BT;

class MoveitGroup: public RosActionNode<moveit_msgs::action::MoveGroup>
{
public:
  MoveitGroup(const std::string& name,
              const NodeConfig& conf,
              const RosNodeParams& params)
    : RosActionNode<moveit_msgs::action::MoveGroup>(name, conf, params)
  {}

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({InputPort<std::string>("group_name"),InputPort<std::string>("planner_id"),InputPort<std::string>("pipeline_id"), InputPort<int32_t>("num_planing_attempts"),InputPort<double>("allowed_planning_time"),InputPort<double>("max_acceleration_scaling_factor"),InputPort<double>("max_velocity_scaling_factor"), });
  }

  bool setGoal(Goal& goal) override{
    // auto frame = getInput<std::string>("frame");
    // goal.frame = frame.value();

    // # Motion planning request to pass to planner
    moveit_msgs::msg::MotionPlanRequest request;

    // # Planning options
    moveit_msgs::msg::PlanningOptions planning_options;

    goal.request = request;
    goal.planning_options = planning_options;

    return true;
  }

  void onHalt() override{
    RCLCPP_INFO( node_->get_logger(), "%s: onHalt", name().c_str() );
  }

    NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback)
  {
    // std::stringstream ss;
    // ss << "Next number in sequence received: ";
    // for (auto number : feedback->partial_sequence) {
    //   ss << number << " ";
    // }
    // feedback->state
    RCLCPP_INFO(node_->get_logger(), feedback->state.c_str());
    // RCLCPP_INFO(node_->get_logger(), ss.str().c_str());
    return NodeStatus::RUNNING;
  }

  BT::NodeStatus onResultReceived(const WrappedResult& wr) override{
    // RCLCPP_INFO( node_->get_logger(), "%s: onResultReceived. Done = %s", name().c_str(), 
            //    wr.result-> error_code? "true" : "false" );
    
    // moveit_msgs::msg::MoveItErrorCodes error_code;

    // # The full starting state of the robot at the start of the trajectory
    moveit_msgs::msg::RobotState trajectory_start;

    // # The trajectory that moved group produced for execution
    moveit_msgs::msg::RobotTrajectory planned_trajectory;

    // # The trace of the trajectory recorded during execution
    moveit_msgs::msg::RobotTrajectory executed_trajectory;

    // # The amount of time it took to complete the motion plan
    double planning_time;

    bool success = false;
    if(wr.result->error_code.SUCCESS == 1){
        success = true;
    }

    return success ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
  }

  virtual BT::NodeStatus onFailure(ActionNodeErrorCode error) override{
    RCLCPP_ERROR( node_->get_logger(), "%s: onFailure with error: %s", name().c_str(), toStr(error) );
    return NodeStatus::FAILURE;
  }
};