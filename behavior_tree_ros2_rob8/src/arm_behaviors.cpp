// #include "home_action.hpp"
#include "behaviortree_ros2/bt_action_node.hpp"
#include "behaviortree_ros2/plugins.hpp"
#include "behavior_tree_ros2_actions/action/arm_move_joints.hpp"
#include "behavior_tree_ros2_actions/action/arm_move_pose.hpp"
#include "behavior_tree_ros2_actions/action/arm_move_relative_pose.hpp"
#include "behavior_tree_ros2_actions/action/arm_move_to_frame.hpp"
#include "behavior_tree_ros2_actions/action/arm_move_pliz_lin_pose_msg.hpp"
#include "tf2/LinearMath/Quaternion.h"
// #include <tf2/LinearMath/Quaternion.h>


#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
using namespace BT;

class ArmArrayToPoseAction : public BT::SyncActionNode
{
public:
  ArmArrayToPoseAction(const std::string& name, const BT::NodeConfig& config) :
    BT::SyncActionNode(name, config)
  {}

  // This Action simply write a value in the port "text"
  BT::NodeStatus tick() override
  { 
    auto array = getInput<std::vector<double>>("array");
    std::vector<double> array_ = array.value();
    geometry_msgs::msg::PoseStamped pose_goal;
    pose_goal.pose.position.x = array_[0];
    pose_goal.pose.position.y = array_[1];
    pose_goal.pose.position.z = array_[2];

    tf2::Quaternion q_new;
    q_new.setRPY(array_[3], array_[4], array_[5]);
  

    q_new.normalize();
    pose_goal.header.frame_id = "world";
    pose_goal.pose.orientation.x = q_new.x();
    pose_goal.pose.orientation.y = q_new.y();
    pose_goal.pose.orientation.z = q_new.z();
    pose_goal.pose.orientation.w = q_new.w();
    

    setOutput("pose", pose_goal);
    
    return BT::NodeStatus::SUCCESS;
  }

  // A node having ports MUST implement this STATIC method
  static BT::PortsList providedPorts()
  {
    return {BT::OutputPort<geometry_msgs::msg::PoseStamped>("pose"),InputPort<std::vector<double>>("array")};
  }
};

class ArmMoveToFrameAction: public RosActionNode<behavior_tree_ros2_actions::action::ArmMoveToFrame>
{
public:
  ArmMoveToFrameAction(const std::string& name,
              const NodeConfig& conf,
              const RosNodeParams& params)
    : RosActionNode<behavior_tree_ros2_actions::action::ArmMoveToFrame>(name, conf, params)
  {}

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({InputPort<std::string>("frame")});
  }

  bool setGoal(Goal& goal) override{
    auto frame = getInput<std::string>("frame");
    goal.frame = frame.value();
    return true;
  }

  void onHalt() override{
    RCLCPP_INFO( node_->get_logger(), "%s: onHalt", name().c_str() );
  }

  BT::NodeStatus onResultReceived(const WrappedResult& wr) override{
    RCLCPP_INFO( node_->get_logger(), "%s: onResultReceived. Done = %s", name().c_str(), 
               wr.result->done ? "true" : "false" );

    return wr.result->done ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
  }

  virtual BT::NodeStatus onFailure(ActionNodeErrorCode error) override{
    RCLCPP_ERROR( node_->get_logger(), "%s: onFailure with error: %s", name().c_str(), toStr(error) );
    return NodeStatus::FAILURE;
  }
};

using namespace BT;

class ArmMovePoseAction: public RosActionNode<behavior_tree_ros2_actions::action::ArmMovePose>
{
public:
  ArmMovePoseAction(const std::string& name,
              const NodeConfig& conf,
              const RosNodeParams& params)
    : RosActionNode<behavior_tree_ros2_actions::action::ArmMovePose>(name, conf, params)
  {}

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({InputPort<std::string>("pose")});
    // return providedBasicPorts({InputPort<std::vector<double>>("pose")});
  }

  bool setGoal(Goal& goal) override{
    // auto pose = getInput<std::vector<double>>("pose");
    
    
    // goal.pose = pose.value();
     auto pose = getInput<std::string>("pose");
    //  goal.pose = pose.value();
    //  auto pose = getInput<std::string>("pose");
    goal.pose = pose.value();
    
    
    return true;
  }

  void onHalt() override{
    RCLCPP_INFO( node_->get_logger(), "%s: onHalt", name().c_str() );
  }

  BT::NodeStatus onResultReceived(const WrappedResult& wr) override{
    RCLCPP_INFO( node_->get_logger(), "%s: onResultReceived. Done = %s", name().c_str(), 
               wr.result->done ? "true" : "false" );

    return wr.result->done ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
  }

  virtual BT::NodeStatus onFailure(ActionNodeErrorCode error) override{
    RCLCPP_ERROR( node_->get_logger(), "%s: onFailure with error: %s", name().c_str(), toStr(error) );
    return NodeStatus::FAILURE;
  }
};




class ArmMovePlizLinPoseMsgAction: public RosActionNode<behavior_tree_ros2_actions::action::ArmMovePlizLinPoseMsg>
{
public:
  ArmMovePlizLinPoseMsgAction(const std::string& name,
              const NodeConfig& conf,
              const RosNodeParams& params)
    : RosActionNode<behavior_tree_ros2_actions::action::ArmMovePlizLinPoseMsg>(name, conf, params)
  {}

  static BT::PortsList providedPorts()
  {
    // return providedBasicPorts({InputPort<std::string>("pose")});
    return providedBasicPorts({InputPort<geometry_msgs::msg::PoseStamped>("pose"),InputPort<double>("speed"),InputPort<double>("accel")});
  }

  bool setGoal(Goal& goal) override{
    auto pose = getInput<geometry_msgs::msg::PoseStamped>("pose");
    auto speed = getInput<double>("speed");
    auto accel = getInput<double>("accel");
    
    // goal.pose = pose.value();
    //  auto pose = getInput<std::string>("pose");
    //  goal.pose = pose.value();
    //  auto pose = getInput<std::string>("pose");
    goal.pose = pose.value();
    goal.speed = speed.value();
    goal.accel = accel.value();
    
    
    return true;
  }

  void onHalt() override{
    RCLCPP_INFO( node_->get_logger(), "%s: onHalt", name().c_str() );
  }

  BT::NodeStatus onResultReceived(const WrappedResult& wr) override{
    RCLCPP_INFO( node_->get_logger(), "%s: onResultReceived. Done = %s", name().c_str(), 
               wr.result->done ? "true" : "false" );

    return wr.result->done ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
  }

  virtual BT::NodeStatus onFailure(ActionNodeErrorCode error) override{
    RCLCPP_ERROR( node_->get_logger(), "%s: onFailure with error: %s", name().c_str(), toStr(error) );
    return NodeStatus::FAILURE;
  }
};