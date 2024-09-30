// #include "home_action.hpp"
#include "behaviortree_ros2/plugins.hpp"
#include "behaviortree_ros2/bt_action_node.hpp"




#include "ssg48_gripper_msgs/action/grasp.hpp"
#include "ssg48_gripper_msgs/action/homing.hpp"
#include "ssg48_gripper_msgs/action/move.hpp"

using namespace BT;

class SSG48GripperGraspAction: public RosActionNode<ssg48_gripper_msgs::action::Grasp>
{
public:
  SSG48GripperGraspAction(const std::string& name,
              const NodeConfig& conf,
              const RosNodeParams& params)
    : RosActionNode<ssg48_gripper_msgs::action::Grasp>(name, conf, params)
  {}

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({InputPort<double>("width"),InputPort<double>("speed"),InputPort<double>("force"),InputPort<double>("epsilon")});
  }

  bool setGoal(Goal& goal) override{
    auto width = getInput<double>("width");
    auto epsilon = getInput<double>("epsilon");
    auto speed = getInput<double>("speed");
    auto force = getInput<double>("force");
    goal.width = width.value();
    goal.epsilon = epsilon.value();
    goal.speed = speed.value();
    goal.force = force.value();
    return true;
  }

  void onHalt() override{
    RCLCPP_INFO( node_->get_logger(), "%s: onHalt", name().c_str() );
  }

  BT::NodeStatus onResultReceived(const WrappedResult& wr) override{
    RCLCPP_INFO( node_->get_logger(), "%s: onResultReceived. Done = %s", name().c_str(), 
               wr.result->success ? "true" : "false" );
    
    if(wr.result->success == false)
    {
        RCLCPP_INFO( node_->get_logger(), "Error: = %s", name().c_str(), 
               wr.result->error);
    }

    return wr.result->success ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
  }

  virtual BT::NodeStatus onFailure(ActionNodeErrorCode error) override{
    RCLCPP_ERROR( node_->get_logger(), "%s: onFailure with error: %s", name().c_str(), toStr(error) );
    return NodeStatus::FAILURE;
  }
};



class SSG48GripperMoveAction: public RosActionNode<ssg48_gripper_msgs::action::Move>
{
public:
  SSG48GripperMoveAction(const std::string& name,
              const NodeConfig& conf,
              const RosNodeParams& params)
    : RosActionNode<ssg48_gripper_msgs::action::Move>(name, conf, params)
  {}

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({InputPort<double>("width"), InputPort<double>("speed")});
  }

  bool setGoal(Goal& goal) override{
    auto width = getInput<double>("width");
    auto speed = getInput<double>("speed");
    goal.width = width.value();
    goal.speed = speed.value();
    
    return true;
  }

  void onHalt() override{
    RCLCPP_INFO( node_->get_logger(), "%s: onHalt", name().c_str() );
  }

  BT::NodeStatus onResultReceived(const WrappedResult& wr) override{
    RCLCPP_INFO( node_->get_logger(), "%s: onResultReceived. Done = %s", name().c_str(), 
               wr.result->success ? "true" : "false" );

    if(wr.result->success == false)
    {
        RCLCPP_INFO( node_->get_logger(), "Error: = %s", name().c_str(), 
               wr.result->error);
    }

    return wr.result->success ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
  }

  virtual BT::NodeStatus onFailure(ActionNodeErrorCode error) override{
    RCLCPP_ERROR( node_->get_logger(), "%s: onFailure with error: %s", name().c_str(), toStr(error) );
    return NodeStatus::FAILURE;
  }
};


class SSG48GripperHomingAction: public RosActionNode<ssg48_gripper_msgs::action::Homing>
{
public:
  SSG48GripperHomingAction(const std::string& name,
              const NodeConfig& conf,
              const RosNodeParams& params)
    : RosActionNode<ssg48_gripper_msgs::action::Homing>(name, conf, params)
  {}

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({});
  }

  bool setGoal(Goal& goal) override{
    
    return true;
  }

  void onHalt() override{
    RCLCPP_INFO( node_->get_logger(), "%s: onHalt", name().c_str() );
  }

  BT::NodeStatus onResultReceived(const WrappedResult& wr) override{
    RCLCPP_INFO( node_->get_logger(), "%s: onResultReceived. Done = %s", name().c_str(), 
               wr.result->success ? "true" : "false" );
    
    if(wr.result->success == false)
    {
        RCLCPP_INFO( node_->get_logger(), "Error: = %s", name().c_str(), 
               wr.result->error);
    }
    

    return wr.result->success ? NodeStatus::SUCCESS : NodeStatus::FAILURE;
  }

  virtual BT::NodeStatus onFailure(ActionNodeErrorCode error) override{
    RCLCPP_ERROR( node_->get_logger(), "%s: onFailure with error: %s", name().c_str(), toStr(error) );
    return NodeStatus::FAILURE;
  }
};