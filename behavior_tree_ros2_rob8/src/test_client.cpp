#include "behaviortree_ros2/bt_action_node.hpp"
#include "behaviortree_cpp/bt_factory.h"

<<<<<<< HEAD
=======
#include "behaviortree_cpp/loggers/bt_file_logger_v2.h"

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include "behaviortree_cpp/loggers/bt_sqlite_logger.h"
#include "behaviortree_cpp/xml_parsing.h"
#include "behaviortree_cpp/json_export.h"

>>>>>>> 732acf3 (a)
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors.hpp"

#include "gripper_behaviors.cpp"
#include "arm_behaviors.cpp"
#include "behaviortree_ros2/plugins.hpp"
#include "yaml-cpp/yaml.h"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "iostream"

#include "behaviortree_cpp/loggers/bt_file_logger_v2.h"

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include "behaviortree_cpp/loggers/bt_sqlite_logger.h"
#include "behaviortree_cpp/xml_parsing.h"
#include "behaviortree_cpp/json_export.h"

#include "ssg48_gripper_behaviors.cpp"

#ifndef USE_SLEEP_PLUGIN
#include "sleep_action.hpp"
#endif

using namespace BT;

//-------------------------------------------------------------
// Simple Action to print a number
//-------------------------------------------------------------

class PrintValue : public BT::SyncActionNode
{
public:
  PrintValue(const std::string& name, const BT::NodeConfig& config)
  : BT::SyncActionNode(name, config) {}

  BT::NodeStatus tick() override {
    std::string msg;
    if( getInput("message", msg ) ){
      std::cout << "PrintValue: " << msg << std::endl;
      return NodeStatus::SUCCESS;
    }
    else{
      std::cout << "PrintValue FAILED "<< std::endl;
      return NodeStatus::FAILURE;
    }
  }

  static BT::PortsList providedPorts() {
    return { BT::InputPort<std::string>("message") };
  }
};




//-----------------------------------------------------


const std::string default_bt_xml_file = 
    ament_index_cpp::get_package_share_directory("behavior_tree_ros2_rob8") + "/bt_xml/panda_test.xml";

  // Simple tree, used to execute once each action.
//   static const char* xml_text = R"(
//  <root BTCPP_format="4">
//      <BehaviorTree>#include "behaviortree_cpp/bt_factory.h"
//         <Sequence>
//             <PrintValue message="start"/>
//             <GripperAction name="gripper_open" open="true"/>
//             <ArmMovePoseAction name="arm" pose="0.45,0.1,0.165,0.0,1.529,0.0"/>
//             <ArmMovePoseAction name="arm" pose="0.45,0.1,0.1,0.0,1.529,0.0"/>
//             <GripperAction name="gripper_close" open="false"/>
//             <ArmMovePoseAction name="arm" pose="0.45,0.1,0.165,0.0,1.529,0.0"/>
//             <ArmMovePoseAction name="arm" pose="-0.1,-0.6,0.4,0.0,0.0,-1.529"/>
//             <ArmMovePoseAction name="arm" pose="0.45,0.1,0.165,0.0,1.529,0.0"/>
//             <ArmMovePoseAction name="arm" pose="0.45,0.1,0.1,0.0,1.529,0.0"/>
//             <GripperAction name="gripper_open" open="true"/>
//             <ArmMovePoseAction name="arm" pose="0.45,0.1,0.165,0.0,1.529,0.0"/>
//             <ArmMovePoseAction name="arm" pose="0.1,-0.6,0.4,0.0,0.0,-1.529"/>
//             <GripperAction name="gripper_close" open="false"/>
//             <PrintValue message="sleep completed"/>
//         </Sequence>
//      </BehaviorTree>
//  </root>
//  )";

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto nh = std::make_shared<rclcpp::Node>("test_client");

  BehaviorTreeFactory factory;

  factory.registerNodeType<PrintValue>("PrintValue");

  RosNodeParams params_ssg48_gripper_grasp;
  params_ssg48_gripper_grasp.nh = nh;
  params_ssg48_gripper_grasp.server_timeout = std::chrono::milliseconds(2000);
  params_ssg48_gripper_grasp.wait_for_server_timeout = std::chrono::milliseconds(1000);
  params_ssg48_gripper_grasp.default_port_value = "ssg48_gripper/grasp";
  factory.registerNodeType<SSG48GripperGraspAction>("SSG48GripperGraspAction",params_ssg48_gripper_grasp);
  
  RosNodeParams params_ssg48_gripper_move;
  params_ssg48_gripper_move.nh = nh;
  params_ssg48_gripper_move.server_timeout = std::chrono::milliseconds(2000);
  params_ssg48_gripper_move.wait_for_server_timeout = std::chrono::milliseconds(1000);
  params_ssg48_gripper_move.default_port_value = "ssg48_gripper/move";
  factory.registerNodeType<SSG48GripperMoveAction>("SSG48GripperMoveAction",params_ssg48_gripper_move);

  RosNodeParams params_ssg48_gripper_homing;
  params_ssg48_gripper_homing.nh = nh;
  params_ssg48_gripper_homing.server_timeout = std::chrono::milliseconds(2000);
  params_ssg48_gripper_homing.wait_for_server_timeout = std::chrono::milliseconds(1000);
  params_ssg48_gripper_homing.default_port_value = "ssg48_gripper/homing";
  factory.registerNodeType<SSG48GripperHomingAction>("SSG48GripperHomingAction",params_ssg48_gripper_homing);

  RosNodeParams params_gripper;
  params_gripper.nh = nh;
  params_gripper.server_timeout = std::chrono::milliseconds(2000);
  params_gripper.wait_for_server_timeout = std::chrono::milliseconds(1000);
  params_gripper.default_port_value = "gripper_service";
  factory.registerNodeType<GripperAction>("GripperAction",params_gripper);

  RosNodeParams params_arm_mode_pose;
  params_arm_mode_pose.nh = nh;
  params_arm_mode_pose.server_timeout = std::chrono::milliseconds(2000);
  params_arm_mode_pose.wait_for_server_timeout = std::chrono::milliseconds(1000);
  params_arm_mode_pose.default_port_value = "arm_move_pose_service";
  factory.registerNodeType<ArmMovePoseAction>("ArmMovePoseAction",params_arm_mode_pose);

  RosNodeParams params_arm_move_pliz_lin_pose_msg;
  params_arm_move_pliz_lin_pose_msg.nh = nh;
  params_arm_move_pliz_lin_pose_msg.default_port_value = "arm_move_pliz_lin_pose_msg_service";
  params_arm_move_pliz_lin_pose_msg.server_timeout = std::chrono::milliseconds(2000);
  params_arm_move_pliz_lin_pose_msg.wait_for_server_timeout = std::chrono::milliseconds(1000);
  factory.registerNodeType<ArmMovePlizLinPoseMsgAction>("ArmMovePlizLinPoseMsgAction",params_arm_move_pliz_lin_pose_msg);


  RosNodeParams params_gripper_joint;
  params_gripper_joint.nh = nh;

  params_gripper_joint.default_port_value = "gripper_joint_service";
  factory.registerNodeType<GripperJointAction>("GripperJointAction",params_gripper_joint);

  RosNodeParams params;
  params.nh = nh;
  params.server_timeout = std::chrono::milliseconds(2000);
  params.wait_for_server_timeout = std::chrono::milliseconds(1000);
  params.default_port_value = "sleep_service";

  factory.registerNodeType<ArmArrayToPoseAction>("ArmArrayToPoseAction");

#ifdef USE_SLEEP_PLUGIN
  RegisterRosNode(factory, "../lib/libsleep_action_plugin.so", params);
#else
  factory.registerNodeType<SleepAction>("SleepAction", params);
#endif

<<<<<<< HEAD
  nh->declare_parameter<std::string>("tree_xml_file", default_bt_xml_file);
  std::string tree_xml_file_ = nh->get_parameter("tree_xml_file").as_string();

  std::string xml_models = BT::writeTreeNodesModelXML(factory);
=======
  std::string xml_models = writeTreeNodesModelXML(factory);
>>>>>>> 732acf3 (a)
  std::cout << "----------- XML file  ----------\n"
            << xml_models
            << "--------------------------------\n";

<<<<<<< HEAD
  factory.registerBehaviorTreeFromFile(tree_xml_file_);
  auto tree = factory.createTree("MainTree");

  // std::cout << BT::writeTreeToXML(tree);
  std::cout << "----------- XML file  ----------\n"
            << BT::WriteTreeToXML(tree, false, false)
            << "--------------------------------\n";

  BT::Groot2Publisher publisher(tree);

=======
  factory.registerBehaviorTreeFromFile(ament_index_cpp::get_package_share_directory("behavior_tree_ros2_rob8") + "/bt_xml/pick_and_place.xml");
  auto tree = factory.createTree("pick_and_place");
>>>>>>> 732acf3 (a)
  // auto tree = factory.createTreeFromText(xml_text);
  tree.tickWhileRunning();
  // tree.tickOnce();
  // for(int i=0; i<5; i++){
  //   tree.tickWhileRunning();
  //   // tree.tickOnce();
  // }
<<<<<<< HEAD

    // let's visualize some information about the current state of the blackboards.
  std::cout << "\n------ First BB ------" << std::endl;
  tree.subtrees[0]->blackboard->debugMessage();
  std::cout << "\n------ Second BB------" << std::endl;
  tree.subtrees[1]->blackboard->debugMessage();
=======
>>>>>>> 732acf3 (a)

  return 0;
}
