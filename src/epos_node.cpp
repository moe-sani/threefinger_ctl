#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "threefinger_ctl/epos_driver.hpp"
#include "rclcpp/rclcpp.hpp"
// #include "std_msgs/msg/string.hpp"
#include "example_interfaces/msg/float64_multi_array.hpp"

using namespace std::chrono_literals;

// A ros2 node that subscribes to the joint angles published by the controller node and utilises epos driver to control the fingers
class EposNode : public rclcpp::Node
{
  public:
    threefinger_ctl::eposdriver ePos;							// Create Motor object
    void *deviceHandle;											// Create handle to EPOS device
    
    EposNode() : Node("epos_node")
    {
      // Initialise EPOS
	    deviceHandle = ePos.deviceOpen("USB0", 1);					// Open USB communication	
      init_epos();													// Initialise EPOS
      
      subscription_ = this->create_subscription<example_interfaces::msg::Float64MultiArray>(
      "joint_angles", 10, std::bind(&EposNode::topic_callback, this, std::placeholders::_1));
    }

    ~EposNode()
    {
      // Close EPOS
      end_epos();
    }

    void init_epos()
    {
      // Reset slave nodes (nodes do not reconnect after power down unless they are reset)
      ePos.nodeReset(deviceHandle, 1);
      ePos.nodeReset(deviceHandle, 2);
      ePos.nodeReset(deviceHandle, 3);

      // Set Operation Mode for Nodes (Position Mode)
      ePos.nodeOpMode(deviceHandle, 1);
      ePos.nodeOpMode(deviceHandle, 2);
      ePos.nodeOpMode(deviceHandle, 3);

      // Enable Nodes
      ePos.nodeEnable(deviceHandle, 1);
      ePos.nodeEnable(deviceHandle, 2);
      ePos.nodeEnable(deviceHandle, 3);
    }

    void end_epos()
    {
      // Loop exited. Return instrument to home pose
      ePos.nodeMotion(deviceHandle, 1, 0);				
      ePos.nodeMotion(deviceHandle, 2, 0); 						
      ePos.nodeMotion(deviceHandle, 3, 0); 						

      std::this_thread::sleep_for(std::chrono::milliseconds(3000));														// allow motors sufficient time to return home

      // Disable nodes
      ePos.nodeDisable(deviceHandle, 1);
      ePos.nodeDisable(deviceHandle, 2);
      ePos.nodeDisable(deviceHandle, 3);

      // Close communication
      ePos.deviceClose(deviceHandle, "USB0", 1);	// Close USB communication
    }

  private:
    void moveFinger(int finger, double theta)
    {
      // move the finger to the specified angle
      ePos.nodeMotion(deviceHandle, finger, theta);
    }
    void moveAllFingers(double theta1, double theta2, double theta3)
    {
      // move all the fingers to the specified angles
      ePos.nodeMotion(deviceHandle, 1, theta1);
      ePos.nodeMotion(deviceHandle, 2, theta2);
      ePos.nodeMotion(deviceHandle, 3, theta3);
    }
    void topic_callback(const example_interfaces::msg::Float64MultiArray::SharedPtr msg) const
    {
      // print all the joint angles
      RCLCPP_INFO(this->get_logger(), "I heard: '%f', '%f', '%f'", msg->data[0], msg->data[1], msg->data[2]);
      // move the fingers to the specified angles
      // moveFinger(1, msg->data[0]);
      // moveFinger(2, msg->data[1]);
      // moveFinger(3, msg->data[2]);


    }

    rclcpp::Subscription<example_interfaces::msg::Float64MultiArray>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  // Initialise EPOS node
  auto node = std::make_shared<EposNode>();

  // Spin the ros node
  rclcpp::spin(node);
  rclcpp::shutdown();
  // node.end_epos();

  return 0;
}