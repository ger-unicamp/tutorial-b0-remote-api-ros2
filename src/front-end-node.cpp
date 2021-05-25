#include <iostream>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32.hpp"

#include "b0RemoteApi.h"

using namespace std::chrono_literals;

class FrontEndNode : public rclcpp::Node
{
  private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_;
    size_t count_;
  public:
    FrontEndNode(String sender_topic, String reciever_topic)
    : Node("front-end-node"), count_(0)
    {
      publisher_ = this->create_publisher<std_msgs::msg::String>(sender_topic, 10);
      subscription_ = this->create_subscription<std_msgs::msg::String>(
        reciever_topic, 
        10, 
        std::bind(&FrontEndNode::topic_callback, this, _1)
      );
      timer_ = this->create_wall_timer(
      500ms, std::bind(&FrontEndNode::send_handler, this));
    }
    void send_handler(int handler)
    {
      std_msgs::msg::Int32 message;
      message.data = handler;
      RCLCPP_INFO(this->get_logger(), "Publishing handler: '%d'", message.data);
      publisher_->publish(message);
    }
    void topic_callback(const std_msgs::msg::Int32::SharedPtr msg) const
    {
      RCLCPP_INFO(this->get_logger(), "Handler: '%d' was recieved", msg->data);
    }
};


int main(int argc, char** argv)
{
  
  printf("hello world tutorial-b0-remote-api-ros2 package\n");

  b0RemoteApi client("TutorialRemoteAPIRosClient","TutorialRemoteAPI");

  rclcpp::init(argc, argv);
  FrontEndNode node("front-end-sender", "front-end-reciever");

  for (int i = 1; i < argc; i++) {
    node.send_handler(std::stoi(argv[i]))
  }
  rclcpp::spin(std::make_shared<FrontEndNode>());
  rclcpp::shutdown();

  return 0;
}
