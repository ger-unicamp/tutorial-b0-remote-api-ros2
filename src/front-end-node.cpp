#include <iostream>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "projeto_bixo_interfaces/msg/Handler.hpp"
#include "projeto_bixo_interfaces/srv/projeto_bixo_service.hpp"

#include "b0RemoteApi.h"

class FrontEndNode : public rclcpp::Node
{
  private:
    rclcpp::Client<projeto_bixo_interfaces::srv::ProjetoBixoService>::SharedPtr client;

  public:
    FrontEndNode(): 
      rclcpp::Node("front-end-node")
    {
      client = this->create_client<projeto_bixo_interfaces::srv::ProjetoBixoService>("client");
    }

    void send_request(string component, long handler)
    {
      auto request = std::make_shared<projeto_bixo_interfaces::srv::ProjetoBixoService::Request>();
      request->component = component;
      request->handler = handler;

      auto result = client->async_send_request(request);
      if (rclcpp::spin_until_future_complete(std::make_shared<rclpp::Node>(this), result) ==
        rclcpp::executor::FutureReturnCode::SUCCESS)
      {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Response: %s", result.get()->response);
      } else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service projeto_bixo_service");    // CHANGE
      }
    }
};


int main(int argc, char** argv)
{

  b0RemoteApi client("TutorialRemoteAPIRosClient","b0RemoteApiAddOn");

  rclcpp::init(argc, argv);
  FrontEndNode node;

  for (int i = 1; i < argc; i+=2) {
    node.send_request(argv[i], std::stoi(argv[i+1]))
  }

  rclcpp::spin(std::make_shared(node));
  rclcpp::shutdown();

  return 0;
}
