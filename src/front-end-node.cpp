#include <iostream>
#include <functional>
#include <memory>
#include <string>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "projeto_bixo_interfaces/msg/handler.hpp"
#include "projeto_bixo_interfaces/srv/projeto_bixo_service.hpp"

#include "b0RemoteApi.h"

int main(int argc, char** argv)
{

  b0RemoteApi b0_client("TutorialRemoteAPIRosClient","b0RemoteApiAddOn");

  rclcpp::init(argc, argv);

  std::unordered_map<std::string, long> request_map;
  request_map.insert(std::make_pair<std::string, long>("transportadorRob_leftMotor", 0));
  request_map.insert(std::make_pair<std::string, long>("transportadorRob_rightMotor", 0));
  request_map.insert(std::make_pair<std::string, long>("ForceSensor", 0));
  request_map.insert(std::make_pair<std::string, long>("carregadorRob_vision", 0));

  for (auto req : request_map)
  {
    request_map[req.first] = b0RemoteApi::readInt(b0_client.simxGetObjectHandle(req.first.c_str(), b0_client.simxServiceCall()), 1);
  }

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("frontend"); // CHANGE
  rclcpp::Client<projeto_bixo_interfaces::srv::ProjetoBixoService>::SharedPtr client =                        // CHANGE
    node->create_client<projeto_bixo_interfaces::srv::ProjetoBixoService>("projeto_bixo_service");                  // CHANGE

  for (auto req : request_map)
  {
    auto request = std::make_shared<projeto_bixo_interfaces::srv::ProjetoBixoService::Request>();               // CHANGE
    request->handler = req.second;
    request->component = req.first;

    while (!client->wait_for_service(std::chrono::milliseconds(1000))) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
        return 0;
      }
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }

    auto result = client->async_send_request(request);
    // Wait for the result.
    if (rclcpp::spin_until_future_complete(node, result) ==
      rclcpp::executor::FutureReturnCode::SUCCESS)
    {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Response: %s", result.get()->response);
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service ProjetoBixoService");    // CHANGE
    }
  }

  rclcpp::shutdown();
  return 0;
}