#include <iostream>
#include <functional>
#include <memory>
#include <string>
#include <map>

#include "rclcpp/rclcpp.hpp"
#include "projeto_bixo_interfaces/msg/handler.hpp"
#include "projeto_bixo_interfaces/srv/projeto_bixo_service.hpp"

#include "b0RemoteApi.h"

// class FrontEndNode : public rclcpp::Node
// {
//   private:
//     rclcpp::Client<projeto_bixo_interfaces::srv::ProjetoBixoService>::SharedPtr client;

//   public:
//     FrontEndNode(): 
//       rclcpp::Node("front-end-node")
//     {
//       client = this->create_client<projeto_bixo_interfaces::srv::ProjetoBixoService>("client");
//     }

//     rclcpp::Client<projeto_bixo_interfaces::srv::ProjetoBixoService>::SharedFuture send_request(std::string component, long handler)
//     {
//       auto request = std::make_shared<projeto_bixo_interfaces::srv::ProjetoBixoService::Request>();
//       request->component = component;
//       request->handler = handler;

//       return(client->async_send_request(request));
      
//       // auto result = client->async_send_request(request);
//       // if (rclcpp::spin_until_future_complete(rclcpp::Node::make_shared<rclcpp::Node>(), result) ==
//       //   rclcpp::executor::FutureReturnCode::SUCCESS)
//       // {
//       //   RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Response: %s", result.get()->response);
//       // } else {
//       //   RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service projeto_bixo_service");    // CHANGE
//       // }
//     }
// };


int main(int argc, char** argv)
{

  b0RemoteApi b0_client("TutorialRemoteAPIRosClient","b0RemoteApiAddOn");

  rclcpp::init(argc, argv);

  std::map<std::string, long> component_map;
  component_map.insert(std::make_pair<std::string, int>("transportadorRob_leftMotor", 0));
  component_map.insert(std::make_pair<std::string, int>("transportadorRob_rightMotor", 0));
  component_map.insert(std::make_pair<std::string, int>("ForceSensor", 0));
  component_map.insert(std::make_pair<std::string, int>("carregadorRob_vision", 0));

  component_map["transportadorRob_leftMotor"] = b0RemoteApi::readInt(b0_client.simxGetObjectHandle("transportadorRob_leftMotor", b0_client.simxServiceCall()), 1);
  component_map["transportadorRob_rightMotor"] = b0RemoteApi::readInt(b0_client.simxGetObjectHandle("transportadorRob_rightMotor", b0_client.simxServiceCall()), 1);
  component_map["ForceSensor"] = b0RemoteApi::readInt(b0_client.simxGetObjectHandle("ForceSensor", b0_client.simxServiceCall()), 1);
  component_map["carregadorRob_vision"] = b0RemoteApi::readInt(b0_client.simxGetObjectHandle("carregadorRob_vision", b0_client.simxServiceCall()), 1);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("frontend"); // CHANGE
  rclcpp::Client<projeto_bixo_interfaces::srv::ProjetoBixoService>::SharedPtr client =                        // CHANGE
    node->create_client<projeto_bixo_interfaces::srv::ProjetoBixoService>("frontend_client");                  // CHANGE

  for (auto req : component_map)
  {
    auto request = std::make_shared<projeto_bixo_interfaces::srv::ProjetoBixoService::Request>();               // CHANGE
    request->handler = req.second;
    request->component = req.first;

    while (!client->wait_for_service(std::chrono::milliseconds(1))) {
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
  
  //auto node = std::make_shared<FrontEndNode>("frontend");

  //for (int i = 1; i < argc; i+=2)
  //{
    //node.send_request(argv[i], std::stoi(argv[i+1]));
  //}

  //rclcpp::spin(node);
  //rclcpp::shutdown();

  //return 0;
//}
