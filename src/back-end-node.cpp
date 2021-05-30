#include <iostream>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "projeto_bixo_interfaces/msg/Handler.hpp"
#include "projeto_bixo_interfaces/srv/projeto_bixo_service.hpp"

#include "b0RemoteApi.h"

class BackEndNode : public rlcpp::Node
{
    private:
        rclcpp::Service<tutorial_interfaces::srv::AddThreeInts>::SharedPtr service;
    public:
    
        BackEndNode():
            rclcpp::Node("back-end-node")
        {
            this->service = 
                this->create_service<projeto_bixo_interfaces::srv::ProjetoBixoService>("service",  &this->send_response);
        }

        void send_response(const std::shared_ptr<projeto_bixo_interfaces::srv::ProjetoBixoService::Request> request,     // CHANGE
          std::shared_ptr<tutorial_interfaces::srv::ProjetoBixoService::Response> response)
        {
            std::string response_builder = "Request recieved with params: ";
            response_builder += std::to_string(request->component) + " " + std::to_string(request->handler);
            response->response = response_builder;

            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: %s", response->response);
        }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to recieve rquests!");

    rclcpp::spin(std::make_shared<rclcpp::Node>(BackEndNode()));
    rclcpp::shutdown();

    return 0;
}