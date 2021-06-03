#include <iostream>
#include <functional>
#include <memory>
#include <string>
#include <cstdarg>

#include "rclcpp/rclcpp.hpp"
#include "projeto_bixo_interfaces/msg/handler.hpp"
#include "projeto_bixo_interfaces/srv/projeto_bixo_service.hpp"

#include "b0RemoteApi.h"

class BackEndNode : public rclcpp::Node
{
    private:
        rclcpp::Service<projeto_bixo_interfaces::srv::ProjetoBixoService>::SharedPtr service;
    public:
    
        BackEndNode():
            rclcpp::Node("back_end_node")
        {
            this->service = 
                this->create_service<projeto_bixo_interfaces::srv::ProjetoBixoService>("projeto_bixo_service", this->send_response);
        }

        static void send_response(const std::shared_ptr<projeto_bixo_interfaces::srv::ProjetoBixoService::Request> request,     // CHANGE
          std::shared_ptr<projeto_bixo_interfaces::srv::ProjetoBixoService::Response> response)
        {
            response->response = build_request(4, "Request recieved with params: ", request->component, " ", std::to_string(request->handler));
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: %s", response->response);
        }

    private:
        static std::string build_request(int argc, ...)
        {
            std::va_list args;
            va_start(args, argc);
            char response_builder[1000];
            for (int i =0; i < argc; i++)
            {
                char* arg = va_arg(args, char*);
                strcpy(response_builder, arg);
            }
            va_end(args);
            return std::string(response_builder);
        }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to recieve rquests!");

    rclcpp::spin(std::make_shared<BackEndNode>());
    rclcpp::shutdown();

    return 0;
}