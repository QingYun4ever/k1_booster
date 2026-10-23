#pragma once

#include <atomic>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "game_controller_packet.h"
#include "game_controller_interface/msg/game_control_data.hpp"

class GameControllerNode : public rclcpp::Node
{
public:
    explicit GameControllerNode(std::string name);
    ~GameControllerNode() override;

    void init();
    void spin();

private:
    bool check_ip_white_list(const std::string &ip) const;
    void handle_packet(
        const game_controller::Packet &data,
        game_controller_interface::msg::GameControlData &msg) const;

    int _port = 3838;
    bool _enable_ip_white_list = false;
    std::vector<std::string> _ip_white_list;
    int _socket = -1;
    std::atomic<bool> _stopping{false};
    std::thread _thread;
    rclcpp::Publisher<game_controller_interface::msg::GameControlData>::SharedPtr _publisher;
};