#include <chrono>
#include <csignal>
#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "brain.h"

namespace {
volatile std::sig_atomic_t stopRequested = 0;
void requestStop(int) { stopRequested = 1; }
}

int main(int argc, char **argv)
{
    // Cancel skills while DDS is still alive, then shut down the ROS context.
    rclcpp::init(argc, argv, rclcpp::InitOptions{}, rclcpp::SignalHandlerOptions::None);
    std::signal(SIGINT, requestStop);
    std::signal(SIGTERM, requestStop);
    try {
        auto brain = std::make_shared<Brain>();
        brain->init();
        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(brain);
        while (rclcpp::ok() && !stopRequested) executor.spin_once(std::chrono::milliseconds(10));
        brain->tree->halt();
        brain->client->stopMotion();
        executor.remove_node(brain);
        brain.reset();
    } catch (const std::exception &error) {
        std::cerr << "brain startup/runtime failed: " << error.what() << '\n';
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
