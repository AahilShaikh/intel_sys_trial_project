#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"


using namespace std::chrono_literals;

class StartPublisher : public rclcpp::Node {
public:
    StartPublisher() : Node("start_publisher") {
        publisher_ = this->create_publisher<geometry_msgs::msg::Point>("start", 10);

        subscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "map",
            rclcpp::QoS(1).transient_local(),
            std::bind(&StartPublisher::topicCallback, this, std::placeholders::_1));
    }

private:
    void topicCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        std::vector<int> free_spaces; // Contains the index of cells that are free space in msg.data
        for(size_t i = 0; i < msg->info.width * msg->info.height; i++) {
            if (msg->data[i] == 0) {
                free_spaces.push_back(i);
            }
        }

        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_int_distribution<> distrib(0, free_spaces.size() - 1);

        int random_index = distrib(gen);
        int start_cell = free_spaces[random_index]; // Index of the start cell in msg.data

        auto message = geometry_msgs::msg::Point();
        message.x = start_cell % msg->info.width;
        message.y = start_cell / msg->info.height;

        RCLCPP_INFO(this->get_logger(), "Publishing starting point: [x: %.2f, y: %.2f]", message.x, message.y);
        this->publisher_->publish(message);
    }

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr publisher_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StartPublisher>());
    rclcpp::shutdown();
    return 0;
}