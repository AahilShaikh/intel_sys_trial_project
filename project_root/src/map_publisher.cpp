#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

using namespace std::chrono_literals;

class MapPublisher : public rclcpp::Node {
public:
    MapPublisher() : Node("map_publisher") {
        publisher_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map", rclcpp::QoS(1).transient_local());

        auto message = nav_msgs::msg::OccupancyGrid();
        message.header.frame_id = "map";
        message.info.resolution = 1.0;
        message.info.width = 100;
        message.info.height = 100;

        for(size_t r = 0; r < message.info.width; r++) {
            for(size_t c = 0; c < message.info.height; c++) {
                if (r == c) {
                    message.data.push_back(100);
                } else {
                    message.data.push_back(0);
                }
            } 
        }

        RCLCPP_INFO(this->get_logger(), "Publishing map");
        this->publisher_->publish(message);
    }

private:
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapPublisher>());
    rclcpp::shutdown();
    return 0;
}