#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"


using namespace std::chrono_literals;

class PointPublisher : public rclcpp::Node {
public:
    PointPublisher() : Node("point_publisher") {
        // Get topic from ROS parameter
        this->declare_parameter<std::string>("point_topic", "default_topic");
        std::string topic_name = this->get_parameter("point_topic").as_string();

        publisher_ = this->create_publisher<geometry_msgs::msg::Point>(topic_name, rclcpp::QoS(1).transient_local());

        mapSubscription_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "map",
            rclcpp::QoS(1).transient_local(),
            std::bind(&PointPublisher::topicCallback, this, std::placeholders::_1));
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
        int cell = free_spaces[random_index]; // Index of the cell in msg.data

        auto message = geometry_msgs::msg::Point();
        message.x = cell % msg->info.width;
        message.y = cell / msg->info.width;

        RCLCPP_INFO(this->get_logger(), "Publishing %s point: [x: %.2f, y: %.2f]",
                    this->get_parameter("point_topic").as_string().c_str(), message.x, message.y);
        this->publisher_->publish(message);
    }

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr mapSubscription_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr publisher_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointPublisher>());
    rclcpp::shutdown();
    return 0;
}