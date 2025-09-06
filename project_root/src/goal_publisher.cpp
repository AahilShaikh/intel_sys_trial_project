#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"

using namespace std::chrono_literals;

class GoalPublisher : public rclcpp::Node
{
public:
    GoalPublisher() : Node("goal_publisher")
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Point>("goal", 10);

        int mapHeight = 1000;

        srand(time(0));

        auto message = geometry_msgs::msg::Point();
        message.x = (rand() % (mapHeight + 1));
        message.y = (rand() % (mapHeight + 1));

        RCLCPP_INFO(this->get_logger(), "Publishing goal point: [x: %.2f, y: %.2f]", message.x, message.y);
        this->publisher_->publish(message);
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GoalPublisher>());
    rclcpp::shutdown();
    return 0;
}