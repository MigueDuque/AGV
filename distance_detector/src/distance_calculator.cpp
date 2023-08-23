#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class LidarProcessingNode : public rclcpp::Node
{
public:
    LidarProcessingNode()
    : Node("lidar_processing_node")
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10, std::bind(&LidarProcessingNode::lidar_callback, this, std::placeholders::_1));
    }

private:
    void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        float min_distance = *std::min_element(msg->ranges.begin(), msg->ranges.end());
        if (min_distance < 0.5)
        {
            RCLCPP_WARN(this->get_logger(), "Obstacle detected! Distance: %.2f m", min_distance);
        }
    }
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarProcessingNode>());
    rclcpp::shutdown();
    return 0;
}
