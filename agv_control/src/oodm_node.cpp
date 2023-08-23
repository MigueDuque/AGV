#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "nav_msgs/msg/odometry.hpp"

class OdomNode : public rclcpp::Node
{
public:
  OdomNode() : Node("odom_node")
  {
    motor_velocities_subs_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "motor_velocities", 10,
      std::bind(&OdomNode::onMotorVelocitiesMessage, this, std::placeholders::_1));

    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

    RCLCPP_INFO(this->get_logger(), "odom_node has started");
  }

private:
  void onMotorVelocitiesMessage(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    if (msg->data.size() != 4) {
      RCLCPP_ERROR(this->get_logger(), "Received an unexpected number of motor velocities");
      return;
    }

    float motor_velocity_1 = msg->data[0];
    float motor_velocity_2 = msg->data[1];
    float motor_velocity_3 = msg->data[2];
    float motor_velocity_4 = msg->data[3];

    // Aquí debes calcular las posiciones linear.x, linear.y, y angular.z
    // a partir de las velocidades del motor
    float linear_x = 0.0; // Calculado de alguna forma
    float linear_y = 0.0; // Calculado de alguna forma
    float angular_z = 0.0; // Calculado de alguna forma

    nav_msgs::msg::Odometry odom_msg;
    odom_msg.twist.twist.linear.x = linear_x;
    odom_msg.twist.twist.linear.y = linear_y;
    odom_msg.twist.twist.angular.z = angular_z;

    odom_pub_->publish(odom_msg);
    RCLCPP_INFO(this->get_logger(), "Publishing Odometry");
  }

  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr motor_velocities_subs_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto odom_node = std::make_shared<OdomNode>();
  rclcpp::spin(odom_node);
  rclcpp::shutdown();
  return 0;
}
