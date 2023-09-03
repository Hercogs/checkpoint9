#include "rclcpp/utilities.hpp"
#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

class WheelVelocities : public rclcpp::Node {
public:
  WheelVelocities() : Node("wheel_velocities") {
    timer = this->create_wall_timer(
        std::chrono::seconds(5), std::bind(&WheelVelocities::timer_clb, this));

    pub = this->create_publisher<std_msgs::msg::Float32MultiArray>(
        "/wheel_speed", 3);

    RCLCPP_INFO(this->get_logger(),
                "Initialized wheel velocities publisher node!");
  }

private:
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub;
  rclcpp::TimerBase::SharedPtr timer;

  void timer_clb() {
    // One shot timer
    this->timer->cancel();

    auto msg = std_msgs::msg::Float32MultiArray();
    float velocity = 2.5;

    RCLCPP_INFO(this->get_logger(), "Moving forward for 3 seconds ...");
    msg.set__data(std::vector<float>{velocity, velocity, velocity, velocity});
    pub->publish((msg));
    rclcpp::sleep_for(std::chrono::seconds(3));

    RCLCPP_INFO(this->get_logger(), "Moving backwards for 3 seconds ...");
    msg.set__data(
        std::vector<float>{-velocity, -velocity, -velocity, -velocity});
    pub->publish((msg));
    rclcpp::sleep_for(std::chrono::seconds(3));

    RCLCPP_INFO(this->get_logger(), "Moving left for 3 seconds ...");
    msg.set__data(std::vector<float>{-velocity, velocity, -velocity, velocity});
    pub->publish((msg));
    rclcpp::sleep_for(std::chrono::seconds(3));

    RCLCPP_INFO(this->get_logger(), "Moving right for 3 seconds ...");
    msg.set__data(std::vector<float>{velocity, -velocity, velocity, -velocity});
    pub->publish((msg));
    rclcpp::sleep_for(std::chrono::seconds(3));

    RCLCPP_INFO(this->get_logger(), "Turn clockwise for 3 seconds ...");
    msg.set__data(std::vector<float>{velocity, -velocity, -velocity, velocity});
    pub->publish((msg));
    rclcpp::sleep_for(std::chrono::seconds(3));

    RCLCPP_INFO(this->get_logger(), "Turn counter-clockwise for 3 seconds ...");
    msg.set__data(std::vector<float>{-velocity, velocity, velocity, -velocity});
    pub->publish((msg));
    rclcpp::sleep_for(std::chrono::seconds(3));

    RCLCPP_INFO(this->get_logger(), "Stop robot");
    msg.set__data(std::vector<float>{0.0, 0.0, 0.0, 0.0});
    pub->publish((msg));

    rclcpp::shutdown();
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WheelVelocities>());
  rclcpp::shutdown();
  return 0;
}
