#include "rclcpp/logging.hpp"
#include <chrono>
#include <functional>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>

#include <armadillo>

class KinematicModel : public rclcpp::Node {
public:
  KinematicModel() : Node("kinematic_model") {
    timer =
        this->create_wall_timer(std::chrono::milliseconds(30),
                                std::bind(&KinematicModel::timer_clb, this));

    sub = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/wheel_speed", 3,
        std::bind(&KinematicModel::sub_clb, this, std::placeholders::_1));
    pub = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 3);

    A = {{-L - W, 1, -1}, {L + W, 1, 1}, {L + W, 1, -1}, {-L - W, 1, 1}};
    inv_A = arma::pinv(A);

    RCLCPP_INFO(this->get_logger(), "Initialized kinematic_model node!");
  }

private:
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub;
  rclcpp::TimerBase::SharedPtr timer;

  geometry_msgs::msg::Twist velocity;

  arma::fmat A;
  arma::fmat inv_A;

  // Robot parameters
  float L = 0.17 / 2;
  float R = 0.1 / 2;
  float W = 0.26969 / 2;

  void timer_clb() { this->pub->publish(this->velocity); }

  void sub_clb(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {

    arma::Col<float> u = msg->data;
    arma::Col<float> w = (inv_A * u) * this->R; // * this->R

    this->velocity.angular.z = w[0];
    this->velocity.linear.x = w[1];
    this->velocity.linear.y = w[2];

    // RCLCPP_INFO(this->get_logger(), "X: %.2f", this->velocity.linear.x);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KinematicModel>());
  rclcpp::shutdown();
  return 0;
}