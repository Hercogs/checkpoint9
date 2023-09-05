#include "rclcpp/callback_group.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/utilities.hpp"
#include <chrono>
#include <cmath>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vector>

#include <armadillo>

using nav_msgs::msg::Odometry;

class EightTrajectory : public rclcpp::Node {
public:
  EightTrajectory() : Node("eight_trajectory") {

    this->sub_odom = this->create_subscription<Odometry>(
        "/odometry/filtered", 3,
        std::bind(&EightTrajectory::odom_clb, this, std::placeholders::_1));
    this->pub_wheels = this->create_publisher<std_msgs::msg::Float32MultiArray>(
        "/wheel_speed", 3);

    this->callback_group_timer = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    this->timer = this->create_wall_timer(
        std::chrono::seconds(2), std::bind(&EightTrajectory::timer_clb, this),
        this->callback_group_timer);

    // Initialize matrices

    this->H = {{-L - W, 1, -1}, {L + W, 1, 1}, {L + W, 1, -1}, {-L - W, 1, 1}};

    RCLCPP_INFO(logger, "Eight Trajectory node initilaized");
    rclcpp::sleep_for(std::chrono::seconds(1));
  }

private:
  rclcpp::Logger logger = rclcpp::get_logger("eight_trajectory");
  double phi; // Orientation of robot

  rclcpp::Subscription<Odometry>::SharedPtr sub_odom;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_wheels;
  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::CallbackGroup::SharedPtr callback_group_timer;

  // Matrices and vectors
  arma::mat A; // Vel to twist
  arma::mat H; // Twist to wheels

  // Robot parameters
  float L = 0.17 / 2;
  float R = 0.1 / 2;
  float W = 0.26969 / 2;

  // Odometry callback
  void odom_clb(const Odometry::SharedPtr msg) {

    tf2::Quaternion q;
    tf2::fromMsg(msg->pose.pose.orientation, q);

    tf2::Matrix3x3 mat(q);

    // Dummy variables
    double dummy_x, dummy_y;
    mat.getRPY(dummy_x, dummy_y, this->phi);

    // RCLCPP_INFO(logger, "Odom clb: %.2f", this->phi);
  }

  void timer_clb() {
    this->timer->cancel();
    this->execute_task();
  }

  void execute_task() {
    arma::Col<float> w1 = {0.0, 1, -1};
    arma::Col<float> w2 = {0.0, 1, 1};
    arma::Col<float> w3 = {0.0, 1, 1};
    arma::Col<float> w4 = {1.5708, 1, -1};
    arma::Col<float> w5 = {-3.1415, -1, -1};
    arma::Col<float> w6 = {0.0, -1, 1};
    arma::Col<float> w7 = {0.0, -1, 1};
    arma::Col<float> w8 = {0.0, -1, -1};
    arma::Col<float> w9 = {0.0, 0.0, 0.0}; // End

    std::vector<arma::Col<float>> waypoints = {w1, w2, w3, w4, w5,
                                               w6, w7, w8, w9};

    arma::Col<double> twist(3);
    arma::Col<double> wheel_cmd(4);

    std_msgs::msg::Float32MultiArray wheel_cmd_msg =
        std_msgs::msg::Float32MultiArray();

    // For some reasons simulation is wrong.
    // If speed is 1m/s, it takes 2 seconds to finish 1 grid square

    RCLCPP_INFO_STREAM(logger, "Starting eight trajectory motion!");

    for (auto w : waypoints) {

      RCLCPP_INFO(logger, "Going d_x: %.2f, d_y: %.2f, d_phi: %.2f", w[1], w[2],
                  w[0]);
      // To reduce spped, divide by 3
      float speed_reducer = 3.0;
      w /= speed_reducer;

      for (size_t i = 0; i < 120; i++) {
        this->velocity2twist(w, twist);

        wheel_cmd = (this->H * twist) / this->R;

        std::vector<float> wheel_cmd_vec =
            arma::conv_to<std::vector<float>>::from(wheel_cmd.col(0));
        wheel_cmd_msg.data = wheel_cmd_vec;

        this->pub_wheels->publish(wheel_cmd_msg);
        rclcpp::sleep_for(std::chrono::milliseconds(50));
      }

      // Short stop for robot to speed down, to increase precision
      wheel_cmd_msg.data = {0.0, 0.0, 0.0, 0.0};
      this->pub_wheels->publish(wheel_cmd_msg);
      rclcpp::sleep_for(std::chrono::milliseconds(250));
    }
    // RCLCPP_INFO_STREAM(logger, "Twist type : " << typeid(twist).name());
    // RCLCPP_INFO_STREAM(logger, "w1 type : " << typeid(w1).name());
    // RCLCPP_INFO_STREAM(logger, "H type : " << typeid(this->H).name());
    // RCLCPP_INFO_STREAM(logger, "A type : " << typeid(this->A).name());
    RCLCPP_INFO_STREAM(logger, "Robot stopped");
    RCLCPP_INFO_STREAM(logger, "Finished eight trajectory motion! Well done!");

    rclcpp::shutdown();
  }

  void velocity2twist(arma::Col<float> in_vec, arma::Col<double> &out_vec) {
    this->A = {{1, 0, 0},
               {0, cos(this->phi), sin(this->phi)},
               {0, -sin(this->phi), cos(this->phi)}};
    out_vec = this->A * in_vec;
    // RCLCPP_INFO_STREAM(logger, "Phi : " << this->phi);
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  std::shared_ptr<EightTrajectory> node = std::make_shared<EightTrajectory>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}