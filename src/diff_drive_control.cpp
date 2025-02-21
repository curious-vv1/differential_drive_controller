#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float64.hpp"

class DiffDriveController : public rclcpp::Node {
public:
    DiffDriveController() : Node("diff_drive_controller") {
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&DiffDriveController::cmdVelCallback, this, std::placeholders::_1));

        left_wheel_pub_ = this->create_publisher<std_msgs::msg::Float64>("/left_wheel_rpm", 10);
        right_wheel_pub_ = this->create_publisher<std_msgs::msg::Float64>("/right_wheel_rpm", 10);

        // Set parameters
        this->declare_parameter("wheel_radius", 0.05);  // meters
        this->declare_parameter("wheel_base", 0.35);  // meters

        double wheel_radius = this->get_parameter("wheel_radius").as_double();
        double wheel_base = this->get_parameter("wheel_base").as_double();
        RCLCPP_INFO(this->get_logger(),"Publishing left wheel RPM and right wheel RPM. Wheel Base: %.2f, Wheel Radius: %.2f", wheel_base, wheel_radius);
    }

private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        double wheel_radius = this->get_parameter("wheel_radius").as_double();
        double wheel_base = this->get_parameter("wheel_base").as_double();

        
        double Vx = msg->linear.x;
        double omega = msg->angular.z;

        // Compute wheel velocities (m/s)
        double Vr = Vx + (wheel_base * omega) / 2.0;
        double Vl = Vx - (wheel_base * omega) / 2.0;

        // Convert to RPM
        int rpm_r = static_cast<int>((Vr / (2 * M_PI * wheel_radius)) * 60);
        int rpm_l = static_cast<int>((Vl / (2 * M_PI * wheel_radius)) * 60);

        // Publish RPM values
        std_msgs::msg::Float64 right_rpm_msg, left_rpm_msg;
        right_rpm_msg.data = rpm_r;
        left_rpm_msg.data = rpm_l;

        right_wheel_pub_->publish(right_rpm_msg);
        left_wheel_pub_->publish(left_rpm_msg);
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_wheel_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr right_wheel_pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DiffDriveController>());
    rclcpp::shutdown();
    return 0;
}
