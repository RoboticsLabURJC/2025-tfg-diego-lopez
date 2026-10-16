#ifndef G1_LOCOMOTION__NAVIGATION_HPP_
#define G1_LOCOMOTION__NAVIGATION_HPP_

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"


namespace g1_locomotion
{

class Navigation : public rclcpp::Node
{
public:
    Navigation();

private:
    void odometry_callback(const nav_msgs::msg::Odometry::ConstSharedPtr odometry);

    void control_cycle();

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher_;

    rclcpp::TimerBase::SharedPtr timer_;

    std::array<float,2> pos;
    std::array<float, 4> quat;

    int index;

    std::array<std::array<float, 2>, 4> targets_ = {{
        {0.76, 2.4},
        {3.30, 4.72},
        {7.83, 3.11},
        {5.25, -1.86},
    }};
};

} // namespace g1_locomotion

#endif //G1_LOCOMOTION__NAVIGATION_HPP_