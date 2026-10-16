#include "g1_locomotion/Navigation.hpp"
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>


namespace g1_locomotion
{

using namespace std::chrono_literals;
using std::placeholders::_1;

Navigation::Navigation() : Node("navigation")
{  
    index = 0;
    odometry_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/g1/pelvis/odometry", 10, std::bind(&Navigation::odometry_callback, this, _1));
    
    twist_publisher_ = create_publisher<geometry_msgs::msg::Twist> ("/cmd_vel", 10);

    timer_ = create_wall_timer(100ms, std::bind(&Navigation::control_cycle, this));
}



void
Navigation::odometry_callback(const nav_msgs::msg::Odometry::ConstSharedPtr odometry)
{
    pos[0] = odometry->pose.pose.position.x;
    pos[1] = odometry->pose.pose.position.y;

    quat[0] = odometry->pose.pose.orientation.x;
    quat[1] = odometry->pose.pose.orientation.y;
    quat[2] = odometry->pose.pose.orientation.z;
    quat[3] = odometry->pose.pose.orientation.w;
}

void
Navigation::control_cycle()
{
    float x, w;
    switch (index)
    {
    case 4:
        x = 0.0;
        w = 0.0;
        break;
    
    default:
        float dx = pos[0] - targets_[index][0];
        float dy = pos[1] - targets_[index][1];
        float dist = std::sqrt((dx * dx) + (dy * dy));
        if (dist < 0.1) {
            index++;
        }

        if (index > 3) {
            x = 0.0;
            w = 0.0;
            break;
        }

        const float qx = quat[0];
        const float qy = quat[1];
        const float qz = quat[2];
        const float qw = quat[3];

        const float yaw = std::atan2(
            2.0f * (qw * qz + qx * qy),
            1.0f - 2.0f * (qy * qy + qz * qz)
        );

        dx = targets_[index][0] - pos[0];
        dy = targets_[index][1] - pos[1];
        const float distance = std::sqrt(dx * dx + dy * dy);

        const float target_angle = std::atan2(dy, dx);

        float angle_error = target_angle - yaw;

        angle_error = std::atan2(
            std::sin(angle_error),
            std::cos(angle_error)
        );

        constexpr float MAX_SPEED = 0.6f;
        constexpr float K_LINEAR = 1.0f;
        constexpr float K_ANGULAR = 1.5f;

        float linear_speed = std::min(
            MAX_SPEED,
            K_LINEAR * distance
        );

        // No avanzar si estamos mirando demasiado lejos
        // de la dirección del objetivo.
        linear_speed *= std::max(
            0.0f,
            std::cos(angle_error)
        );

        x = linear_speed;
        w = angle_error * K_ANGULAR;
        break;
    }

    geometry_msgs::msg::Twist msg;
    msg.linear.x = x;
    msg.angular.z = w;

    twist_publisher_->publish(msg);

}


}