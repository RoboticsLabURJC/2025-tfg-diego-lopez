#ifndef G1_LOCOMOTION__RANDOM_MOVEMENT_HPP_
#define G1_LOCOMOTION__RANDOM_MOVEMENT_HPP_

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/float64.hpp"

#include <string>
#include <unordered_map>

#include <random>

namespace g1_locomotion
{
class Random_movement : public rclcpp::Node
{
public:
    Random_movement();
private:
    std::vector<std::string> joints_ = {
        "left_elbow_joint",
        "right_elbow_joint",
    };
    std::vector<float> joints_pos;
    std::vector<float> joints_dt;
    std::unordered_map<std::string, std::array<float, 2>> joints_limits_ = {
        {"left_elbow_joint", {-1.0472, 2.0944}},
        {"right_elbow_joint", {-1.0472, 2.0944}},
    };

    std::vector<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr> joint_publishers_;
    rclcpp::TimerBase::SharedPtr objetive_timer_;
    rclcpp::TimerBase::SharedPtr movement_timer_;

    
    void movement_cycle();
    void objetive_cycle();

    std::mt19937 gen_;
    std::uniform_real_distribution<float> dist_;
};
}

#endif
