#include "g1_locomotion/Random_movement.hpp"

namespace g1_locomotion
{
using namespace std::chrono_literals;
using std::placeholders::_1;

Random_movement::Random_movement() : Node("random_movement")
{
    joints_pos.assign(joints_.size(),0.f);
    joints_dt.assign(joints_.size(),0.f);
    joint_publishers_.resize(joints_.size());

    for (size_t i = 0; i < joints_.size(); i++) {
        std::string topic = "/g1/cmd_pos/" + joints_[i];
        joint_publishers_[i] = create_publisher<std_msgs::msg::Float64> (
            topic, 10);
    }

    objetive_timer_ = create_wall_timer(5s, std::bind(&Random_movement::objetive_cycle, this));
    movement_timer_ = create_wall_timer(100ms, std::bind(&Random_movement::movement_cycle, this));

    gen_ = std::mt19937(std::random_device{}());
    dist_ = std::uniform_real_distribution<float>(-1, 1);
}

void
Random_movement::objetive_cycle()
{
    for (size_t i = 0; i < joints_.size(); i++) {
        joints_dt[i] = (std::clamp(
            joints_pos[i] + dist_(gen_),
            joints_limits_[joints_[i]][0],
            joints_limits_[joints_[i]][1]) 
            - joints_pos[i]) / 50;
    }
}

void
Random_movement::movement_cycle()
{
    for (size_t i = 0; i < joints_.size(); i++) {
        joints_pos[i] = std::clamp(
            joints_pos[i] + joints_dt[i],
            joints_limits_[joints_[i]][0],
            joints_limits_[joints_[i]][1]);
    }

    for (size_t i = 0; i < joints_.size(); i++) {
        std_msgs::msg::Float64 msg;
        msg.data = joints_pos[i];
        joint_publishers_[i]->publish(msg);
    }
}
}