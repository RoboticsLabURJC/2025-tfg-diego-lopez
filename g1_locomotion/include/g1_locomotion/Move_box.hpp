#ifndef G1_LOCOMOTION__MOVE_BOX_HPP_
#define G1_LOCOMOTION__MOVE_BOX_HPP_

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float64.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include <unordered_map>

namespace g1_locomotion
{
    
class Move_box : public rclcpp::Node
{
public:
    Move_box();
    
private:    
    void odometry_callback(const nav_msgs::msg::Odometry::ConstSharedPtr odometry);
    void joint_state_callback(const sensor_msgs::msg::JointState::ConstSharedPtr joint_state);
    void control_cycle();
    
    void check_state();
    void move_arms();
    void navigate(float goal_x, float goal_y, float goal_yaw);
    float distance(float x, float y);
    void twist_publisher(float x, float w);

    std::vector<std::string> not_controlled_joints_ = {
        
        "left_shoulder_roll_joint",
        "left_shoulder_yaw_joint",
        
        "left_elbow_joint",
        
        "left_wrist_pitch_joint",
        "left_wrist_yaw_joint",
        
        "right_shoulder_roll_joint",
        "right_shoulder_yaw_joint",
        
        "right_elbow_joint",
        
        "right_wrist_pitch_joint",
        "right_wrist_yaw_joint",
    };
    std::vector<std::string> controlled_joints_ = {
        "left_shoulder_pitch_joint",
        
        "left_wrist_roll_joint",
        
        "right_shoulder_pitch_joint",
        
        "right_wrist_roll_joint",
    };
    std::vector<float> not_controlled_joints_pos_ = {
        0.5, 0.0, 
        0.0, 
        0.0, 0.0,
        -0.5, 0.0, 
        0.0, 
        0.0, 0.0,
    };
    std::array<std::array<float, 2>, 2> targets_ = {{
        {1.8, 0.0},
        {-1.7, 0.0},
    }};

    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

    std::vector<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr> not_controlled_joint_publishers_;
    std::vector<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr> controlled_joint_publishers_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher_; 
    
    int state;
    bool first;

    std::unordered_map<std::string, float> controlled_joints_pos_ = {
        {"left_shoulder_pitch_joint", 0.0},
        {"left_wrist_roll_joint", 0.0},
        {"right_shoulder_pitch_joint", 0.0},
        {"right_wrist_roll_joint", 0.0},
    };

    float SHOULDER_PITCH_POS_ = -0.75;
    float WRIST_ROLL_POS_ = -2.7;

    float SHOULDER_SCALE_ = 0.43857732;
    float WRIST_SCALE_= 0.7450087;

    bool first_joint_state_;

    int shoulder_pitch_index_;
    int wrist_roll_index_;

    float shoulder_pitch_pos_;
    float wrist_roll_pos_;
    
    std::array<float,2> pos;
    std::array<float, 4> quat;

    enum{
        GOAL_0,
        PICK_1,
        PICK_2,
        GOAL_1,
        DROP_1,
        DROP_2,
        WAIT
    };

    enum NavPhase {
        TURN_TO_GOAL,
        DRIVE_TO_GOAL,
        TURN_TO_FINAL,
        NAV_DONE
    };

    NavPhase nav_phase_ = TURN_TO_GOAL;

    bool navigation_done_ = false;

    std::array<float, 2> target_yaw_ = {
        0.0f,
        3.14159,
    };
};

} // namespace g1_locomotion

#endif //G1_LOCOMOTION__MOVE_BOX_HPP_