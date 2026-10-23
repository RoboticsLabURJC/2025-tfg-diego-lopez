#include "g1_locomotion/Move_box.hpp"
#include <cmath>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <algorithm>

namespace g1_locomotion
{

using namespace std::chrono_literals;
using std::placeholders::_1;

Move_box::Move_box() : Node("move_box")
{  
    state = GOAL_0;
    first = true;

    shoulder_pitch_pos_ = 0.0;
    wrist_roll_pos_ = 0.0;
    
    first_joint_state_ = true;

    shoulder_pitch_index_ = -1;
    wrist_roll_index_ = -1;

    odometry_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/g1/pelvis/odometry", 10, std::bind(&Move_box::odometry_callback, this, _1));
    
    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "/g1/joint_states", 10, std::bind(&Move_box::joint_state_callback, this, _1));
    
    twist_publisher_ = create_publisher<geometry_msgs::msg::Twist> ("/cmd_vel", 10);

    not_controlled_joint_publishers_.resize(not_controlled_joints_.size());
    for (size_t i = 0; i < not_controlled_joints_.size(); i++) {
        std::string topic = "/g1/cmd_pos/" + not_controlled_joints_[i];
        not_controlled_joint_publishers_[i] = create_publisher<std_msgs::msg::Float64> (
            topic, 10);
    }

    controlled_joint_publishers_.resize(controlled_joints_.size());
    for (size_t i = 0; i < controlled_joints_.size(); i++) {
        std::string topic = "/g1/cmd_pos/" + controlled_joints_[i];
        controlled_joint_publishers_[i] = create_publisher<std_msgs::msg::Float64> (
            topic, 10);
    }

    timer_ = create_wall_timer(100ms, std::bind(&Move_box::control_cycle, this));
}

float
Move_box::distance(float x, float y)
{
    float dx = x - pos[0];
    float dy = y - pos[1];
    return std::sqrt((dx * dx) + (dy * dy));
}

void
Move_box::twist_publisher(float x, float w)
{
    geometry_msgs::msg::Twist msg;
    msg.linear.x = x;
    msg.angular.z = w;
    
    twist_publisher_->publish(msg);
}

void
Move_box::odometry_callback(const nav_msgs::msg::Odometry::ConstSharedPtr odometry)
{
    pos[0] = odometry->pose.pose.position.x;
    pos[1] = odometry->pose.pose.position.y;
    
    quat[0] = odometry->pose.pose.orientation.x;
    quat[1] = odometry->pose.pose.orientation.y;
    quat[2] = odometry->pose.pose.orientation.z;
    quat[3] = odometry->pose.pose.orientation.w;
}

void
Move_box::joint_state_callback(const sensor_msgs::msg::JointState::ConstSharedPtr joint_state)
{
    if (first_joint_state_) {
        for (size_t i = 0; i < joint_state->name.size(); ++i) {
            if (joint_state->name[i] == "left_shoulder_pitch_joint") {
                shoulder_pitch_index_ = i;
            }
            else if (joint_state->name[i] == "left_wrist_roll_joint") {
                wrist_roll_index_ = i;
            }
        }
        first_joint_state_ = false;
    }
    
    shoulder_pitch_pos_ = joint_state->position[shoulder_pitch_index_];
    wrist_roll_pos_ = joint_state->position[wrist_roll_index_];

    // RCLCPP_INFO(
    //     this->get_logger(),
    //     "Shoulder -> %.3f",
    //     wrist_roll_pos_ / WRIST_SCALE_
    // );
}

void Move_box::navigate(
    float goal_x,
    float goal_y,
    float goal_yaw)
{
    constexpr float GOAL_TOLERANCE = 0.04f;
    constexpr float ANGULAR_TOLERANCE = 0.04f;

    constexpr float MAX_ANGULAR_SPEED = 0.6f;

    constexpr float K_LINEAR = 0.5f;
    constexpr float K_ANGULAR = 1.5f;

    constexpr float ALIGNMENT_TOLERANCE = 0.15f;

    // Obtener posición actual.
    const float dx = goal_x - pos[0];
    const float dy = goal_y - pos[1];

    const float dist = std::sqrt(dx * dx + dy * dy);

    // Obtener yaw actual desde el cuaternión.
    const float qx = quat[0];
    const float qy = quat[1];
    const float qz = quat[2];
    const float qw = quat[3];

    const float yaw = std::atan2(
        2.0f * (qw * qz + qx * qy),
        1.0f - 2.0f * (qy * qy + qz * qz)
    );

    // Error angular normalizado entre -pi y pi.
    auto angle_error = [](float target, float current) {
        return std::atan2(
            std::sin(target - current),
            std::cos(target - current)
        );
    };

    float x = 0.0f;
    float w = 0.0f;

    // Inicializar la navegación al entrar en un objetivo.
    if (first)
    {
        nav_phase_ = TURN_TO_GOAL;
        navigation_done_ = false;
        first = false;
    }

    switch (nav_phase_)
    {
        case TURN_TO_GOAL:
        {
            // Orientación necesaria para alcanzar el objetivo.
            const float target_angle = std::atan2(dy, dx);

            const float error =
                angle_error(target_angle, yaw);

            // No avanzar mientras se realiza el giro.
            x = 0.0f;

            w = std::clamp(
                K_ANGULAR * error,
                -MAX_ANGULAR_SPEED,
                MAX_ANGULAR_SPEED
            );

            // Solo comenzar a avanzar cuando esté alineado.
            if (std::abs(error) < ALIGNMENT_TOLERANCE)
            {
                w = 0.0f;
                nav_phase_ = DRIVE_TO_GOAL;
            }

            break;
        }

        case DRIVE_TO_GOAL:
        {
            if (dist < GOAL_TOLERANCE)
            {
                x = 0.0f;
                w = 0.0f;
                nav_phase_ = TURN_TO_FINAL;
                break;
            }

            const float target_angle = std::atan2(dy, dx);
            const float error = angle_error(target_angle, yaw);

            w = std::clamp(
                K_ANGULAR * error,
                -MAX_ANGULAR_SPEED,
                MAX_ANGULAR_SPEED
            );

            if (std::abs(error) > ALIGNMENT_TOLERANCE)
            {
                // No avanzar si el robot no está bien orientado.
                x = 0.0f;
            }
            else
            {
                constexpr float MIN_LINEAR_SPEED = 0.4f;
                constexpr float MAX_SPEED = 0.6f;
                constexpr float SLOW_DISTANCE = 0.5f;

                if (dist > SLOW_DISTANCE)
                {
                    x = std::clamp(
                        K_LINEAR * dist,
                        MIN_LINEAR_SPEED,
                        MAX_SPEED
                    );
                }
                else
                {
                    // Aproximación final: velocidad mínima efectiva.
                    x = MIN_LINEAR_SPEED;
                }
            }

            break;
        }

        case TURN_TO_FINAL:
        {
            float angle_error = tf2NormalizeAngle(goal_yaw - yaw);

            constexpr float MIN_ANGULAR_SPEED = 0.55f;
            constexpr float MAX_ANGULAR_SPEED = 0.8f;

            if (std::abs(angle_error) < ANGULAR_TOLERANCE)
            {
                x = 0.0f;
                w = 0.0f;

                nav_phase_ = NAV_DONE;
                navigation_done_ = true;
            }
            else
            {
                x = 0.0f;

                // Velocidad angular proporcional al error,
                // respetando el mínimo que necesita el robot.
                float angular_speed = K_ANGULAR * std::abs(angle_error);

                angular_speed = std::clamp(
                    angular_speed,
                    MIN_ANGULAR_SPEED,
                    MAX_ANGULAR_SPEED
                );

                w = std::copysign(angular_speed, angle_error);
            }

            RCLCPP_INFO(
                this->get_logger(),
                "TURN_TO_FINAL | error=%.3f | x=%.3f | w=%.3f",
                angle_error, x, w
            );

            break;
        }

        case NAV_DONE:
        {
            x = 0.0f;
            w = 0.0f;
            navigation_done_ = true;

            break;
        }
    }

    RCLCPP_INFO(
        this->get_logger(),
        "NAV phase=%d | dist=%.3f | yaw=%.3f | x=%.3f | w=%.3f",
        static_cast<int>(nav_phase_),
        dist,
        yaw,
        x,
        w
    );

    twist_publisher(x, w);
}

void
Move_box::move_arms() {
    for(size_t i = 0; i < not_controlled_joints_.size(); i++) {
        std_msgs::msg::Float64 msg;
        msg.data = not_controlled_joints_pos_[i];
        not_controlled_joint_publishers_[i]->publish(msg);
    }
    
    for(size_t i = 0; i < controlled_joints_.size(); i++) {
        std_msgs::msg::Float64 msg;
        msg.data = controlled_joints_pos_[controlled_joints_[i]];
        controlled_joint_publishers_[i]->publish(msg);
        // RCLCPP_INFO(
        //     this->get_logger(),
        //     "PUBLICANDO -> %s = %.3f",
        //     controlled_joints_[i].c_str(),
        //     controlled_joints_pos_[controlled_joints_[i]]
        // );
    }
}

void
Move_box::check_state()
{
    switch (state)
    {
    case GOAL_0:
        if (navigation_done_) {
            state = PICK_1;
            first = true;
        }
        break;

    case PICK_1:
        if (abs(wrist_roll_pos_ - (WRIST_ROLL_POS_* WRIST_SCALE_)) < 0.1){
            state = PICK_2;
            first = true;
        }
        break;

    case PICK_2:
        if (abs(shoulder_pitch_pos_ - (SHOULDER_PITCH_POS_* SHOULDER_SCALE_)) < 0.05){
            state = GOAL_1;
            first = true;
            navigation_done_ = false;
        }
        break;

    case GOAL_1:
        if (navigation_done_) {
            state = DROP_1;
            first = true;
        }
        break;
        
    case DROP_1:
        if (abs(shoulder_pitch_pos_) < 0.05){
            state = DROP_2;
            first = true;
        }
        break;

    case DROP_2:
        if (abs(wrist_roll_pos_) < 0.05){
            state = WAIT;
            first = true;
        }
        break;

    case WAIT:
        break;
    }
}

void
Move_box::control_cycle() {
    check_state();

    RCLCPP_INFO(
        this->get_logger(),
        "CONTROL CYCLE -> state=%d, first=%s",
        state,
        first ? "true" : "false"
    );

    switch (state)
    {
    case GOAL_0:
        navigate(targets_[0][0], targets_[0][1], target_yaw_[0]);
        break;

    case PICK_1:
        if (first){
            controlled_joints_pos_["left_wrist_roll_joint"] = WRIST_ROLL_POS_;
            controlled_joints_pos_["right_wrist_roll_joint"] = -WRIST_ROLL_POS_;
            first = false;
        }
        twist_publisher(0.0, 0.0);
        break;

    case PICK_2:
        if (first){
            controlled_joints_pos_["left_shoulder_pitch_joint"] = SHOULDER_PITCH_POS_;
            controlled_joints_pos_["right_shoulder_pitch_joint"] = SHOULDER_PITCH_POS_;
            first = false;
        }
        twist_publisher(0.0, 0.0);
        break;

    case GOAL_1:
        navigate(targets_[1][0], targets_[1][1], target_yaw_[1]);
        break;

    case DROP_1:
        if (first){
            controlled_joints_pos_["left_shoulder_pitch_joint"] = 0.0;
            controlled_joints_pos_["right_shoulder_pitch_joint"] = 0.0;
            first = false;
        }
        twist_publisher(0.0, 0.0);
        break;

    case DROP_2:
        if (first){
            controlled_joints_pos_["left_wrist_roll_joint"] = 0.0;
            controlled_joints_pos_["right_wrist_roll_joint"] = 0.0;
            first = false;
        }
        twist_publisher(0.0, 0.0);
        break;

    case WAIT:
        twist_publisher(0.0, 0.0);
        break;
    }

    move_arms();
}

}