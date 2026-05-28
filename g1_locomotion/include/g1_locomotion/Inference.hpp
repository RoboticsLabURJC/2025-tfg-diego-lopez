#ifndef G1_LOCOMOTION__INFERENCE_HPP_
#define G1_LOCOMOTION__INFERENCE_HPP_

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float64.hpp"

#include <unordered_map>
#include <string>

#include <onnxruntime_cxx_api.h>


namespace g1_locomotion
{

class Inference : public rclcpp::Node
{
public:
    Inference();

private:
    void twist_callback(const geometry_msgs::msg::Twist::ConstSharedPtr twist);
    void imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr imu);
    void joint_state_callback(const sensor_msgs::msg::JointState::ConstSharedPtr joint_state);
    void odometry_callback(const nav_msgs::msg::Odometry::ConstSharedPtr odometry);

    void inference_cycle();

    std::array<float, 3> imu_lin_{};
    std::array<float, 3> imu_ang_{};
    std::array<float, 3> gravity_{};
    std::array<float, 29> pos_{};
    std::array<float, 29> vel_{};
    std::array<float, 29> previous_inference_{};
    std::array<float, 3> cmd_vel_{};

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;

    std::array<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr, 29> joint_publishers_;

    rclcpp::TimerBase::SharedPtr timer_;

    std::unordered_map<std::string, float> default_pos_ = {
        {"left_hip_pitch_joint", -0.312},
        {"left_hip_roll_joint", 0.0},
        {"left_hip_yaw_joint", 0.0},
        {"left_knee_joint", 0.669},
        {"left_ankle_pitch_joint", -0.363},
        {"left_ankle_roll_joint", 0.0},
        {"right_hip_pitch_joint", -0.312},
        {"right_hip_roll_joint", 0.0},
        {"right_hip_yaw_joint", 0.0},
        {"right_knee_joint", 0.669},
        {"right_ankle_pitch_joint", -0.363},
        {"right_ankle_roll_joint", 0.0},
        {"waist_yaw_joint", 0.0},
        {"waist_roll_joint", 0.0},
        {"waist_pitch_joint", 0.0},
        {"left_shoulder_pitch_joint", 0.2},
        {"left_shoulder_roll_joint", 0.2},
        {"left_shoulder_yaw_joint", 0.0},
        {"left_elbow_joint", 0.6},
        {"left_wrist_roll_joint", 0.0},
        {"left_wrist_pitch_joint", 0.0},
        {"left_wrist_yaw_joint", 0.0},
        {"right_shoulder_pitch_joint", 0.2},
        {"right_shoulder_roll_joint", -0.2},
        {"right_shoulder_yaw_joint", 0.0},
        {"right_elbow_joint", 0.6},
        {"right_wrist_roll_joint", 0.0},
        {"right_wrist_pitch_joint", 0.0},
        {"right_wrist_yaw_joint", 0.0}
    };

    std::unordered_map<std::string, float> action_scale_ = {
        {"left_hip_pitch_joint", 0.54754645},
        {"left_hip_roll_joint", 0.35066146},
        {"left_hip_yaw_joint", 0.54754645},
        {"left_knee_joint", 0.35066146},
        {"left_ankle_pitch_joint", 0.43857732},
        {"left_ankle_roll_joint", 0.43857732},
        {"right_hip_pitch_joint", 0.54754645},
        {"right_hip_roll_joint", 0.35066146},
        {"right_hip_yaw_joint", 0.54754645},
        {"right_knee_joint", 0.35066146},
        {"right_ankle_pitch_joint", 0.43857732},
        {"right_ankle_roll_joint", 0.43857732},
        {"waist_yaw_joint", 0.54754645},
        {"waist_roll_joint", 0.43857732},
        {"waist_pitch_joint", 0.43857732},
        {"left_shoulder_pitch_joint", 0.43857732},
        {"left_shoulder_roll_joint", 0.43857732},
        {"left_shoulder_yaw_joint", 0.43857732},
        {"left_elbow_joint", 0.43857732},
        {"left_wrist_roll_joint", 0.43857732},
        {"left_wrist_pitch_joint", 0.07450087},
        {"left_wrist_yaw_joint", 0.07450087},
        {"right_shoulder_pitch_joint", 0.43857732},
        {"right_shoulder_roll_joint", 0.43857732},
        {"right_shoulder_yaw_joint", 0.43857732},
        {"right_elbow_joint", 0.43857732},
        {"right_wrist_roll_joint", 0.43857732},
        {"right_wrist_pitch_joint", 0.07450087},
        {"right_wrist_yaw_joint", 0.07450087}
    };

    std::unordered_map<std::string, int> joint_order_ = {
        {"left_hip_pitch_joint", 0},
        {"left_hip_roll_joint", 1},
        {"left_hip_yaw_joint", 2},
        {"left_knee_joint", 3},
        {"left_ankle_pitch_joint", 4},
        {"left_ankle_roll_joint", 5},
        {"right_hip_pitch_joint", 6},
        {"right_hip_roll_joint", 7},
        {"right_hip_yaw_joint", 8},
        {"right_knee_joint", 9},
        {"right_ankle_pitch_joint", 10},
        {"right_ankle_roll_joint", 11},
        {"waist_yaw_joint", 12},
        {"waist_roll_joint", 13},
        {"waist_pitch_joint", 14},
        {"left_shoulder_pitch_joint", 15},
        {"left_shoulder_roll_joint", 16},
        {"left_shoulder_yaw_joint", 17},
        {"left_elbow_joint", 18},
        {"left_wrist_roll_joint", 19},
        {"left_wrist_pitch_joint", 20},
        {"left_wrist_yaw_joint", 21},
        {"right_shoulder_pitch_joint", 22},
        {"right_shoulder_roll_joint", 23},
        {"right_shoulder_yaw_joint", 24},
        {"right_elbow_joint", 25},
        {"right_wrist_roll_joint", 26},
        {"right_wrist_pitch_joint", 27},
        {"right_wrist_yaw_joint", 28}
    };

    std::vector<std::string> order_ = {
        "left_hip_pitch_joint",
        "left_hip_roll_joint",
        "left_hip_yaw_joint",
        "left_knee_joint",
        "left_ankle_pitch_joint",
        "left_ankle_roll_joint",
        "right_hip_pitch_joint",
        "right_hip_roll_joint",
        "right_hip_yaw_joint",
        "right_knee_joint",
        "right_ankle_pitch_joint",
        "right_ankle_roll_joint",
        "waist_yaw_joint",
        "waist_roll_joint",
        "waist_pitch_joint",
        "left_shoulder_pitch_joint",
        "left_shoulder_roll_joint",
        "left_shoulder_yaw_joint",
        "left_elbow_joint",
        "left_wrist_roll_joint",
        "left_wrist_pitch_joint",
        "left_wrist_yaw_joint",
        "right_shoulder_pitch_joint",
        "right_shoulder_roll_joint",
        "right_shoulder_yaw_joint",
        "right_elbow_joint",
        "right_wrist_roll_joint",
        "right_wrist_pitch_joint",
        "right_wrist_yaw_joint",
    };

    Ort::Env env_{ORT_LOGGING_LEVEL_WARNING, "g1"};
    Ort::SessionOptions session_options_;
    std::unique_ptr<Ort::Session> session_;
};

} // namespace g1_locomotion

#endif //G1_LOCOMOTION__INFERENCE_HPP_