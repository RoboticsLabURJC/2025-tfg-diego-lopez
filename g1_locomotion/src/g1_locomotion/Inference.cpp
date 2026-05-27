#include "g1_locomotion/Inference.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Vector3.h"

#include "ament_index_cpp/get_package_share_directory.hpp"

namespace g1_locomotion
{

using namespace std::chrono_literals;
using std::placeholders::_1;

Inference::Inference() : Node("inference")
{
    twist_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10, std::bind(&Inference::twist_callback, this, _1));
    
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        "/g1/imu/pelvis", 10, std::bind(&Inference::imu_callback, this, _1));
    
    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "/g1/joint_states", 10, std::bind(&Inference::joint_state_callback, this, _1));
    
    odometry_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/g1/pelvis/odometry", 10, std::bind(&Inference::odometry_callback, this, _1));
    
    timer_ = create_wall_timer(20ms, std::bind(&Inference::inference_cycle, this));

    for (int i = 0; i < 29; i++) {
        std::string topic = "/g1/cmd_pos/" + order_[i];
        joint_publishers_[i] = create_publisher<std_msgs::msg::Float64> (
            topic, 10);
    }

    session_options_.SetIntraOpNumThreads(1);
    session_options_.SetGraphOptimizationLevel(
        GraphOptimizationLevel::ORT_ENABLE_ALL);

    std::string package_path =
        ament_index_cpp::get_package_share_directory("g1_locomotion");

    std::string model_path =
        package_path + "/policies/walk.onnx";
    
    session_ = std::make_unique<Ort::Session>(
        env_,
        model_path.c_str(),
        session_options_);
}

void
Inference::twist_callback(const geometry_msgs::msg::Twist::ConstSharedPtr twist)
{
    cmd_vel_[0] = twist->linear.x;
    cmd_vel_[1] = twist->linear.y;
    cmd_vel_[2] = twist->angular.z; 
}

void
Inference::imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr imu)
{
    imu_ang_[0] = imu->angular_velocity.x;
    imu_ang_[1] = imu->angular_velocity.y;
    imu_ang_[2] = imu->angular_velocity.z;

    tf2::Quaternion q(
        imu->orientation.x,
        imu->orientation.y,
        imu->orientation.z,
        imu->orientation.w
    );

    tf2::Matrix3x3 R(q);

    tf2::Vector3 g_world(0.0, 0.0, -1.0);

    tf2::Vector3 g_robot = R.transpose() * g_world;

    gravity_[0] = g_robot.x();
    gravity_[1] = g_robot.y();
    gravity_[2] = g_robot.z();
}

void
Inference::odometry_callback(const nav_msgs::msg::Odometry::ConstSharedPtr odometry)
{
    imu_lin_[0] = odometry->twist.twist.linear.x;
    imu_lin_[1] = odometry->twist.twist.linear.y;
    imu_lin_[2] = odometry->twist.twist.linear.z;
}

void
Inference::joint_state_callback(const sensor_msgs::msg::JointState::ConstSharedPtr joint_state)
{
    for(int i = 0; i < 29; i++){
        pos_[joint_order_[joint_state->name[i]]] = joint_state->position[i] - default_pos_[joint_state->name[i]];
        vel_[joint_order_[joint_state->name[i]]] = joint_state->velocity[i];
    }
}

void
Inference::inference_cycle()
{ 
    std::vector<float> obs;

    obs.insert(obs.end(), imu_lin_.begin(), imu_lin_.end());
    obs.insert(obs.end(), imu_ang_.begin(), imu_ang_.end());
    obs.insert(obs.end(), gravity_.begin(), gravity_.end());
    obs.insert(obs.end(), pos_.begin(), pos_.end());
    obs.insert(obs.end(), vel_.begin(), vel_.end());
    obs.insert(obs.end(), previous_inference_.begin(), previous_inference_.end());
    obs.insert(obs.end(), cmd_vel_.begin(), cmd_vel_.end());

    std::array<int64_t, 2> input_shape{1, 99};

    Ort::MemoryInfo memory_info = Ort::MemoryInfo::CreateCpu(
        OrtArenaAllocator,
        OrtMemTypeDefault);

    Ort::Value input_tensor = Ort::Value::CreateTensor<float>(
        memory_info,
        obs.data(),
        obs.size(),
        input_shape.data(),
        input_shape.size());
    
    const char* input_names[] = {"obs"};
    const char* output_names[] = {"actions"};

    auto output_tensors = session_->Run(
        Ort::RunOptions{nullptr},
        input_names,
        &input_tensor,
        1,
        output_names,
        1);
    
    float* raw_action = output_tensors[0].GetTensorMutableData<float>();

    float action[29];
    for (int i = 0; i < 29; i++) {
        action[i] = default_pos_[order_[i]] + raw_action[i] * action_scale_[order_[i]];
    }

    std::copy(raw_action, raw_action + 29, previous_inference_.begin());

    for (int i = 0; i < 29; i++) {
        std_msgs::msg::Float64 msg;
        msg.data = action[i];
        joint_publishers_[i]->publish(msg);
    }
}

}