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
    // RCLCPP_INFO(
    //     get_logger(),
    //     "policy_joints_.size() = %zu",
    //     policy_joints_.size());
    // La política determina cuántas acciones tiene la red.
    previous_inference_.assign(policy_joints_.size(), 0.0f);
    vel_.assign(policy_joints_.size(), 0.0f);
    pos_.assign(policy_joints_.size(), 0.0f);
    // RCLCPP_INFO(
    //     get_logger(),
    //     "previous_inference_.size() = %zu",
    //     previous_inference_.size());

    joint_publishers_.resize(policy_joints_.size());

    // ------------------------------------------------------------
    // Validación de la política
    // ------------------------------------------------------------

    for (const auto& joint : policy_joints_) {

        if (default_pos_.find(joint) == default_pos_.end()) {
            throw std::runtime_error(
                "No default position configured for policy joint: " + joint);
        }

        if (action_scale_.find(joint) == action_scale_.end()) {
            throw std::runtime_error(
                "No action scale configured for policy joint: " + joint);
        }

        if (joint_order_.find(joint) == joint_order_.end()) {
            throw std::runtime_error(
                "Policy joint is not present in joint_order_: " + joint);
        }
    }

    // ------------------------------------------------------------
    // ROS subscriptions
    // ------------------------------------------------------------

    twist_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel",
        10,
        std::bind(&Inference::twist_callback, this, _1));

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        "/g1/imu/pelvis",
        10,
        std::bind(&Inference::imu_callback, this, _1));

    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "/g1/joint_states",
        10,
        std::bind(&Inference::joint_state_callback, this, _1));

    odometry_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/g1/pelvis/odometry",
        10,
        std::bind(&Inference::odometry_callback, this, _1));

    // ------------------------------------------------------------
    // Publishers
    // ------------------------------------------------------------

    for (size_t i = 0; i < policy_joints_.size(); ++i) {
        const std::string topic =
            "/g1/cmd_pos/" + policy_joints_[i];

        joint_publishers_[i] =
            create_publisher<std_msgs::msg::Float64>(topic, 10);
    }

    // ------------------------------------------------------------
    // ONNX Runtime
    // ------------------------------------------------------------

    session_options_.SetIntraOpNumThreads(1);

    session_options_.SetGraphOptimizationLevel(
        GraphOptimizationLevel::ORT_ENABLE_ALL);

    const std::string package_path =
        ament_index_cpp::get_package_share_directory(
            "g1_locomotion");

    const std::string model_path =
        package_path + model;

    session_ = std::make_unique<Ort::Session>(
        env_,
        model_path.c_str(),
        session_options_);

    // ------------------------------------------------------------
    // Inference timer
    // ------------------------------------------------------------

    timer_ = create_wall_timer(
        20ms,
        std::bind(&Inference::inference_cycle, this));
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
Inference::joint_state_callback(
    const sensor_msgs::msg::JointState::ConstSharedPtr joint_state)
{
    // for (size_t i = 0; i < joint_state->name.size(); ++i) {
    //     const auto& joint_name = joint_state->name[i];

    //     auto order_it = joint_order_.find(joint_name);
    //     if (order_it == joint_order_.end()) {
    //         continue;
    //     }

    //     const size_t index = order_it->second;

    //     auto default_it = default_pos_.find(joint_name);
    //     if (default_it == default_pos_.end()) {
    //         RCLCPP_WARN(
    //             get_logger(),
    //             "No default position configured for joint '%s'",
    //             joint_name.c_str());
    //         continue;
    //     }

    //     if (i < joint_state->position.size()) {
    //         pos_[index] =
    //             joint_state->position[i] - default_it->second;
    //     }

    //     if (i < joint_state->velocity.size()) {
    //         vel_[index] = joint_state->velocity[i];
    //     }
    // }

    for (size_t policy_idx = 0; policy_idx < policy_joints_.size(); ++policy_idx) {

        const auto& joint_name = policy_joints_[policy_idx];

        // Buscar el joint en el mensaje de JointState
        auto it = std::find(
            joint_state->name.begin(),
            joint_state->name.end(),
            joint_name);

        if (it == joint_state->name.end()) {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "Joint '%s' not found in JointState",
                joint_name.c_str());
            continue;
        }

        const size_t msg_idx =
            std::distance(joint_state->name.begin(), it);

        // Default position
        auto default_it = default_pos_.find(joint_name);

        if (default_it == default_pos_.end()) {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "No default position for joint '%s'",
                joint_name.c_str());
            continue;
        }

        // Posición relativa al default
        if (msg_idx < joint_state->position.size()) {
            pos_[policy_idx] =
                static_cast<float>(
                    joint_state->position[msg_idx] -
                    default_it->second);
        }

        // Velocidad
        if (msg_idx < joint_state->velocity.size()) {
            vel_[policy_idx] =
                static_cast<float>(
                    joint_state->velocity[msg_idx]);
        }
    }
}

void
Inference::inference_cycle()
{
    std::vector<float> obs;

    obs.reserve(
        imu_lin_.size() +
        imu_ang_.size() +
        gravity_.size() +
        pos_.size() +
        vel_.size() +
        previous_inference_.size() +
        cmd_vel_.size());

    // ------------------------------------------------------------
    // Observation
    //
    // This order must match the policy training.
    // ------------------------------------------------------------

    obs.insert(obs.end(), imu_lin_.begin(), imu_lin_.end());
    obs.insert(obs.end(), imu_ang_.begin(), imu_ang_.end());
    obs.insert(obs.end(), gravity_.begin(), gravity_.end());

    // 29 joint positions
    obs.insert(obs.end(), pos_.begin(), pos_.end());

    // 29 joint velocities
    obs.insert(obs.end(), vel_.begin(), vel_.end());

    // Previous raw network action
    obs.insert(
        obs.end(),
        previous_inference_.begin(),
        previous_inference_.end());

    // Commanded velocity
    obs.insert(obs.end(), cmd_vel_.begin(), cmd_vel_.end());

    // RCLCPP_INFO_THROTTLE(
    //     get_logger(),
    //     *get_clock(),
    //     1000,
    //     "sizes: lin=%zu ang=%zu gravity=%zu pos=%zu vel=%zu prev=%zu cmd=%zu",
    //     imu_lin_.size(),
    //     imu_ang_.size(),
    //     gravity_.size(),
    //     pos_.size(),
    //     vel_.size(),
    //     previous_inference_.size(),
    //     cmd_vel_.size());
        
    // RCLCPP_INFO_THROTTLE(
    //     get_logger(),
    //     *get_clock(),
    //     1000,
    //     "obs size=%zu | lin=(%.3f %.3f %.3f) | ang=(%.3f %.3f %.3f) | "
    //     "grav=(%.3f %.3f %.3f) | cmd=(%.3f %.3f %.3f)",
    //     obs.size(),
    //     obs[0], obs[1], obs[2],
    //     obs[3], obs[4], obs[5],
    //     obs[6], obs[7], obs[8],
    //     obs[82], obs[83], obs[84]);

    // ------------------------------------------------------------
    // ONNX input
    // ------------------------------------------------------------

    const std::array<int64_t, 2> input_shape{
        1,
        static_cast<int64_t>(obs.size())
    };

    const Ort::MemoryInfo memory_info =
        Ort::MemoryInfo::CreateCpu(
            OrtArenaAllocator,
            OrtMemTypeDefault);

    Ort::Value input_tensor =
        Ort::Value::CreateTensor<float>(
            memory_info,
            obs.data(),
            obs.size(),
            input_shape.data(),
            input_shape.size());

    const char* input_names[] = {"obs"};
    const char* output_names[] = {"actions"};

    // RCLCPP_INFO(
    //     get_logger(),
    //     "OBS: %s",
    //     [&]() {
    //         std::ostringstream ss;
    //         for (size_t i = 0; i < obs.size(); ++i) {
    //             ss << obs[i];
    //             if (i + 1 < obs.size()) ss << ", ";
    //         }
    //         return ss.str();
    //     }().c_str()
    // );

    auto output_tensors = session_->Run(
        Ort::RunOptions{nullptr},
        input_names,
        &input_tensor,
        1,
        output_names,
        1);

    float* raw_action =
        output_tensors[0].GetTensorMutableData<float>();

    
    
    // RCLCPP_INFO_THROTTLE(
    //     get_logger(),
    //     *get_clock(),
    //     1000,
    //     "actions: "
    //     "%.3f %.3f %.3f %.3f %.3f %.3f %.3f %.3f "
    //     "%.3f %.3f %.3f %.3f %.3f %.3f %.3f",
    //     raw_action[0],
    //     raw_action[1],
    //     raw_action[2],
    //     raw_action[3],
    //     raw_action[4],
    //     raw_action[5],
    //     raw_action[6],
    //     raw_action[7],
    //     raw_action[8],
    //     raw_action[9],
    //     raw_action[10],
    //     raw_action[11],
    //     raw_action[12],
    //     raw_action[13],
    //     raw_action[14]);
    
    
    // ------------------------------------------------------------
    // Store raw action for next observation
    // ------------------------------------------------------------

    std::copy(
        raw_action,
        raw_action + policy_joints_.size(),
        previous_inference_.begin());

    // ------------------------------------------------------------
    // Convert actions to joint positions and publish
    // ------------------------------------------------------------

    for (size_t i = 0; i < policy_joints_.size(); ++i) {

        const std::string& joint = policy_joints_[i];

        const float position =
            default_pos_.at(joint) +
            raw_action[i] * action_scale_.at(joint);

        std_msgs::msg::Float64 msg;
        msg.data = position;

        joint_publishers_[i]->publish(msg);
    }

    // RCLCPP_INFO_THROTTLE(
    //     get_logger(),
    //     *get_clock(),
    //     1000,
    //     "target: %.3f %.3f %.3f %.3f %.3f %.3f ...",
    //     default_pos_.at(policy_joints_[0]) +
    //         raw_action[0] * action_scale_.at(policy_joints_[0]),
    //     default_pos_.at(policy_joints_[1]) +
    //         raw_action[1] * action_scale_.at(policy_joints_[1]),
    //     default_pos_.at(policy_joints_[2]) +
    //         raw_action[2] * action_scale_.at(policy_joints_[2]),
    //     default_pos_.at(policy_joints_[3]) +
    //         raw_action[3] * action_scale_.at(policy_joints_[3]),
    //     default_pos_.at(policy_joints_[4]) +
    //         raw_action[4] * action_scale_.at(policy_joints_[4]),
    //     default_pos_.at(policy_joints_[5]) +
    //         raw_action[5] * action_scale_.at(policy_joints_[5]));

    // RCLCPP_INFO_THROTTLE(
    //     get_logger(),
    //     *get_clock(),
    //     1000,
    //     "obs size=%zu | lin=(%.3f %.3f %.3f) | ang=(%.3f %.3f %.3f) | "
    //     "grav=(%.3f %.3f %.3f) | cmd=(%.3f %.3f %.3f)",
    //     obs.size(),
    //     obs[0], obs[1], obs[2],
    //     obs[3], obs[4], obs[5],
    //     obs[6], obs[7], obs[8],
    //     obs[82], obs[83], obs[84]);
    }

}