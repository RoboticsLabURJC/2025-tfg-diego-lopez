#include "g1_locomotion/Policy_manager.hpp"

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <stdexcept>
#include <algorithm>
#include <filesystem>
#include <unordered_set>
#include <cstdint>
#include <limits>
#include <utility>

#include "ament_index_cpp/get_package_share_directory.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Vector3.h"


namespace g1_locomotion
{

PolicyManager::PolicyManager()
    : Node("policy_manager")
{
    // Retain the latest catalog for compatible late subscribers.
    const auto catalog_qos =
        rclcpp::QoS(rclcpp::KeepLast(1))
            .reliable()
            .transient_local();

    available_policies_pub_ =
        create_publisher<g1_locomotion::msg::PolicyList>(
            "/g1/locomotion/available_policies",
            catalog_qos);
    
    active_policy_pub_ =
        create_publisher<std_msgs::msg::String>(
            "/g1/locomotion/active_policy",
            catalog_qos);

    load_robot_config();
    // Register all joints from the shared configuration.
    for (const auto& entry : robot_config_.default_positions) {
        joint_measurements_.try_emplace(entry.first);
    }
    
    
    load_policies();
    if (policies_.find(robot_config_.default_policy) == policies_.end()) {
        throw std::runtime_error(
            "Default policy is not available: " +
            robot_config_.default_policy);
    }

    load_policy_models();
    create_joint_publishers();

    publish_available_policies();
    publish_active_policy();

    policy_sub_ = create_subscription<std_msgs::msg::String>(
        "/g1/locomotion/set_policy",
        rclcpp::QoS(1).reliable().durability_volatile(),
        [this](const std_msgs::msg::String::ConstSharedPtr msg)
        {
            policy_callback(msg);
        });

    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        "/g1/joint_states",
        rclcpp::QoS(1).best_effort(),
        [this](const sensor_msgs::msg::JointState::ConstSharedPtr msg)
        {
            joint_state_callback(msg);
        });

    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        "/g1/imu/pelvis",
        rclcpp::QoS(1).best_effort(),
        [this](const sensor_msgs::msg::Imu::ConstSharedPtr msg)
        {
            imu_callback(msg);
        });
    
    odometry_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        "/g1/pelvis/odometry",
        rclcpp::QoS(1).best_effort(),
        [this](const nav_msgs::msg::Odometry::ConstSharedPtr msg)
        {
            odometry_callback(msg);
        });

    twist_sub_ = create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel",
        rclcpp::QoS(1).reliable().durability_volatile(),
        [this](const geometry_msgs::msg::Twist::ConstSharedPtr msg)
        {
            twist_callback(msg);
        });

    // Request the startup policy.
    pending_policy_name_ = robot_config_.default_policy;
    ++policy_request_id_;

    // Check activation progress without blocking sensor callbacks.
    supervision_timer_ = create_wall_timer(
        std::chrono::milliseconds(20),
        [this]()
        {
            supervision_cycle();
        });

}

PolicyManager::PolicyConfig 
PolicyManager::load_policy_config(const std::string& yaml_path)
{
    const YAML::Node config = YAML::LoadFile(yaml_path);

    if (!config.IsMap()) {
        throw std::runtime_error(
            "Policy configuration must be a map: " + yaml_path);
    }

    PolicyConfig policy;

    // Required fields.
    policy.name = config["name"].as<std::string>();
    policy.model_path = config["model"].as<std::string>();
    policy.controlled_joints =
        config["controlled_joints"].as<std::vector<std::string>>();
    policy.observation_order =
        config["observation_order"].as<std::vector<std::string>>();

    // Optional fields override the defaults only when present.
    if (config["frequency_hz"].IsDefined()) {
        policy.frequency_hz = config["frequency_hz"].as<double>();
    }

    if (config["input_name"].IsDefined()) {
        policy.input_name = config["input_name"].as<std::string>();
    }

    if (config["output_name"].IsDefined()) {
        policy.output_name = config["output_name"].as<std::string>();
    }

    // Observe controlled joints unless an explicit list is provided.
    if (config["observed_joints"].IsDefined()) {
        policy.observed_joints =
            config["observed_joints"].as<std::vector<std::string>>();
    } else {
        policy.observed_joints = policy.controlled_joints;
    }

    // Reject empty identifiers and lists.
    if (policy.name.empty() ||
        policy.model_path.empty() ||
        policy.input_name.empty() ||
        policy.output_name.empty() ||
        policy.controlled_joints.empty() ||
        policy.observed_joints.empty() ||
        policy.observation_order.empty())
    {
        throw std::runtime_error(
            "Policy configuration contains empty fields: " + yaml_path);
    }

    if (!std::isfinite(policy.frequency_hz) ||
        policy.frequency_hz <= 0.0)
    {
        throw std::runtime_error(
            "Policy frequency must be finite and positive: " + yaml_path);
    }

    return policy;
}

void 
PolicyManager::load_policies()
{
    namespace fs = std::filesystem;

    const fs::path package_path =
        ament_index_cpp::get_package_share_directory("g1_locomotion");

    const fs::path config_directory = package_path / "config";

    if (!fs::is_directory(config_directory)) {
        throw std::runtime_error(
            "Policy configuration directory not found: " +
            config_directory.string());
    }

    std::vector<fs::path> yaml_files;

    // Collect policy files. Reserve robot.yaml for shared settings.
    for (const auto& entry : fs::directory_iterator(config_directory)) {
        if (!entry.is_regular_file()) {
            continue;
        }

        const auto& path = entry.path();
        const std::string extension = path.extension().string();

        if ((extension == ".yaml" || extension == ".yml") &&
            path.filename() != "robot.yaml")
        {
            yaml_files.push_back(path);
        }
    }

    // Keep loading and catalog publication order deterministic.
    std::sort(yaml_files.begin(), yaml_files.end());

    policies_.clear();
    available_policy_names_.clear();

    for (const auto& yaml_path : yaml_files) {
        PolicyConfig policy = load_policy_config(yaml_path.string());
        validate_policy_joints(policy);
        validate_observation_order(policy);

        policy.observation_size = calculate_observation_size(policy);

        // Resolve relative model paths from the package share directory.
        fs::path model_path(policy.model_path);
        if (model_path.is_relative()) {
            model_path = package_path / model_path;
        }

        if (!fs::is_regular_file(model_path)) {
            throw std::runtime_error(
                "Policy model file not found: " + model_path.string());
        }

        policy.model_path = model_path.string();

        if (policies_.find(policy.name) != policies_.end()) {
            throw std::runtime_error(
                "Duplicate policy name in: " + yaml_path.string());
        }

        policies_.emplace(policy.name, policy);
        available_policy_names_.push_back(policy.name);

        RCLCPP_INFO(
            get_logger(),
            "Loaded policy configuration: '%s'",
            policy.name.c_str());
    }

    if (policies_.empty()) {
        throw std::runtime_error("No policy configurations found");
    }
}

void 
PolicyManager::publish_available_policies()
{
    g1_locomotion::msg::PolicyList msg;
    msg.names = available_policy_names_;

    available_policies_pub_->publish(msg);

}

void 
PolicyManager::policy_callback(
    const std_msgs::msg::String::ConstSharedPtr msg)
{
    const std::string& requested_policy = msg->data;

    // Invalid requests do not replace an existing valid request.
    if (policies_.find(requested_policy) == policies_.end()) {
        RCLCPP_WARN(
            get_logger(),
            "Unknown policy requested: '%s'",
            requested_policy.c_str());
        return;
    }

    // Requesting the active policy cancels any pending switch.
    if (requested_policy == active_policy_name_) {
        ++policy_request_id_;
        pending_policy_name_.clear();
        return;
    }

    // Repeated requests do not restart the same preparation.
    if (requested_policy == pending_policy_name_) {
        return;
    }

    pending_policy_name_ = requested_policy;
    ++policy_request_id_;

    RCLCPP_INFO(
        get_logger(),
        "Policy switch requested: '%s'",
        requested_policy.c_str());
}

void 
PolicyManager::load_robot_config()
{
    namespace fs = std::filesystem;

    const fs::path config_path =
        fs::path(
            ament_index_cpp::get_package_share_directory("g1_locomotion"))
        / "config" / "robot.yaml";

    const YAML::Node config = YAML::LoadFile(config_path.string());

    if (!config.IsMap()) {
        throw std::runtime_error(
            "Robot configuration must be a map: " + config_path.string());
    }

    const YAML::Node robot = config["robot"];

    if (!robot.IsMap()) {
        throw std::runtime_error(
            "Missing or invalid 'robot' section: " + config_path.string());
    }

    // Build a temporary configuration before updating the node state.
    RobotConfig loaded_config;

    // Require an explicit startup policy.
    loaded_config.default_policy =
        robot["default_policy"].as<std::string>();

    if (loaded_config.default_policy.empty()) {
        throw std::runtime_error("Default policy name must not be empty");
    }

    // Override the default sensor timeout when explicitly configured.
    if (robot["sensor_timeout_ms"].IsDefined()) {
        const int timeout_ms = robot["sensor_timeout_ms"].as<int>();

        if (timeout_ms <= 0) {
            throw std::runtime_error(
                "sensor_timeout_ms must be a positive integer");
        }

        loaded_config.sensor_timeout =
            std::chrono::milliseconds(timeout_ms);
    }

    const auto read_joint_map =
        [](const YAML::Node& section,
           const std::string& section_name,
           std::unordered_map<std::string, float>& destination)
        {
            if (!section.IsMap() || section.size() == 0) {
                throw std::runtime_error(
                    "Expected a non-empty map: " + section_name);
            }

            for (const auto& entry : section) {
                const std::string joint = entry.first.as<std::string>();
                const float value = entry.second.as<float>();

                if (joint.empty() || !std::isfinite(value)) {
                    throw std::runtime_error(
                        "Invalid joint or value in " + section_name);
                }

                if (!destination.emplace(joint, value).second) {
                    throw std::runtime_error(
                        "Duplicate joint in " + section_name + ": " + joint);
                }
            }
        };

    read_joint_map(
        robot["default_positions"],
        "default_positions",
        loaded_config.default_positions);

    read_joint_map(
        robot["action_scales"],
        "action_scales",
        loaded_config.action_scales);

    // Require matching joint sets in the shared configuration.
    if (loaded_config.default_positions.size() !=
        loaded_config.action_scales.size())
    {
        throw std::runtime_error(
            "Default positions and action scales must cover the same joints");
    }

    for (const auto& entry : loaded_config.default_positions) {
        if (loaded_config.action_scales.find(entry.first) ==
            loaded_config.action_scales.end())
        {
            throw std::runtime_error(
                "Missing action scale for joint: " + entry.first);
        }
    }

    robot_config_ = std::move(loaded_config);

    RCLCPP_INFO(
        get_logger(),
        "Loaded shared configuration for %zu joints",
        robot_config_.default_positions.size());
}

void 
PolicyManager::validate_policy_joints(
    const PolicyConfig& policy) const
{
    const auto validate_list =
        [this, &policy](
            const std::vector<std::string>& joints,
            const std::string& list_name,
            bool requires_action_scale)
        {
            std::unordered_set<std::string> seen;

            for (const auto& joint : joints) {
                if (!seen.insert(joint).second) {
                    throw std::runtime_error(
                        "Policy '" + policy.name +
                        "': duplicate joint in " + list_name +
                        ": " + joint);
                }

                if (robot_config_.default_positions.find(joint) ==
                    robot_config_.default_positions.end())
                {
                    throw std::runtime_error(
                        "Policy '" + policy.name +
                        "': missing default position for joint: " + joint);
                }

                if (requires_action_scale &&
                    robot_config_.action_scales.find(joint) ==
                    robot_config_.action_scales.end())
                {
                    throw std::runtime_error(
                        "Policy '" + policy.name +
                        "': missing action scale for joint: " + joint);
                }
            }
        };

    validate_list(policy.observed_joints, "observed_joints", false);
    validate_list(policy.controlled_joints, "controlled_joints", true);
}

void 
PolicyManager::validate_observation_order(
    const PolicyConfig& policy) const
{
    static const std::unordered_set<std::string> supported_blocks{
        "base_linear_velocity",
        "base_angular_velocity",
        "projected_gravity",
        "relative_joint_positions",
        "joint_velocities",
        "previous_actions",
        "velocity_command"
    };

    std::unordered_set<std::string> seen;

    for (const auto& block : policy.observation_order) {
        if (supported_blocks.find(block) == supported_blocks.end()) {
            throw std::runtime_error(
                "Policy '" + policy.name +
                "': unsupported observation block: " + block);
        }

        if (!seen.insert(block).second) {
            throw std::runtime_error(
                "Policy '" + policy.name +
                "': duplicate observation block: " + block);
        }
    }
}

std::size_t 
PolicyManager::calculate_observation_size(
    const PolicyConfig& policy) const
{
    std::size_t size = 0;

    for (const auto& block : policy.observation_order) {
        if (block == "relative_joint_positions" ||
            block == "joint_velocities")
        {
            size += policy.observed_joints.size();
        }
        else if (block == "previous_actions") {
            size += policy.controlled_joints.size();
        }
        else if (block == "base_linear_velocity" ||
                 block == "base_angular_velocity" ||
                 block == "projected_gravity" ||
                 block == "velocity_command")
        {
            size += 3;
        }
        else {
            throw std::runtime_error(
                "Policy '" + policy.name +
                "': cannot calculate size for observation block: " + block);
        }
    }

    return size;
}

void 
PolicyManager::load_policy_models()
{
    session_options_.SetIntraOpNumThreads(1);
    session_options_.SetGraphOptimizationLevel(
        GraphOptimizationLevel::ORT_ENABLE_ALL);

    Ort::AllocatorWithDefaultOptions allocator;

    // Build all runtimes before updating the stored collection.
    std::unordered_map<std::string, PolicyRuntime> loaded_runtimes;

    for (const auto& name : available_policy_names_) {
        const PolicyConfig& policy = policies_.at(name);

        try {
            PolicyRuntime runtime;

            runtime.session = std::make_shared<Ort::Session>(
                ort_env_,
                policy.model_path.c_str(),
                session_options_);

            auto& session = *runtime.session;

            if (session.GetInputCount() != 1 ||
                session.GetOutputCount() != 1)
            {
                throw std::runtime_error(
                    "Expected exactly one input and one output");
            }

            const auto input_name =
                session.GetInputNameAllocated(0, allocator);
            const auto output_name =
                session.GetOutputNameAllocated(0, allocator);

            if (policy.input_name != input_name.get() ||
                policy.output_name != output_name.get())
            {
                throw std::runtime_error(
                    "Input/output names do not match the YAML configuration");
            }

            // Validate the tensor type and its declared dimensions.
            const auto validate_tensor =
                [](const Ort::TypeInfo& type_info,
                   std::size_t expected_size,
                   const std::string& label)
                {
                    if (type_info.GetONNXType() != ONNX_TYPE_TENSOR) {
                        throw std::runtime_error(label + " must be a tensor");
                    }

                    const auto tensor_info =
                        type_info.GetTensorTypeAndShapeInfo();

                    if (tensor_info.GetElementType() !=
                        ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT)
                    {
                        throw std::runtime_error(label + " must use float32");
                    }

                    const auto shape = tensor_info.GetShape();

                    if (shape.size() != 2) {
                        throw std::runtime_error(
                            label + " must have shape [batch, size]");
                    }

                    if ((shape[0] != 1 && shape[0] != -1) ||
                        shape[1] != static_cast<int64_t>(expected_size))
                    {
                        throw std::runtime_error(
                            label + " dimensions do not match the policy");
                    }
                };

            validate_tensor(
                session.GetInputTypeInfo(0),
                policy.observation_size,
                "Input");

            validate_tensor(
                session.GetOutputTypeInfo(0),
                policy.controlled_joints.size(),
                "Output");

            runtime.previous_actions.assign(
                policy.controlled_joints.size(), 0.0f);
            runtime.observation.assign(
                policy.observation_size, 0.0f);
            runtime.actions.assign(policy.controlled_joints.size(), 0.0f);
            runtime.position_targets.assign(
                policy.controlled_joints.size(), 0.0);

            loaded_runtimes.emplace(name, std::move(runtime));

            RCLCPP_INFO(
                get_logger(),
                "Loaded ONNX policy: '%s'",
                name.c_str());
        }
        catch (const std::exception& error) {
            throw std::runtime_error(
                "Failed to load policy '" + name + "': " + error.what());
        }
    }

    policy_runtimes_ = std::move(loaded_runtimes);
}

void 
PolicyManager::publish_active_policy()
{
    std_msgs::msg::String msg;
    msg.data = active_policy_name_;

    active_policy_pub_->publish(msg);
}

void 
PolicyManager::joint_state_callback(
    const sensor_msgs::msg::JointState::ConstSharedPtr msg)
{
    // Require both position and velocity for every listed joint.
    if (msg->position.size() != msg->name.size() ||
        msg->velocity.size() != msg->name.size())
    {
        // Invalidate stored measurements after a malformed message.
        for (auto& entry : joint_measurements_) {
            entry.second.valid = false;
        }

        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "JointState must provide position and velocity for every name");
        return;
    }

    // Reject ambiguous messages before updating any measurements.
    std::unordered_set<std::string> seen;

    for (const auto& name : msg->name) {
        if (!seen.insert(name).second) {
            for (auto& entry : joint_measurements_) {
                entry.second.valid = false;
            }

            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "JointState contains duplicate joint names");
            return;
        }
    }

    const auto received_at = std::chrono::steady_clock::now();

    for (std::size_t i = 0; i < msg->name.size(); ++i) {
        auto it = joint_measurements_.find(msg->name[i]);

        // Ignore joints outside the configured robot model.
        if (it == joint_measurements_.end()) {
            continue;
        }

        auto& measurement = it->second;

        if (!std::isfinite(msg->position[i]) ||
            !std::isfinite(msg->velocity[i]))
        {
            measurement.valid = false;

            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "Non-finite measurement for joint '%s'",
                msg->name[i].c_str());
            continue;
        }

        measurement.position = msg->position[i];
        measurement.velocity = msg->velocity[i];
        measurement.last_update = received_at;
        measurement.valid = true;
    }
}

void
PolicyManager::imu_callback(
    const sensor_msgs::msg::Imu::ConstSharedPtr msg)
{
    const auto received_at = std::chrono::steady_clock::now();

    // Revalidate each observation independently for every message.
    imu_measurement_.angular_velocity_valid = false;
    imu_measurement_.projected_gravity_valid = false;

    const std::array<double, 3> angular_velocity{
        msg->angular_velocity.x,
        msg->angular_velocity.y,
        msg->angular_velocity.z
    };

    const bool angular_values_valid = std::all_of(
        angular_velocity.begin(),
        angular_velocity.end(),
        [](double value)
        {
            return std::isfinite(value) &&
                   std::abs(value) <= std::numeric_limits<float>::max();
        });

    if (msg->angular_velocity_covariance[0] != -1.0 &&
        angular_values_valid)
    {
        for (std::size_t i = 0; i < angular_velocity.size(); ++i) {
            imu_measurement_.angular_velocity[i] =
                static_cast<float>(angular_velocity[i]);
        }

        imu_measurement_.angular_velocity_updated_at = received_at;
        imu_measurement_.angular_velocity_valid = true;
    }

    const auto& orientation = msg->orientation;

    // Compute the quaternion norm without squaring large components.
    const double norm = std::hypot(
        std::hypot(orientation.x, orientation.y),
        std::hypot(orientation.z, orientation.w));

    if (msg->orientation_covariance[0] != -1.0 &&
        std::isfinite(norm) && norm > 1e-12)
    {
        const tf2::Quaternion q(
            orientation.x / norm,
            orientation.y / norm,
            orientation.z / norm,
            orientation.w / norm);

        const tf2::Matrix3x3 rotation(q);
        const tf2::Vector3 gravity_world(0.0, 0.0, -1.0);

        const tf2::Vector3 gravity_base =
            rotation.transpose() * gravity_world;

        imu_measurement_.projected_gravity = {
            static_cast<float>(gravity_base.x()),
            static_cast<float>(gravity_base.y()),
            static_cast<float>(gravity_base.z())
        };

        imu_measurement_.projected_gravity_updated_at = received_at;
        imu_measurement_.projected_gravity_valid = true;
    }

    if (!imu_measurement_.angular_velocity_valid ||
        !imu_measurement_.projected_gravity_valid)
    {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "IMU observation unavailable: angular_velocity_valid=%d, "
            "projected_gravity_valid=%d",
            static_cast<int>(imu_measurement_.angular_velocity_valid),
            static_cast<int>(imu_measurement_.projected_gravity_valid));
    }
}

void 
PolicyManager::odometry_callback(
    const nav_msgs::msg::Odometry::ConstSharedPtr msg)
{
    base_velocity_measurement_.valid = false;

    const auto& linear = msg->twist.twist.linear;

    const std::array<double, 3> velocity{
        linear.x,
        linear.y,
        linear.z
    };

    // Require finite values representable as float32.
    const bool values_valid = std::all_of(
        velocity.begin(),
        velocity.end(),
        [](double value)
        {
            return std::isfinite(value) &&
                   std::abs(value) <= std::numeric_limits<float>::max();
        });

    if (!values_valid) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Odometry contains invalid base linear velocity");
        return;
    }

    for (std::size_t i = 0; i < velocity.size(); ++i) {
        base_velocity_measurement_.linear_velocity[i] =
            static_cast<float>(velocity[i]);
    }

    base_velocity_measurement_.last_update =
        std::chrono::steady_clock::now();

    base_velocity_measurement_.valid = true;
}

void 
PolicyManager::twist_callback(
    const geometry_msgs::msg::Twist::ConstSharedPtr msg)
{
    const std::array<double, 3> command{
        msg->linear.x,
        msg->linear.y,
        msg->angular.z
    };

    const bool values_valid = std::all_of(
        command.begin(),
        command.end(),
        [](double value)
        {
            return std::isfinite(value) &&
                   std::abs(value) <= std::numeric_limits<float>::max();
        });

    if (!values_valid) {
        // Preserve the last valid command.
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000,
            "Invalid velocity command; keeping the previous command");
        return;
    }

    for (std::size_t i = 0; i < command.size(); ++i) {
        velocity_command_[i] = static_cast<float>(command[i]);
    }
}

void
PolicyManager::create_joint_publishers()
{
    for (const auto& policy_name : available_policy_names_) {
        const auto& policy = policies_.at(policy_name);

        for (const auto& joint : policy.controlled_joints) {
            // Reuse publishers shared by multiple policies.
            if (joint_publishers_.find(joint) != joint_publishers_.end()) {
                continue;
            }

            const std::string topic = "/g1/cmd_pos/" + joint;

            auto publisher =
                create_publisher<std_msgs::msg::Float64>(
                    topic,
                    rclcpp::QoS(1).reliable().durability_volatile());

            joint_publishers_.emplace(joint, std::move(publisher));
        }
    }

    RCLCPP_INFO(
        get_logger(),
        "Created position publishers for %zu joints",
        joint_publishers_.size());
}

void 
PolicyManager::build_observation(
    const PolicyConfig& policy, PolicyRuntime& runtime, 
    const ObservationSnapshot& snapshot, const std::array<float, 3>& command)
{
    // Reuse the memory allocated during model loading.
    auto& observation = runtime.observation;
    observation.clear();

    const auto append_value =
        [&observation](double value)
        {
            if (!std::isfinite(value) ||
                std::abs(value) > std::numeric_limits<float>::max())
            {
                throw std::runtime_error(
                    "Observation contains a value not representable as float32");
            }

            observation.push_back(static_cast<float>(value));
        };

    const auto append_values =
        [&append_value](const auto& values)
        {
            for (const auto value : values) {
                append_value(value);
            }
        };

    for (const auto& block : policy.observation_order) {
        if (block == "base_linear_velocity") {
            append_values(snapshot.base_velocity.linear_velocity);
        }
        else if (block == "base_angular_velocity") {
            append_values(snapshot.imu.angular_velocity);
        }
        else if (block == "projected_gravity") {
            append_values(snapshot.imu.projected_gravity);
        }
        else if (block == "relative_joint_positions") {
            for (const auto& joint : policy.observed_joints) {
                const double relative_position =
                    snapshot.joints.at(joint).position -
                    robot_config_.default_positions.at(joint);

                append_value(relative_position);
            }
        }
        else if (block == "joint_velocities") {
            for (const auto& joint : policy.observed_joints) {
                append_value(snapshot.joints.at(joint).velocity);
            }
        }
        else if (block == "previous_actions") {
            if (runtime.previous_actions.size() !=
                policy.controlled_joints.size())
            {
                throw std::runtime_error(
                    "Previous action size mismatch for policy: " + policy.name);
            }

            append_values(runtime.previous_actions);
        }
        else if (block == "velocity_command") {
            append_values(command);
        }
        else {
            throw std::runtime_error(
                "Unsupported observation block: " + block);
        }
    }

    if (observation.size() != policy.observation_size) {
        throw std::runtime_error(
            "Observation size mismatch for policy: " + policy.name);
    }
}

bool
PolicyManager::observations_ready(
    const PolicyConfig& policy, std::chrono::steady_clock::duration max_age) const
{
    using Clock = std::chrono::steady_clock;

    if (max_age <= Clock::duration::zero()) {
        return false;
    }

    const auto now = Clock::now();

    const auto is_recent =
        [now, max_age](bool valid, Clock::time_point updated_at)
        {
            return valid &&
                   updated_at <= now &&
                   now - updated_at <= max_age;
        };

    for (const auto& block : policy.observation_order) {
        if (block == "base_linear_velocity") {
            if (!is_recent(
                    base_velocity_measurement_.valid,
                    base_velocity_measurement_.last_update))
            {
                return false;
            }
        }
        else if (block == "base_angular_velocity") {
            if (!is_recent(
                    imu_measurement_.angular_velocity_valid,
                    imu_measurement_.angular_velocity_updated_at))
            {
                return false;
            }
        }
        else if (block == "projected_gravity") {
            if (!is_recent(
                    imu_measurement_.projected_gravity_valid,
                    imu_measurement_.projected_gravity_updated_at))
            {
                return false;
            }
        }
        else if (block == "relative_joint_positions" ||
                 block == "joint_velocities")
        {
            for (const auto& joint : policy.observed_joints) {
                const auto& measurement = joint_measurements_.at(joint);

                if (!is_recent(
                        measurement.valid,
                        measurement.last_update))
                {
                    return false;
                }
            }
        }
        else if (block == "previous_actions" ||
                 block == "velocity_command")
        {
            // These blocks use internal state, not sensor measurements.
            continue;
        }
        else {
            return false;
        }
    }

    return true;
}

void 
PolicyManager::compute_actions(
    const PolicyConfig& policy, PolicyRuntime& runtime,
    const ObservationSnapshot& snapshot, const std::array<float, 3>& command)
{
    if (!runtime.session) {
        throw std::runtime_error(
            "Missing ONNX session for policy: " + policy.name);
    }

    build_observation(policy, runtime, snapshot, command);

    const std::array<int64_t, 2> input_shape{
        1,
        static_cast<int64_t>(runtime.observation.size())
    };

    const auto memory_info =
        Ort::MemoryInfo::CreateCpu(
            OrtArenaAllocator,
            OrtMemTypeDefault);

    auto input_tensor = Ort::Value::CreateTensor<float>(
        memory_info,
        runtime.observation.data(),
        runtime.observation.size(),
        input_shape.data(),
        input_shape.size());

    const char* input_names[]{policy.input_name.c_str()};
    const char* output_names[]{policy.output_name.c_str()};

    auto outputs = runtime.session->Run(
        Ort::RunOptions{nullptr},
        input_names,
        &input_tensor,
        1,
        output_names,
        1);

    if (outputs.size() != 1 || !outputs[0].IsTensor()) {
        throw std::runtime_error(
            "Expected one output tensor for policy: " + policy.name);
    }

    const auto output_info = outputs[0].GetTensorTypeAndShapeInfo();

    if (output_info.GetElementType() !=
        ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT)
    {
        throw std::runtime_error(
            "Expected float32 actions for policy: " + policy.name);
    }

    const auto output_shape = output_info.GetShape();
    const auto action_count = policy.controlled_joints.size();

    if (output_shape.size() != 2 ||
        output_shape[0] != 1 ||
        output_shape[1] != static_cast<int64_t>(action_count))
    {
        throw std::runtime_error(
            "Unexpected action tensor shape for policy: " + policy.name);
    }

    const float* actions = outputs[0].GetTensorData<float>();

    // Validate the complete output before updating stored actions.
    for (std::size_t i = 0; i < action_count; ++i) {
        if (!std::isfinite(actions[i])) {
            throw std::runtime_error(
                "Non-finite action returned by policy: " + policy.name);
        }
    }

    if (runtime.actions.size() != action_count) {
        throw std::runtime_error(
            "Action buffer size mismatch for policy: " + policy.name);
    }

    std::copy(
        actions,
        actions + action_count,
        runtime.actions.begin());
}

void 
PolicyManager::reset_policy_runtime(
    const PolicyConfig& policy, PolicyRuntime& runtime)
{
    const auto action_count = policy.controlled_joints.size();

    // Discard actions from any previous execution of this policy.
    runtime.previous_actions.assign(action_count, 0.0f);
    runtime.actions.assign(action_count, 0.0f);

    // Reset the observation buffer to its configured size.
    runtime.observation.assign(policy.observation_size, 0.0f);
}

void 
PolicyManager::prepare_joint_targets(
    const PolicyConfig& policy, PolicyRuntime& runtime)
{
    const auto joint_count = policy.controlled_joints.size();

    if (runtime.actions.size() != joint_count ||
        runtime.position_targets.size() != joint_count)
    {
        throw std::runtime_error(
            "Command buffer size mismatch for policy: " + policy.name);
    }

    for (std::size_t i = 0; i < joint_count; ++i) {
        const auto& joint = policy.controlled_joints[i];

        const double reference =
            robot_config_.default_positions.at(joint);
        const double scale =
            robot_config_.action_scales.at(joint);

        const double target =
            reference + static_cast<double>(runtime.actions[i]) * scale;

        if (!std::isfinite(target)) {
            throw std::runtime_error(
                "Non-finite position target for joint: " + joint);
        }

        const auto publisher_it = joint_publishers_.find(joint);

        if (publisher_it == joint_publishers_.end() ||
            !publisher_it->second)
        {
            throw std::runtime_error(
                "Missing position publisher for joint: " + joint);
        }

        runtime.position_targets[i] = target;
    }
}

void 
PolicyManager::publish_joint_targets(
    const PolicyConfig& policy, const PolicyRuntime& runtime)
{
    for (std::size_t i = 0; i < policy.controlled_joints.size(); ++i) {
        const auto& joint = policy.controlled_joints[i];

        std_msgs::msg::Float64 msg;
        msg.data = runtime.position_targets.at(i);

        joint_publishers_.at(joint)->publish(msg);
    }
}

void 
PolicyManager::inference_cycle()
{
    process_policy_request();
    try {
        if (active_policy_name_.empty()) {
            return;
        }

        const auto& policy = policies_.at(active_policy_name_);
        auto& runtime = policy_runtimes_.at(active_policy_name_);

        if (!observations_ready(policy, robot_config_.sensor_timeout)) {
            throw std::runtime_error(
                "Required sensor observations are invalid or outdated");
        }

        const auto snapshot = capture_observation_snapshot();

        compute_actions(policy, runtime, snapshot, velocity_command_);
        prepare_joint_targets(policy, runtime);
        publish_joint_targets(policy, runtime);

        // Save the raw action only after publishing the current targets.
        std::copy(
            runtime.actions.begin(),
            runtime.actions.end(),
            runtime.previous_actions.begin());
        // The new policy completed its first inference and publication cycle.
        activation_backup_.reset();

    }
    catch (const std::exception& error) {
        RCLCPP_ERROR_THROTTLE(
            get_logger(),
            *get_clock(),
            2000,
            "Inference cycle failed for policy '%s': %s",
            active_policy_name_.c_str(),
            error.what());

        if (activation_backup_) {
            try {
                rollback_policy_activation();
            }
            catch (const std::exception& rollback_error) {
                RCLCPP_ERROR(
                    get_logger(),
                    "Policy rollback encountered an error: %s",
                    rollback_error.what());
            }
        }

        // Keep the selected policy and retry on the next timer callback.
        // Do not update action history after a failed cycle.
    }
}

PolicyManager::ObservationSnapshot
PolicyManager::capture_observation_snapshot() const
{
    ObservationSnapshot snapshot;

    snapshot.joints = joint_measurements_;
    snapshot.imu = imu_measurement_;
    snapshot.base_velocity = base_velocity_measurement_;

    return snapshot;
}

void 
PolicyManager::start_policy_preparation()
{
    // Allow only one background preparation at a time.
    if (policy_preparation_.valid() ||
        pending_policy_name_.empty() ||
        pending_policy_name_ == active_policy_name_)
    {
        return;
    }

    const PolicyConfig policy = policies_.at(pending_policy_name_);

    if (!observations_ready(policy, robot_config_.sensor_timeout)) {
        return;
    }

    auto snapshot = capture_observation_snapshot();
    auto session = policy_runtimes_.at(policy.name).session;
    const std::uint64_t request_id = policy_request_id_;

    preparation_request_id_ = request_id;
    policy_preparation_ = std::async(
        std::launch::async,
        [this,
         policy,
         snapshot = std::move(snapshot),
         session = std::move(session),
         request_id]() -> PreparedPolicy
        {
            PreparedPolicy prepared;
            prepared.name = policy.name;
            prepared.request_id = request_id;
            prepared.runtime.session = session;

            // Allocate independent buffers for this activation attempt.
            reset_policy_runtime(policy, prepared.runtime);

            prepared.runtime.position_targets.assign(
                policy.controlled_joints.size(), 0.0);

            const std::array<float, 3> zero_command{
                0.0f, 0.0f, 0.0f
            };

            compute_actions(
                policy,
                prepared.runtime,
                snapshot,
                zero_command);

            prepare_joint_targets(policy, prepared.runtime);

            return prepared;
        });
}

std::optional<PolicyManager::PreparedPolicy>
PolicyManager::collect_policy_preparation()
{
    if (!policy_preparation_.valid()) {
        return std::nullopt;
    }

    // Poll without waiting for the background task.
    if (policy_preparation_.wait_for(std::chrono::seconds(0)) !=
        std::future_status::ready)
    {
        return std::nullopt;
    }

    try {
        // Consume the result and release the future for another request.
        PreparedPolicy prepared = policy_preparation_.get();

        if (prepared.request_id != policy_request_id_ ||
            prepared.name != pending_policy_name_)
        {
            return std::nullopt;
        }

        const auto& policy = policies_.at(prepared.name);

        // Keep the request pending if sensor availability has changed.
        if (!observations_ready(policy, robot_config_.sensor_timeout)) {
            return std::nullopt;
        }

        return prepared;
    }
    catch (const std::exception& error) {
        RCLCPP_ERROR(
            get_logger(),
            "Policy preparation failed: %s",
            error.what());

        // An obsolete failure must not cancel a newer request.
        if (preparation_request_id_ == policy_request_id_) {
            pending_policy_name_.clear();
        }

        return std::nullopt;
    }
}

bool 
PolicyManager::commit_prepared_policy(PreparedPolicy&& prepared)
{
    if (prepared.request_id != policy_request_id_ ||
        prepared.name != pending_policy_name_)
    {
        return false;
    }

    const auto& policy = policies_.at(prepared.name);

    if (!observations_ready(policy, robot_config_.sensor_timeout)) {
        return false;
    }

    auto& stored_runtime = policy_runtimes_.at(prepared.name);
    rclcpp::TimerBase::SharedPtr next_timer;
    std::optional<ActivationBackup> backup;

    try {
        // Preserve the running policy before preparing the replacement timer.
        if (!active_policy_name_.empty()) {
            backup.emplace(ActivationBackup{
                active_policy_name_,
                velocity_command_,
                inference_timer_
            });
        }
        const long double period_ns =
            1'000'000'000.0L / policy.frequency_hz;

        if (!std::isfinite(period_ns) ||
            period_ns < 1.0L ||
            period_ns >= static_cast<long double>(
                std::chrono::nanoseconds::max().count()))
        {
            throw std::runtime_error(
                "Unsupported inference period for policy: " + prepared.name);
        }

        const auto period = std::chrono::nanoseconds(
            static_cast<std::chrono::nanoseconds::rep>(period_ns));

        next_timer = create_wall_timer(
            period,
            [this]()
            {
                inference_cycle();
            });

        // Callbacks cannot overlap within the default callback group.
        next_timer->cancel();
        next_timer->reset();
    }
    catch (const std::exception& error) {
        pending_policy_name_.clear();

        RCLCPP_ERROR(
            get_logger(),
            "Could not prepare inference timer for '%s': %s",
            prepared.name.c_str(),
            error.what());

        return false;
    }

    activation_backup_ = std::move(backup);
    // Replace execution state only after timer preparation succeeds.
    if (inference_timer_) {
        inference_timer_->cancel();
    }

    std::swap(stored_runtime, prepared.runtime);
    active_policy_name_.swap(prepared.name);
    inference_timer_.swap(next_timer);

    velocity_command_.fill(0.0f);
    pending_policy_name_.clear();

    publish_active_policy();

    RCLCPP_INFO(
        get_logger(),
        "Activated policy '%s'",
        active_policy_name_.c_str());

    return true;
}

bool 
PolicyManager::process_policy_request()
{
    // Complete or roll back the current activation before starting another.
    if (activation_backup_.has_value()) {
        return false;
    }
    try {
        auto prepared = collect_policy_preparation();

        if (prepared &&
            commit_prepared_policy(std::move(*prepared)))
        {
            return true;
        }

        // Start a pending request when the previous task has been collected.
        start_policy_preparation();
    }
    catch (const std::exception& error) {
        pending_policy_name_.clear();

        RCLCPP_ERROR(
            get_logger(),
            "Policy request processing failed: %s",
            error.what());
    }

    return false;
}

void
PolicyManager::supervision_cycle()
{
    // Running policies handle requests within their inference cycle.
    if (!active_policy_name_.empty()) {
        return;
    }

    if (process_policy_request()) {
        // Run the first inference immediately after activation.
        inference_cycle();
    }
}

bool 
PolicyManager::rollback_policy_activation()
{
    if (!activation_backup_) {
        return false;
    }

    auto& backup = *activation_backup_;

    if (!backup.timer || backup.policy_name.empty()) {
        throw std::runtime_error(
            "Cannot restore an incomplete activation backup");
    }

    // Restart the previous timer before replacing execution state.
    backup.timer->reset();

    if (inference_timer_) {
        inference_timer_->cancel();
    }

    active_policy_name_.swap(backup.policy_name);
    inference_timer_.swap(backup.timer);
    velocity_command_ = backup.velocity_command;

    // Previous policy buffers remain stored in policy_runtimes_.
    activation_backup_.reset();

    publish_active_policy();

    RCLCPP_WARN(
        get_logger(),
        "Restored previous policy '%s'",
        active_policy_name_.c_str());

    return true;
}

}  // namespace g1_locomotion