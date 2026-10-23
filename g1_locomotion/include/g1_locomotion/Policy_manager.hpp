#ifndef G1_LOCOMOTION__POLICY_MANAGER_HPP_
#define G1_LOCOMOTION__POLICY_MANAGER_HPP_


#include "rclcpp/rclcpp.hpp"

#include "g1_locomotion/msg/policy_list.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <string>
#include <vector>
#include <unordered_map>
#include <cstddef>
#include <memory>
#include <onnxruntime_cxx_api.h>
#include <chrono>
#include <array>
#include <future>
#include <cstdint>
#include <optional>

namespace g1_locomotion
{

class PolicyManager : public rclcpp::Node
{
public:
    PolicyManager();

private:
    // Stores the configuration read from a policy YAML file.
    struct PolicyConfig
    {
        std::string name;
        std::string model_path;
        double frequency_hz{50.0};

        std::vector<std::string> controlled_joints;
        std::vector<std::string> observed_joints;

        std::string input_name{"obs"};
        std::string output_name{"actions"};

        std::vector<std::string> observation_order;

        // Computed from the observation layout, not read from YAML.
        std::size_t observation_size{0};
    };

    // Stores shared settings for all locomotion policies.
    struct RobotConfig
    {
        std::string default_policy;
        // Maximum time without a valid sensor update.
        std::chrono::milliseconds sensor_timeout{200};
        std::unordered_map<std::string, float> default_positions;
        std::unordered_map<std::string, float> action_scales;
    };

    // Holds the loaded model and its policy-specific inference state.
    struct PolicyRuntime
    {
        // Shares the loaded model while keeping inference buffers independent.
        std::shared_ptr<Ort::Session> session;
        std::vector<float> previous_actions;
        std::vector<float> observation;
        // Stores the latest validated network output.
        std::vector<float> actions;
        // Stores joint position targets computed from validated actions.
        std::vector<double> position_targets;
    };

    // Stores the latest complete measurement for one joint.
    struct JointMeasurement
    {
        double position{0.0};
        double velocity{0.0};
        bool valid{false};

        std::chrono::steady_clock::time_point last_update{};
    };

    // Stores IMU-derived observations in the robot base frame.
    struct ImuMeasurement
    {
        std::array<float, 3> angular_velocity{};
        std::array<float, 3> projected_gravity{};

        bool angular_velocity_valid{false};
        bool projected_gravity_valid{false};

        std::chrono::steady_clock::time_point angular_velocity_updated_at{};
        std::chrono::steady_clock::time_point projected_gravity_updated_at{};
    };

    // Stores the measured base linear velocity.
    struct BaseVelocityMeasurement
    {
        std::array<float, 3> linear_velocity{};
        bool valid{false};

        std::chrono::steady_clock::time_point last_update{};
    };

    // Owns a snapshot of sensor measurements for policy preparation.
    struct ObservationSnapshot
    {
        std::unordered_map<std::string, JointMeasurement> joints;
        ImuMeasurement imu;
        BaseVelocityMeasurement base_velocity;
    };

    // Holds a candidate prepared for one specific activation request.
    struct PreparedPolicy
    {
        std::string name;
        std::uint64_t request_id{0};
        PolicyRuntime runtime;
    };
    // Preserves the previous execution until the new policy completes a cycle.
    struct ActivationBackup
    {
        std::string policy_name;
        std::array<float, 3> velocity_command{};
        rclcpp::TimerBase::SharedPtr timer;
    };

    // Policy catalog management

    // Publishes the names of loaded and validated policies.
    void publish_available_policies();
    // Loads and validates policy configurations during startup.
    void load_policies();
    PolicyConfig load_policy_config(const std::string& yaml_path);
    // Loads shared joint settings from robot.yaml.
    void load_robot_config();
    // Validates policy joint lists against the shared robot configuration.
    void validate_policy_joints(const PolicyConfig& policy) const;
    // Validates the observation blocks supported by the current adapter.
    void validate_observation_order(const PolicyConfig& policy) const;
    // Computes the number of scalar values in the observation.
    std::size_t calculate_observation_size(const PolicyConfig& policy) const;
    // Loads ONNX models and validates their input/output interfaces.
    void load_policy_models();
    // Publishes the currently active policy name.
    void publish_active_policy();
    // Creates one position publisher per joint controlled by any policy.
    void create_joint_publishers();
    void build_observation(const PolicyConfig& policy, PolicyRuntime& runtime, const ObservationSnapshot& snapshot, const std::array<float, 3>& command);
    // Checks the validity and freshness of the required sensor observations.
    bool observations_ready(const PolicyConfig& policy, std::chrono::steady_clock::duration max_age) const;
    // Runs inference using the supplied velocity command.
    void compute_actions(const PolicyConfig& policy, PolicyRuntime& runtime, const ObservationSnapshot& snapshot, const std::array<float, 3>& command);
    // Clears inference history before starting a new policy execution.
    void reset_policy_runtime(const PolicyConfig& policy, PolicyRuntime& runtime);
    // Computes and validates all position targets without publishing.
    void prepare_joint_targets(const PolicyConfig& policy, PolicyRuntime& runtime);
    // Publishes targets previously prepared and validated.
    void publish_joint_targets(const PolicyConfig& policy, const PolicyRuntime& runtime);
    // Copies sensor state from the mutually exclusive ROS callback group.
    ObservationSnapshot capture_observation_snapshot() const;
    // Starts background preparation for the latest pending request.
    void start_policy_preparation();
    // Collects a completed preparation if it matches the current request.
    std::optional<PreparedPolicy> collect_policy_preparation();
    // Commits a prepared policy at a control-cycle boundary.
    bool commit_prepared_policy(PreparedPolicy&& prepared);
    // Advances preparation and commits completed policy requests.
    bool process_policy_request();
    // Handles startup activation while no policy is running.
    void supervision_cycle();
    // Restores the previous policy after an unsuccessful activation.
    bool rollback_policy_activation();

    // ROS callbacks
    void policy_callback(const std_msgs::msg::String::ConstSharedPtr msg);
    void imu_callback(const sensor_msgs::msg::Imu::ConstSharedPtr msg);
    void odometry_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg);
    void twist_callback(const geometry_msgs::msg::Twist::ConstSharedPtr msg);
    void joint_state_callback(const sensor_msgs::msg::JointState::ConstSharedPtr msg);
    // Executes one control cycle for the active policy.
    void inference_cycle();

    // Atributes

    // Stores the names of available policies.
    std::vector<std::string> available_policy_names_;
    // Maps each unique policy name to its configuration.
    std::unordered_map<std::string, PolicyConfig> policies_;
    // Policy selection state
    std::string active_policy_name_;
    std::string pending_policy_name_;
    // Shared joint settings.
    RobotConfig robot_config_;
    ImuMeasurement imu_measurement_;
    BaseVelocityMeasurement base_velocity_measurement_;
    // Holds the latest command and resets on policy activation.
    std::array<float, 3> velocity_command_{0.0f, 0.0f, 0.0f};
    // Declare the environment before sessions so it outlives them.
    Ort::Env ort_env_{ORT_LOGGING_LEVEL_WARNING, "g1_locomotion"};
    Ort::SessionOptions session_options_;

    // Stores loaded models and inference buffers by policy name.
    std::unordered_map<std::string, PolicyRuntime> policy_runtimes_;
    // Stores absolute joint measurements independently of the active policy.
    std::unordered_map<std::string, JointMeasurement> joint_measurements_;
    // Identifies the latest accepted policy request.
    std::uint64_t policy_request_id_{0};
    // Identifies the request associated with the current background task.
    std::uint64_t preparation_request_id_{0};
    std::optional<ActivationBackup> activation_backup_;
    
    // ROS publishers
    rclcpp::Publisher<g1_locomotion::msg::PolicyList>::SharedPtr available_policies_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr active_policy_pub_;
    std::unordered_map<std::string,rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr> joint_publishers_;
    
    // ROS subscriptions
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr policy_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
    
    // ROS timers
    rclcpp::TimerBase::SharedPtr inference_timer_;
    rclcpp::TimerBase::SharedPtr supervision_timer_;
    
    // Tracks the single background preparation currently in progress.
    std::future<PreparedPolicy> policy_preparation_;
};

}  // namespace g1_locomotion

#endif  // G1_LOCOMOTION__POLICY_MANAGER_HPP_