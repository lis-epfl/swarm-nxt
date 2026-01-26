/**
 * @file command_converter_node.cpp
 * @brief Converts MPC controller output to swarmnxt_msgs/ControllerCommand
 * Handles VehicleRatesSetpoint, VehicleTorqueSetpoint, VehicleThrustSetpoint,
 * and ActuatorMotors messages
 */

#include <cmath>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <px4_msgs/msg/actuator_motors.hpp>
#include <px4_msgs/msg/vehicle_rates_setpoint.hpp>
#include <px4_msgs/msg/vehicle_thrust_setpoint.hpp>
#include <px4_msgs/msg/vehicle_torque_setpoint.hpp>
#include <rclcpp/rclcpp.hpp>
#include <swarmnxt_msgs/msg/controller_command.hpp>

class CommandConverter : public rclcpp::Node {
public:
  CommandConverter() : Node("command_converter") {
    // Declare parameters
    this->declare_parameter<std::string>("rates_input_topic",
                                         "/fmu/in/vehicle_rates_setpoint");
    this->declare_parameter<std::string>("torque_input_topic",
                                         "/fmu/in/vehicle_torque_setpoint");
    this->declare_parameter<std::string>("thrust_input_topic",
                                         "/fmu/in/vehicle_thrust_setpoint");
    this->declare_parameter<std::string>("motors_input_topic",
                                         "/fmu/in/actuator_motors");
    this->declare_parameter<std::string>("output_topic", "/controller/cmd");
    this->declare_parameter<double>("sync_tolerance_sec",
                                    0.001); // 1ms tolerance for message sync

    // Get parameters
    std::string rates_input =
        this->get_parameter("rates_input_topic").as_string();
    std::string torque_input =
        this->get_parameter("torque_input_topic").as_string();
    std::string thrust_input =
        this->get_parameter("thrust_input_topic").as_string();
    std::string motors_input =
        this->get_parameter("motors_input_topic").as_string();
    std::string output_topic = this->get_parameter("output_topic").as_string();
    double sync_tolerance =
        this->get_parameter("sync_tolerance_sec").as_double();

    // Create regular subscribers for rates and motors
    rates_sub_ = this->create_subscription<px4_msgs::msg::VehicleRatesSetpoint>(
        rates_input, 10,
        std::bind(&CommandConverter::ratesCallback, this,
                  std::placeholders::_1));

    motors_sub_ = this->create_subscription<px4_msgs::msg::ActuatorMotors>(
        motors_input, 10,
        std::bind(&CommandConverter::motorsCallback, this,
                  std::placeholders::_1));

    // Create message filter subscribers for torque/thrust synchronization
    torque_sub_filter_ = std::make_shared<
        message_filters::Subscriber<px4_msgs::msg::VehicleTorqueSetpoint>>(
        this, torque_input);
    thrust_sub_filter_ = std::make_shared<
        message_filters::Subscriber<px4_msgs::msg::VehicleThrustSetpoint>>(
        this, thrust_input);

    // Create approximate time synchronizer for torque and thrust
    // The '10' parameter is the queue size
    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
        SyncPolicy(10), *torque_sub_filter_, *thrust_sub_filter_);

    // Set the tolerance for approximate time synchronization
    sync_->setInterMessageLowerBound(
        0, rclcpp::Duration::from_seconds(sync_tolerance));
    sync_->setInterMessageLowerBound(
        1, rclcpp::Duration::from_seconds(sync_tolerance));

    // Register callback for synchronized messages
    sync_->registerCallback(std::bind(&CommandConverter::torqueThrustCallback,
                                      this, std::placeholders::_1,
                                      std::placeholders::_2));

    // Create publisher
    pub_ = this->create_publisher<swarmnxt_msgs::msg::ControllerCommand>(
        output_topic, 10);

    RCLCPP_INFO(this->get_logger(), "Command Converter initialized");
    RCLCPP_INFO(this->get_logger(), "  Rates input topic: %s",
                rates_input.c_str());
    RCLCPP_INFO(this->get_logger(), "  Torque input topic: %s",
                torque_input.c_str());
    RCLCPP_INFO(this->get_logger(), "  Thrust input topic: %s",
                thrust_input.c_str());
    RCLCPP_INFO(this->get_logger(), "  Motors input topic: %s",
                motors_input.c_str());
    RCLCPP_INFO(this->get_logger(), "  Output topic: %s", output_topic.c_str());
    RCLCPP_INFO(this->get_logger(), "  Sync tolerance: %.3f sec",
                sync_tolerance);
  }

private:
  // Define the synchronization policy type
  typedef message_filters::sync_policies::ApproximateTime<
      px4_msgs::msg::VehicleTorqueSetpoint,
      px4_msgs::msg::VehicleThrustSetpoint>
      SyncPolicy;

  void ratesCallback(const px4_msgs::msg::VehicleRatesSetpoint::SharedPtr msg) {
    // Create output message
    auto out_msg = swarmnxt_msgs::msg::ControllerCommand();

    // Set header
    out_msg.header.stamp = this->get_clock()->now();
    out_msg.header.frame_id = "base_link";

    // Set command type to RATE_SETPOINT
    out_msg.command_type_mask =
        swarmnxt_msgs::msg::ControllerCommand::RATE_SETPOINT;

    // Fill in rate command array [roll_rate, pitch_rate, yaw_rate]
    out_msg.rate_cmd.clear();
    out_msg.rate_cmd.push_back(msg->roll);  // roll rate (rad/s)
    out_msg.rate_cmd.push_back(msg->pitch); // pitch rate (rad/s)
    out_msg.rate_cmd.push_back(msg->yaw);   // yaw rate (rad/s)

    // Thrust - already in correct format from controller
    double thrust = msg->thrust_body[2];
    out_msg.thrust_cmd = thrust;

    // Clear unused arrays
    out_msg.torque_cmd.clear();
    out_msg.motor_cmd.clear();

    // Publish converted message
    pub_->publish(out_msg);

    RCLCPP_DEBUG(
        this->get_logger(),
        "Converted rate command: roll=%.3f, pitch=%.3f, yaw=%.3f, thrust=%.3f",
        out_msg.rate_cmd[0], out_msg.rate_cmd[1], out_msg.rate_cmd[2],
        out_msg.thrust_cmd);
  }

  void torqueThrustCallback(
      const px4_msgs::msg::VehicleTorqueSetpoint::ConstSharedPtr &torque_msg,
      const px4_msgs::msg::VehicleThrustSetpoint::ConstSharedPtr &thrust_msg) {

    // Create output message
    auto out_msg = swarmnxt_msgs::msg::ControllerCommand();

    // Set header - use the timestamp from the torque message (could use either)
    out_msg.header.stamp = this->get_clock()->now();
    out_msg.header.frame_id = "base_link";

    // Set command type to TORQUE_SETPOINT
    out_msg.command_type_mask =
        swarmnxt_msgs::msg::ControllerCommand::TORQUE_SETPOINT;

    // Fill in torque command array [roll_torque, pitch_torque, yaw_torque]
    out_msg.torque_cmd.clear();
    out_msg.torque_cmd.push_back(torque_msg->xyz[0]); // roll torque (Nm)
    out_msg.torque_cmd.push_back(torque_msg->xyz[1]); // pitch torque (Nm)
    out_msg.torque_cmd.push_back(torque_msg->xyz[2]); // yaw torque (Nm)

    // Thrust - already in correct format from controller
    double thrust = thrust_msg->xyz[2];
    out_msg.thrust_cmd = thrust;

    // Clear unused arrays
    out_msg.rate_cmd.clear();
    out_msg.motor_cmd.clear();

    // Publish converted message
    pub_->publish(out_msg);

    RCLCPP_DEBUG(this->get_logger(),
                 "Synchronized torque/thrust command: roll_torque=%.3f, "
                 "pitch_torque=%.3f, yaw_torque=%.3f, thrust=%.3f",
                 out_msg.torque_cmd[0], out_msg.torque_cmd[1],
                 out_msg.torque_cmd[2], out_msg.thrust_cmd);
  }

  void motorsCallback(const px4_msgs::msg::ActuatorMotors::SharedPtr msg) {
    // Create output message
    auto out_msg = swarmnxt_msgs::msg::ControllerCommand();

    // Set header
    out_msg.header.stamp = this->get_clock()->now();
    out_msg.header.frame_id = "base_link";

    // Set command type to MOTOR_SETPOINT
    out_msg.command_type_mask =
        swarmnxt_msgs::msg::ControllerCommand::MOTOR_SETPOINT;

    // Convert motor commands
    // PX4 ActuatorMotors has control[12] array with normalized values [0, 1]
    // or NaN for unused motors
    out_msg.motor_cmd.clear();

    for (size_t i = 0; i < 12; ++i) {
      float motor_value = msg->control[i];

      // Skip NaN values (unused motors)
      if (std::isnan(motor_value)) {
        break; // Assume all motors after first NaN are also unused
      }

      // Keep normalized [0, 1] values, just clamp to valid range
      motor_value = std::clamp(motor_value, 0.0f, 1.0f);
      out_msg.motor_cmd.push_back(static_cast<double>(motor_value));
    }

    // For motor commands, thrust_cmd is typically not used but set to 0
    out_msg.thrust_cmd = 0.0;

    // Clear unused arrays
    out_msg.rate_cmd.clear();
    out_msg.torque_cmd.clear();

    // Publish converted message
    pub_->publish(out_msg);

    RCLCPP_DEBUG(this->get_logger(), "Converted motor command: %zu motors",
                 out_msg.motor_cmd.size());
  }

  // Regular subscribers for rates and motors
  rclcpp::Subscription<px4_msgs::msg::VehicleRatesSetpoint>::SharedPtr
      rates_sub_;
  rclcpp::Subscription<px4_msgs::msg::ActuatorMotors>::SharedPtr motors_sub_;

  // Message filter subscribers for synchronized torque/thrust
  std::shared_ptr<
      message_filters::Subscriber<px4_msgs::msg::VehicleTorqueSetpoint>>
      torque_sub_filter_;
  std::shared_ptr<
      message_filters::Subscriber<px4_msgs::msg::VehicleThrustSetpoint>>
      thrust_sub_filter_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

  // Publisher
  rclcpp::Publisher<swarmnxt_msgs::msg::ControllerCommand>::SharedPtr pub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CommandConverter>());
  rclcpp::shutdown();
  return 0;
}
