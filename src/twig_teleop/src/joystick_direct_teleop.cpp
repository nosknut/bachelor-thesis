// Based on https://github.com/ros-planning/moveit2/blob/humble/moveit_ros/moveit_servo/src/teleop_demo/twig_joystick_servo_example.cpp

#include <sensor_msgs/msg/joy.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/float32.hpp>
#include <rclcpp/client.hpp>
#include <rclcpp/experimental/buffers/intra_process_buffer.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/qos_event.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp/utilities.hpp>

const std::string NODE_NAME = "joystick_teleop";
const std::string JOY_TOPIC = "/twig_joy";

const std::string SHOULDER_TOPIC = "/shoulder/servo/velocity/cmd";
const std::string WRIST_TOPIC = "/wrist/servo/velocity/cmd";
const std::string GRIPPER_TOPIC = "/gripper/servo/velocity/cmd";

const std::string ACKNOWLEDGE_HARDWARE_REBOOT_TOPIC = "/acknowledge_hardware_reboot";

const std::string ACTIVATE_SHOULDER_SERVO_TOPIC = "/shoulder/servo/activate";
const std::string ACTIVATE_WRIST_SERVO_TOPIC = "/wrist/servo/activate";
const std::string ACTIVATE_GRIPPER_SERVO_TOPIC = "/gripper/servo/activate";

const std::string DEACTIVATE_SHOULDER_SERVO_TOPIC = "/shoulder/servo/deactivate";
const std::string DEACTIVATE_WRIST_SERVO_TOPIC = "/wrist/servo/deactivate";
const std::string DEACTIVATE_GRIPPER_SERVO_TOPIC = "/gripper/servo/deactivate";

const std::string PARAM_SHOULDER_MAX_SPEED = "shoulder_max_speed";
const std::string PARAM_WRIST_MAX_SPEED = "wrist_max_speed";
const std::string PARAM_GRIPPER_MAX_SPEED = "gripper_max_speed";

const std::string PARAM_DEADBAND = "joy.deadband";

const std::string PARAM_DEADMANSWITCH_BUTTON = "joy.button.deadmanswitch";
const std::string PARAM_ACTIVATE_SERVOS_BUTTON = "joy.button.activate_servos";
const std::string PARAM_DEACTIVATE_SERVOS_BUTTON = "joy.button.deactivate_servos";
const std::string PARAM_SHOULDER_AXIS = "joy.axis.shoulder";
const std::string PARAM_WRIST_AXIS = "joy.axis.wrist";
const std::string PARAM_GRIPPER_AXIS = "joy.axis.gripper";

enum CommandType
{
  TWIST = 0,
  JOG = 1
};

namespace twig_teleop
{

float ignoreDeadbandDirect(float deadband, float value)
{
  if (abs(value) < deadband) {
    return 0.0;
  }

  // Map the range [deadband, 1.0] to [0.0, 1.0]
  return value / (1.0 - deadband) + (value > 0 ? -deadband : deadband);
}

class JoystickDirectTeleop : public rclcpp::Node
{
private:
  bool activate_servos_was_pressed = false;
  bool deactivate_servos_was_pressed = false;

  bool arm_button_was_pressed = false;

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr shoulder_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr wrist_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr gripper_pub_;

  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr acknowledge_hardware_reboot_client_;

  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr shoulder_servo_activate_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr wrist_servo_activate_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr gripper_servo_activate_client_;

  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr shoulder_servo_deactivate_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr wrist_servo_deactivate_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr gripper_servo_deactivate_client_;

public:
  JoystickDirectTeleop(const rclcpp::NodeOptions & options)
  : Node(NODE_NAME, options)
  {

    this->declare_parameter(PARAM_SHOULDER_MAX_SPEED, 1.0);
    this->declare_parameter(PARAM_WRIST_MAX_SPEED, 1.0);
    this->declare_parameter(PARAM_GRIPPER_MAX_SPEED, 1.0);

    this->declare_parameter(PARAM_DEADBAND, 0.05);

    // Joy Axis
    this->declare_parameter(PARAM_SHOULDER_AXIS, 1);
    this->declare_parameter(PARAM_WRIST_AXIS, 0);
    this->declare_parameter(PARAM_GRIPPER_AXIS, 3);

    // Joy Buttons
    this->declare_parameter(PARAM_DEADMANSWITCH_BUTTON, 7);
    this->declare_parameter(PARAM_ACTIVATE_SERVOS_BUTTON, 8);
    this->declare_parameter(PARAM_DEACTIVATE_SERVOS_BUTTON, 9);

    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      JOY_TOPIC,
      rclcpp::SystemDefaultsQoS(),
      [this](const sensor_msgs::msg::Joy::ConstSharedPtr & msg) {
        return joyCallback(msg);
      }
    );

    shoulder_pub_ = this->create_publisher<std_msgs::msg::Float32>(
      SHOULDER_TOPIC,
      rclcpp::SystemDefaultsQoS());

    wrist_pub_ = this->create_publisher<std_msgs::msg::Float32>(
      WRIST_TOPIC,
      rclcpp::SystemDefaultsQoS());

    gripper_pub_ = this->create_publisher<std_msgs::msg::Float32>(
      GRIPPER_TOPIC,
      rclcpp::SystemDefaultsQoS());

    acknowledge_hardware_reboot_client_ = this->create_client<std_srvs::srv::Trigger>(
      ACKNOWLEDGE_HARDWARE_REBOOT_TOPIC);

    shoulder_servo_activate_client_ = this->create_client<std_srvs::srv::Trigger>(
      ACTIVATE_SHOULDER_SERVO_TOPIC);
    wrist_servo_activate_client_ = this->create_client<std_srvs::srv::Trigger>(
      ACTIVATE_WRIST_SERVO_TOPIC);
    gripper_servo_activate_client_ = this->create_client<std_srvs::srv::Trigger>(
      ACTIVATE_GRIPPER_SERVO_TOPIC);

    shoulder_servo_deactivate_client_ = this->create_client<std_srvs::srv::Trigger>(
      DEACTIVATE_SHOULDER_SERVO_TOPIC);
    wrist_servo_deactivate_client_ = this->create_client<std_srvs::srv::Trigger>(
      DEACTIVATE_WRIST_SERVO_TOPIC);
    gripper_servo_deactivate_client_ = this->create_client<std_srvs::srv::Trigger>(
      DEACTIVATE_GRIPPER_SERVO_TOPIC);
  }

  void activateServos()
  {
    shoulder_servo_activate_client_->wait_for_service(std::chrono::seconds(1));
    shoulder_servo_activate_client_->async_send_request(
      std::make_shared<std_srvs::srv::Trigger::Request>());

    wrist_servo_activate_client_->wait_for_service(std::chrono::seconds(1));
    wrist_servo_activate_client_->async_send_request(
      std::make_shared<std_srvs::srv::Trigger::Request>());

    gripper_servo_activate_client_->wait_for_service(std::chrono::seconds(1));
    gripper_servo_activate_client_->async_send_request(
      std::make_shared<std_srvs::srv::Trigger::Request>());
  }

  void deactivateServos()
  {
    shoulder_servo_deactivate_client_->wait_for_service(std::chrono::seconds(1));
    shoulder_servo_deactivate_client_->async_send_request(
      std::make_shared<std_srvs::srv::Trigger::Request>());

    wrist_servo_deactivate_client_->wait_for_service(std::chrono::seconds(1));
    wrist_servo_deactivate_client_->async_send_request(
      std::make_shared<std_srvs::srv::Trigger::Request>());

    gripper_servo_deactivate_client_->wait_for_service(std::chrono::seconds(1));
    gripper_servo_deactivate_client_->async_send_request(
      std::make_shared<std_srvs::srv::Trigger::Request>());

    acknowledge_hardware_reboot_client_->wait_for_service(std::chrono::seconds(1));
    acknowledge_hardware_reboot_client_->async_send_request(
      std::make_shared<std_srvs::srv::Trigger::Request>());
  }

  std::string getParam(const std::string & param_name)
  {
    return this->get_parameter(param_name).as_string();
  }

  int getIntParam(const std::string & param_name)
  {
    return this->get_parameter(param_name).as_int();
  }

  double getDoubleParam(const std::string & param_name)
  {
    return this->get_parameter(param_name).as_double();
  }

  std::optional<bool> getButton(
    const sensor_msgs::msg::Joy::ConstSharedPtr & msg,
    std::string button)
  {
    auto button_id = getIntParam(button);
    if (msg->buttons.size() < button_id) {
      RCLCPP_WARN(
        this->get_logger(),
        "Expected at least %d buttons, but got %d. If Foxglove is being used to publish the joy topic, this warning can likely be ignored.",
        button_id,
        int(msg->buttons.size()));
      return {};
    }
    return msg->buttons[button_id];
  }

  std::optional<float> getAxis(const sensor_msgs::msg::Joy::ConstSharedPtr & msg, std::string axis)
  {
    auto axis_id = getIntParam(axis);
    if (msg->axes.size() < axis_id) {
      RCLCPP_WARN(
        this->get_logger(),
        "Expected at least %d axes, but got %d. If Foxglove is being used to publish the joy topic, this warning can likely be ignored.",
        axis_id,
        int(msg->axes.size()));
      return {};
    }
    return msg->axes[axis_id];
  }

  void joyCallback(const sensor_msgs::msg::Joy::ConstSharedPtr & msg)
  {
    if (getButton(msg, PARAM_ACTIVATE_SERVOS_BUTTON).value_or(false)) {
      if (!activate_servos_was_pressed) {
        activateServos();
        activate_servos_was_pressed = true;
      }
    } else {
      activate_servos_was_pressed = false;
    }

    if (getButton(msg, PARAM_DEACTIVATE_SERVOS_BUTTON).value_or(false)) {
      if (!deactivate_servos_was_pressed) {
        deactivateServos();
        deactivate_servos_was_pressed = true;
      }
    } else {
      deactivate_servos_was_pressed = false;
    }

    // Right bumper must be held down while moving
    if (!getButton(msg, PARAM_DEADMANSWITCH_BUTTON).value_or(false)) {
      if (arm_button_was_pressed) {
        shoulder_pub_->publish(std_msgs::msg::Float32().set__data(0));
        wrist_pub_->publish(std_msgs::msg::Float32().set__data(0));
        gripper_pub_->publish(std_msgs::msg::Float32().set__data(0));
        arm_button_was_pressed = false;
      }
      return;
    }
    arm_button_was_pressed = true;

    shoulder_pub_->publish(
      std_msgs::msg::Float32().set__data(
        getDoubleParam(PARAM_SHOULDER_MAX_SPEED) *
        ignoreDeadbandDirect(
          getDoubleParam(PARAM_DEADBAND),
          getAxis(msg, PARAM_SHOULDER_AXIS).value_or(0))));

    wrist_pub_->publish(
      std_msgs::msg::Float32().set__data(
        getDoubleParam(PARAM_WRIST_MAX_SPEED) *
        ignoreDeadbandDirect(
          getDoubleParam(PARAM_DEADBAND),
          getAxis(msg, PARAM_WRIST_AXIS).value_or(0))));

    gripper_pub_->publish(
      std_msgs::msg::Float32().set__data(
        getDoubleParam(PARAM_GRIPPER_MAX_SPEED) *
        ignoreDeadbandDirect(
          getDoubleParam(PARAM_DEADBAND),
          getAxis(msg, PARAM_GRIPPER_AXIS).value_or(0))));
  }
};
}

// Register the component with class_loader
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(twig_teleop::JoystickDirectTeleop)
