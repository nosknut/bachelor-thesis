// Based on https://github.com/ros-planning/moveit2/blob/humble/moveit_ros/moveit_servo/src/teleop_demo/joystick_servo_example.cpp

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
const std::string JOY_TOPIC = "/joy";

const std::string SHOULDER_TOPIC = "/shoulder/servo/velocity/cmd";
const std::string WRIST_TOPIC = "/wrist/servo/velocity/cmd";
const std::string GRIPPER_TOPIC = "/gripper/servo/velocity/cmd";

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

const std::string PARAM_NUM_AXIS = "num_axis";
const std::string PARAM_NUM_BUTTONS = "num_buttons";

const std::string PARAM_LEFT_STICK_X = "joy.axis.left_stick_x";
const std::string PARAM_LEFT_STICK_Y = "joy.axis.left_stick_y";
const std::string PARAM_RIGHT_STICK_X = "joy.axis.right_stick_x";
const std::string PARAM_RIGHT_STICK_Y = "joy.axis.right_stick_y";
const std::string PARAM_LEFT_BUMPER_AXIS = "joy.axis.left_bumper";
const std::string PARAM_RIGHT_BUMPER_AXIS = "joy.axis.right_bumper";
const std::string PARAM_DPAD_X = "joy.axis.dpad_x";
const std::string PARAM_DPAD_Y = "joy.axis.dpad_y";

const std::string PARAM_SQUARE = "joy.button.square";
const std::string PARAM_CROSS = "joy.button.cross";
const std::string PARAM_CIRCLE = "joy.button.circle";
const std::string PARAM_TRIANGLE = "joy.button.triangle";
const std::string PARAM_LEFT_TRIGGER = "joy.button.left_trigger";
const std::string PARAM_RIGHT_TRIGGER = "joy.button.right_trigger";
const std::string PARAM_LEFT_BUMPER = "joy.button.left_bumper";
const std::string PARAM_RIGHT_BUMPER = "joy.button.right_bumper";
const std::string PARAM_SHARE = "joy.button.share";
const std::string PARAM_OPTIONS = "joy.button.options";
const std::string PARAM_LEFT_JOYSTICK_BUTTON = "joy.button.left_joystick_button";
const std::string PARAM_RIGHT_JOYSTICK_BUTTON = "joy.button.right_joystick_button";
const std::string PARAM_PS_BUTTON = "joy.button.ps_button";

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

    this->declare_parameter(PARAM_NUM_AXIS, 4);
    this->declare_parameter(PARAM_NUM_BUTTONS, 18);

    // Joy Axis
    this->declare_parameter(PARAM_LEFT_STICK_X, 0);
    this->declare_parameter(PARAM_LEFT_STICK_Y, 1);
    this->declare_parameter(PARAM_LEFT_BUMPER_AXIS, 2);
    this->declare_parameter(PARAM_RIGHT_STICK_X, 3);
    this->declare_parameter(PARAM_RIGHT_STICK_Y, 4);
    this->declare_parameter(PARAM_RIGHT_BUMPER_AXIS, 5);
    this->declare_parameter(PARAM_DPAD_X, 6);
    this->declare_parameter(PARAM_DPAD_Y, 7);

    // Joy Buttons
    this->declare_parameter(PARAM_CROSS, 0);
    this->declare_parameter(PARAM_CIRCLE, 1);
    this->declare_parameter(PARAM_TRIANGLE, 2);
    this->declare_parameter(PARAM_SQUARE, 3);
    this->declare_parameter(PARAM_LEFT_TRIGGER, 4);
    this->declare_parameter(PARAM_RIGHT_TRIGGER, 5);
    this->declare_parameter(PARAM_LEFT_BUMPER, 6);
    this->declare_parameter(PARAM_RIGHT_BUMPER, 7);
    this->declare_parameter(PARAM_SHARE, 8);
    this->declare_parameter(PARAM_OPTIONS, 9);
    this->declare_parameter(PARAM_PS_BUTTON, 10);
    this->declare_parameter(PARAM_LEFT_JOYSTICK_BUTTON, 11);
    this->declare_parameter(PARAM_RIGHT_JOYSTICK_BUTTON, 12);

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

  void joyCallback(const sensor_msgs::msg::Joy::ConstSharedPtr & msg)
  {
    if (msg->axes.size() < getIntParam(PARAM_NUM_AXIS)) {
      RCLCPP_WARN(
        this->get_logger(),
        "Expected at least %d axes, but got %d. If Foxglove is being used to publish the joy topic, this warning can likely be ignored.",
        getIntParam(PARAM_NUM_AXIS),
        int(msg->axes.size()));
      return;
    }

    if (msg->buttons.size() < getIntParam(PARAM_NUM_BUTTONS)) {
      RCLCPP_WARN(
        this->get_logger(),
        "Expected at least %d buttons, but got %d. If Foxglove is being used to publish the joy topic, this warning can likely be ignored.",
        getIntParam(PARAM_NUM_BUTTONS),
        int(msg->buttons.size()));
      return;
    }

    if (msg->buttons[getIntParam(PARAM_OPTIONS)]) {
      if (!activate_servos_was_pressed) {
        activateServos();
        activate_servos_was_pressed = true;
      }
    } else {
      activate_servos_was_pressed = false;
    }

    if (msg->buttons[getIntParam(PARAM_SHARE)]) {
      if (!deactivate_servos_was_pressed) {
        deactivateServos();
        deactivate_servos_was_pressed = true;
      }
    } else {
      deactivate_servos_was_pressed = false;
    }

    // Right bumper must be held down while moving
    if (!msg->buttons[getIntParam(PARAM_RIGHT_BUMPER)]) {
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
          msg->axes[getIntParam(PARAM_LEFT_STICK_Y)])));

    wrist_pub_->publish(
      std_msgs::msg::Float32().set__data(
        getDoubleParam(PARAM_SHOULDER_MAX_SPEED) *
        ignoreDeadbandDirect(
          getDoubleParam(PARAM_DEADBAND),
          msg->axes[getIntParam(PARAM_LEFT_STICK_X)])));

    gripper_pub_->publish(
      std_msgs::msg::Float32().set__data(
        getDoubleParam(PARAM_SHOULDER_MAX_SPEED) *
        ignoreDeadbandDirect(
          getDoubleParam(PARAM_DEADBAND),
          msg->axes[getIntParam(PARAM_RIGHT_STICK_Y)])));
  }
};
}

// Register the component with class_loader
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(twig_teleop::JoystickDirectTeleop)
