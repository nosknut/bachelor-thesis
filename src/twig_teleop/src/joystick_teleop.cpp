// Based on https://github.com/ros-planning/moveit2/blob/humble/moveit_ros/moveit_servo/src/teleop_demo/twig_joystick_servo_example.cpp

#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <control_msgs/msg/joint_jog.hpp>
#include <std_srvs/srv/trigger.hpp>
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
const std::string SERVO_START_SERVICE_NAME = "/start_servo";
const std::string JOY_TOPIC = "/twig_joy";
const std::string JOINT_TOPIC = "/cmd_jog";

const std::string PARAM_JOG_FRAME_ID = "jog_frame_id";

const std::string PARAM_SHOULDER_MAX_SPEED = "shoulder_max_speed";
const std::string PARAM_WRIST_MAX_SPEED = "wrist_max_speed";
const std::string PARAM_GRIPPER_MAX_SPEED = "gripper_max_speed";

const std::string PARAM_DEADBAND = "joy.deadband";

const std::string PARAM_DEADMANSWITCH_BUTTON = "joy.button.deadmanswitch";
const std::string PARAM_TRIGGER_SERVO_BUTTON = "joy.button.activate_servos";
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

float ignoreDeadband(float deadband, float value)
{
  if (abs(value) < deadband) {
    return 0.0;
  }

  // Map the range [deadband, 1.0] to [0.0, 1.0]
  return value / (1.0 - deadband) + (value > 0 ? -deadband : deadband);
}

class JoystickTeleop : public rclcpp::Node
{
private:
  bool trigger_servo_was_pressed = false;
  std::thread collision_pub_thread_;

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_pub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr servo_start_client_;

public:
  JoystickTeleop(const rclcpp::NodeOptions & options)
  : Node(NODE_NAME, options)
  {
    this->declare_parameter(PARAM_JOG_FRAME_ID, "base_link");

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
    this->declare_parameter(PARAM_TRIGGER_SERVO_BUTTON, 8);

    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      JOY_TOPIC,
      rclcpp::SystemDefaultsQoS(),
      [this](const sensor_msgs::msg::Joy::ConstSharedPtr & msg) {
        return joyCallback(msg);
      }
    );

    joint_pub_ = this->create_publisher<control_msgs::msg::JointJog>(
      JOINT_TOPIC,
      rclcpp::SystemDefaultsQoS());

    triggerServo();
  }

  void triggerServo()
  {
    // Create a service client to start the ServoNode
    servo_start_client_ = this->create_client<std_srvs::srv::Trigger>(SERVO_START_SERVICE_NAME);
    servo_start_client_->wait_for_service(std::chrono::seconds(1));
    servo_start_client_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
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

  void convertJoyToJog(
    const sensor_msgs::msg::Joy::ConstSharedPtr & msg,
    std::unique_ptr<control_msgs::msg::JointJog> & jog)
  {
    jog->joint_names.push_back("twig_shoulder_joint");
    jog->velocities.push_back(
      getDoubleParam(PARAM_SHOULDER_MAX_SPEED) *
      ignoreDeadband(
        getDoubleParam(PARAM_DEADBAND),
        getAxis(msg, PARAM_SHOULDER_AXIS).value_or(0)));

    jog->joint_names.push_back("twig_wrist_joint");
    jog->velocities.push_back(
      getDoubleParam(PARAM_WRIST_MAX_SPEED) *
      ignoreDeadband(getDoubleParam(PARAM_DEADBAND), getAxis(msg, PARAM_WRIST_AXIS).value_or(0)));

    jog->joint_names.push_back("twig_left_finger_joint");
    jog->velocities.push_back(
      getDoubleParam(PARAM_GRIPPER_MAX_SPEED) *
      ignoreDeadband(getDoubleParam(PARAM_DEADBAND), getAxis(msg, PARAM_GRIPPER_AXIS).value_or(0)));
  }

  void joyCallback(const sensor_msgs::msg::Joy::ConstSharedPtr & msg)
  {
    auto joint_msg = std::make_unique<control_msgs::msg::JointJog>();

    // Trigger the servo start service when the options button is pressed
    if (getButton(msg, PARAM_TRIGGER_SERVO_BUTTON).value_or(false)) {
      if (!trigger_servo_was_pressed) {
        triggerServo();
        trigger_servo_was_pressed = true;
      }
    } else {
      trigger_servo_was_pressed = false;
    }

    // Right bumper must be held down while moving
    if (!getButton(msg, PARAM_DEADMANSWITCH_BUTTON).value_or(false)) {
      return;
    }

    convertJoyToJog(msg, joint_msg);
    joint_msg->header.stamp = this->now();
    joint_msg->header.frame_id = getParam(PARAM_JOG_FRAME_ID);
    joint_pub_->publish(std::move(joint_msg));
  }
};
}

// Register the component with class_loader
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(twig_teleop::JoystickTeleop)
