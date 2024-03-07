// Based on https://github.com/ros-planning/moveit2/blob/humble/moveit_ros/moveit_servo/src/teleop_demo/joystick_servo_example.cpp

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
const std::string JOY_TOPIC = "/joy";
const std::string JOINT_TOPIC = "/cmd_jog";

const std::string PARAM_JOG_FRAME_ID = "jog_frame_id";

const std::string PARAM_SHOULDER_MAX_SPEED = "shoulder_max_speed";
const std::string PARAM_WRIST_MAX_SPEED = "wrist_max_speed";
const std::string PARAM_GRIPPER_MAX_SPEED = "gripper_max_speed";

const std::string PARAM_DEADBAND = "joy.deadband";

const std::string PARAM_LEFT_STICK_X = "joy.axis.left_stick_x";
const std::string PARAM_LEFT_STICK_Y = "joy.axis.left_stick_y";
const std::string PARAM_RIGHT_STICK_X = "joy.axis.right_stick_x";
const std::string PARAM_RIGHT_STICK_Y = "joy.axis.right_stick_y";

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
const std::string PARAM_DPAD_UP = "joy.button.dpad_up";
const std::string PARAM_DPAD_DOWN = "joy.button.dpad_down";
const std::string PARAM_DPAD_LEFT = "joy.button.dpad_left";
const std::string PARAM_DPAD_RIGHT = "joy.button.dpad_right";
const std::string PARAM_PS_BUTTON = "joy.button.ps_button";
const std::string PARAM_TOUCHPAD = "joy.button.touchpad";

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
    this->declare_parameter(PARAM_LEFT_STICK_X, 0);
    this->declare_parameter(PARAM_LEFT_STICK_Y, 1);
    this->declare_parameter(PARAM_RIGHT_STICK_X, 2);
    this->declare_parameter(PARAM_RIGHT_STICK_Y, 3);

    // Joy Buttons
    this->declare_parameter(PARAM_SQUARE, 2);
    this->declare_parameter(PARAM_CROSS, 0);
    this->declare_parameter(PARAM_CIRCLE, 1);
    this->declare_parameter(PARAM_TRIANGLE, 3);
    this->declare_parameter(PARAM_LEFT_TRIGGER, 4);
    this->declare_parameter(PARAM_RIGHT_TRIGGER, 5);
    this->declare_parameter(PARAM_LEFT_BUMPER, 6);
    this->declare_parameter(PARAM_RIGHT_BUMPER, 7);
    this->declare_parameter(PARAM_SHARE, 8);
    this->declare_parameter(PARAM_OPTIONS, 9);
    this->declare_parameter(PARAM_LEFT_JOYSTICK_BUTTON, 10);
    this->declare_parameter(PARAM_RIGHT_JOYSTICK_BUTTON, 11);
    this->declare_parameter(PARAM_DPAD_UP, 12);
    this->declare_parameter(PARAM_DPAD_DOWN, 13);
    this->declare_parameter(PARAM_DPAD_LEFT, 14);
    this->declare_parameter(PARAM_DPAD_RIGHT, 15);
    this->declare_parameter(PARAM_PS_BUTTON, 16);
    this->declare_parameter(PARAM_TOUCHPAD, 17);

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

  void triggerServo() {
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

  void convertJoyToJog(
    const std::vector<float> & axes,
    const std::vector<int> & buttons,
    std::unique_ptr<control_msgs::msg::JointJog> & jog)
  {
    jog->joint_names.push_back("twig_shoulder_joint");
    jog->velocities.push_back(
      getDoubleParam(PARAM_SHOULDER_MAX_SPEED) *
      ignoreDeadband(getDoubleParam(PARAM_DEADBAND), axes[getIntParam(PARAM_LEFT_STICK_Y)]));

    jog->joint_names.push_back("twig_wrist_joint");
    jog->velocities.push_back(
      getDoubleParam(PARAM_WRIST_MAX_SPEED) *
      ignoreDeadband(getDoubleParam(PARAM_DEADBAND), axes[getIntParam(PARAM_LEFT_STICK_X)]));

    jog->joint_names.push_back("twig_left_finger_joint");
    jog->velocities.push_back(
      getDoubleParam(PARAM_GRIPPER_MAX_SPEED) *
      ignoreDeadband(getDoubleParam(PARAM_DEADBAND), axes[getIntParam(PARAM_RIGHT_STICK_Y)]));
  }

  void joyCallback(const sensor_msgs::msg::Joy::ConstSharedPtr & msg)
  {
    auto joint_msg = std::make_unique<control_msgs::msg::JointJog>();

    // Trigger the servo start service when the options button is pressed
    if (msg->buttons[getIntParam(PARAM_OPTIONS)]) {
      if (!trigger_servo_was_pressed) {
        triggerServo();
        trigger_servo_was_pressed = true;
      }
    } else {
      trigger_servo_was_pressed = false;
    }

    // Right bumper must be held down while moving
    if (!msg->buttons[getIntParam(PARAM_RIGHT_BUMPER)]) {
      return;
    }

    convertJoyToJog(msg->axes, msg->buttons, joint_msg);
    joint_msg->header.stamp = this->now();
    joint_msg->header.frame_id = getParam(PARAM_JOG_FRAME_ID);
    joint_pub_->publish(std::move(joint_msg));
  }
};
}

// Register the component with class_loader
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(twig_teleop::JoystickTeleop)
