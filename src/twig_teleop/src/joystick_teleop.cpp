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
const std::string TWIST_TOPIC = "/cmd_vel";
const std::string JOINT_TOPIC = "/cmd_jog";

const std::string PARAM_BASE_FRAME_ID = "base_frame_id";
const std::string PARAM_END_EFFECTOR_FRAME_ID = "end_effector_frame_id";
const std::string PARAM_JOG_FRAME_ID = "jog_frame_id";

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

class JoystickTeleop : public rclcpp::Node
{
private:
  bool wasJog_ = false;
  std::string twist_frame_ = "";
  std::thread collision_pub_thread_;

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr joint_pub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr servo_start_client_;

public:
  JoystickTeleop(const rclcpp::NodeOptions & options)
  : Node(NODE_NAME, options)
  {
    this->declare_parameter(PARAM_BASE_FRAME_ID, "panda_link0");
    this->declare_parameter(PARAM_END_EFFECTOR_FRAME_ID, "panda_hand");
    this->declare_parameter(PARAM_JOG_FRAME_ID, "panda_link3");

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

    twist_frame_ = getParam(PARAM_BASE_FRAME_ID);

    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      JOY_TOPIC,
      rclcpp::SystemDefaultsQoS(),
      [this](const sensor_msgs::msg::Joy::ConstSharedPtr & msg) {
        return joyCallback(msg);
      }
    );
ws://localhost:8765 f
oxglove_bridge
    twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
      TWIST_TOPIC,
      rclcpp::SystemDefaultsQoS());

    joint_pub_ = this->create_publisher<control_msgs::msg::JointJog>(
      JOINT_TOPIC,
      rclcpp::SystemDefaultsQoS());

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

  /** \brief // This converts a joystick axes and buttons array to a bool indicating whether to publish a Twist or JointJog message
   * @param axes The vector of continuous controller joystick axes
   * @param buttons The vector of discrete controller button values
   * @return return TWIST og JOG depending on the axes and buttons
   */
  CommandType joyToCommandType(
    const std::vector<float> & axes,
    const std::vector<int> & buttons,
    bool & wasJog)
  {
    // Give joint jogging priority because it is only buttons
    // If any joint jog command is requested, we are only publishing joint commands
    if (
      buttons[getIntParam(PARAM_SQUARE)] ||
      buttons[getIntParam(PARAM_CIRCLE)] ||
      buttons[getIntParam(PARAM_CROSS)] ||
      buttons[getIntParam(PARAM_TRIANGLE)] ||
      buttons[getIntParam(PARAM_DPAD_LEFT)] ||
      buttons[getIntParam(PARAM_DPAD_RIGHT)] ||
      buttons[getIntParam(PARAM_DPAD_UP)] ||
      buttons[getIntParam(PARAM_DPAD_DOWN)])
    {
      wasJog = true;
      return JOG;
    }

    // Ensures that when jog buttons are released a zero command is sent exactly once
    if (wasJog) {
      wasJog = false;
      return JOG;
    }

    return TWIST;
  }

  /** \brief // This converts a joystick axes and buttons array to a TwistStamped message
   * @param axes The vector of continuous controller joystick axes
   * @param buttons The vector of discrete controller button values
   * @param twist A TwistStamped message to update in prep for publishing
   */
  void convertJoyToTwist(
    const std::vector<float> & axes,
    const std::vector<int> & buttons,
    std::unique_ptr<geometry_msgs::msg::TwistStamped> & twist)
  {
    twist->twist.linear.z = axes[getIntParam(PARAM_RIGHT_STICK_Y)];
    twist->twist.linear.y = axes[getIntParam(PARAM_RIGHT_STICK_X)];

    double lin_x_right = -0.5 * buttons[getIntParam(PARAM_RIGHT_TRIGGER)];
    double lin_x_left = 0.5 * buttons[getIntParam(PARAM_LEFT_TRIGGER)];
    twist->twist.linear.x = lin_x_right + lin_x_left;

    twist->twist.angular.y = axes[getIntParam(PARAM_LEFT_STICK_Y)];
    twist->twist.angular.x = -axes[getIntParam(PARAM_LEFT_STICK_X)];

    double roll_positive = buttons[getIntParam(PARAM_RIGHT_BUMPER)];
    double roll_negative = -1 * (buttons[getIntParam(PARAM_LEFT_BUMPER)]);
    twist->twist.angular.z = roll_positive + roll_negative;
  }

  float buttonToAxis(bool positiveButton, bool negativeButton)
  {
    if (positiveButton) {
      return 1;
    }

    if (negativeButton) {
      return -1;
    }

    return 0;
  }

  /** \brief // This converts a joystick axes and buttons array to a JointJog message
   * @param axes The vector of continuous controller joystick axes
   * @param buttons The vector of discrete controller button values
   * @param jog A JointJog message to update in prep for publishing
   */
  void convertJoyToJog(
    const std::vector<float> & axes,
    const std::vector<int> & buttons,
    std::unique_ptr<geometry_msgs::msg::TwistStamped> & twist,
    std::unique_ptr<control_msgs::msg::JointJog> & jog)
  {
    jog->joint_names.push_back("panda_joint1");
    jog->velocities.push_back(
      buttonToAxis(
        buttons[getIntParam(PARAM_DPAD_LEFT)],
        buttons[getIntParam(PARAM_DPAD_RIGHT)]));

    jog->joint_names.push_back("panda_joint2");
    jog->velocities.push_back(
      buttonToAxis(
        buttons[getIntParam(PARAM_DPAD_UP)],
        buttons[getIntParam(PARAM_DPAD_DOWN)]));

    jog->joint_names.push_back("panda_joint7");
    jog->velocities.push_back(
      buttonToAxis(
        buttons[getIntParam(PARAM_TRIANGLE)],
        buttons[getIntParam(PARAM_CROSS)]));

    jog->joint_names.push_back("panda_joint6");
    jog->velocities.push_back(
      buttonToAxis(
        buttons[getIntParam(PARAM_SQUARE)],
        buttons[getIntParam(PARAM_CIRCLE)]));
  }


  /** \brief // This should update the frame_to_publish_ as needed for changing command frame via controller
   * @param frame_name Set the command frame to this
   * @param buttons The vector of discrete controller button values
   */
  void updateTwistFrame(
    std::string & frame_name,
    const std::vector<int> & buttons)
  {
    if (buttons[getIntParam(PARAM_SHARE)] && frame_name == getParam(PARAM_END_EFFECTOR_FRAME_ID)) {
      frame_name = getParam(PARAM_BASE_FRAME_ID);
    } else if (buttons[getIntParam(PARAM_OPTIONS)] && frame_name == getParam(PARAM_BASE_FRAME_ID)) {
      frame_name = getParam(PARAM_END_EFFECTOR_FRAME_ID);
    }
  }

  void joyCallback(const sensor_msgs::msg::Joy::ConstSharedPtr & msg)
  {
    auto twist_msg = std::make_unique<geometry_msgs::msg::TwistStamped>();
    auto joint_msg = std::make_unique<control_msgs::msg::JointJog>();

    updateTwistFrame(twist_frame_, msg->buttons);

    switch (joyToCommandType(msg->axes, msg->buttons, wasJog_)) {
      case TWIST:
        convertJoyToTwist(msg->axes, msg->buttons, twist_msg);
        twist_msg->header.frame_id = twist_frame_;
        twist_msg->header.stamp = this->now();
        twist_pub_->publish(std::move(twist_msg));
        break;
      case JOG:
        convertJoyToJog(msg->axes, msg->buttons, twist_msg, joint_msg);
        joint_msg->header.stamp = this->now();
        joint_msg->header.frame_id = getParam(PARAM_JOG_FRAME_ID);
        joint_pub_->publish(std::move(joint_msg));
        break;
    }
  }
};
}

// Register the component with class_loader
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(twig_teleop::JoystickTeleop)
