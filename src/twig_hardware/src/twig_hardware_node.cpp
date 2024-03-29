#include "twig_hardware/twig_lib.hpp"

#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
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
#include <std_srvs/srv/trigger.hpp>

const std::string NODE_NAME = "twig_hardware_node";

const std::string PARAM_PUBLISH_RATE = "publish_rate";
const std::string PARAM_PUSH_RATE = "push_rate";
const std::string PARAM_COMMAND_TIMEOUT = "command_timeout";

namespace twig_hardware
{

class TwigHardwareNode : public rclcpp::Node
{
protected:
  TwigLib twig;
  rclcpp::TimerBase::SharedPtr publish_timer_;
  rclcpp::TimerBase::SharedPtr push_timer_;
  rclcpp::TimerBase::SharedPtr command_reset_timer;

  long int publish_period_ = 1e6;
  long int push_period_ = 1e6;

  bool has_new_shoulder_command_ = false;
  bool has_new_wrist_command_ = false;
  bool has_new_gripper_command_ = false;

  bool read_without_subscribers_ = true;
  bool publish_without_subscribers_ = false;

  // Activate Services

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr shoulder_servo_activate_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr wrist_servo_activate_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr gripper_servo_activate_srv_;

  // Deactivate Services

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr shoulder_servo_deactivate_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr wrist_servo_deactivate_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr gripper_servo_deactivate_srv_;

  // velocity Subscribers

  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr shoulder_servo_velocity_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr wrist_servo_velocity_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr gripper_servo_velocity_sub_;

  // Activation Status Publishers

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr shoulder_servo_activation_status_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr wrist_servo_activation_status_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr gripper_servo_activation_status_pub_;

  // Current Publishers

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr shoulder_servo_current_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr wrist_servo_current_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr gripper_servo_current_pub_;

  // Voltage Publishers

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr shoulder_servo_voltage_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr wrist_servo_voltage_pub_;

  // Velocity Publishers

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr shoulder_servo_velocity_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr wrist_servo_velocity_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr gripper_servo_velocity_pub_;

  // Position Publishers

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr shoulder_servo_position_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr wrist_servo_position_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr gripper_servo_position_pub_;

  // Effort Publishers

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr shoulder_servo_effort_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr wrist_servo_effort_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr gripper_servo_effort_pub_;

  // Helpers

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr create_float_publisher(
    const std::string & topic_name)
  {
    return this->create_publisher<std_msgs::msg::Float32>(
      topic_name,
      rclcpp::SystemDefaultsQoS()
    );
  }

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr create_bool_publisher(
    const std::string & topic_name)
  {
    return this->create_publisher<std_msgs::msg::Bool>(
      topic_name,
      rclcpp::SystemDefaultsQoS()
    );
  }

  void publish_float(const rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr & pub, double value)
  {
    pub->publish(std_msgs::msg::Float32().set__data(value));
  }

  void publish_state()
  {
    // Activation Status Publishers
    shoulder_servo_activation_status_pub_->publish(
      std_msgs::msg::Bool().set__data(twig.get_shoulder_servo_activation_status()));

    wrist_servo_activation_status_pub_->publish(
      std_msgs::msg::Bool().set__data(twig.get_wrist_servo_activation_status()));

    gripper_servo_activation_status_pub_->publish(
      std_msgs::msg::Bool().set__data(twig.get_gripper_servo_activation_status()));

    // Current Publishers
    publish_float(shoulder_servo_current_pub_, twig.get_shoulder_servo_current());
    publish_float(wrist_servo_current_pub_, twig.get_wrist_servo_current());
    publish_float(gripper_servo_current_pub_, twig.get_gripper_servo_current());

    // Voltage Publishers
    publish_float(shoulder_servo_voltage_pub_, twig.get_shoulder_servo_voltage());
    publish_float(wrist_servo_voltage_pub_, twig.get_wrist_servo_voltage());

    // Velocity Publishers
    publish_float(shoulder_servo_velocity_pub_, twig.get_shoulder_servo_velocity());
    publish_float(wrist_servo_velocity_pub_, twig.get_wrist_servo_velocity());
    publish_float(gripper_servo_velocity_pub_, twig.get_gripper_servo_velocity());

    // Position Publishers
    publish_float(shoulder_servo_position_pub_, twig.get_shoulder_servo_position());
    publish_float(wrist_servo_position_pub_, twig.get_wrist_servo_position());
    publish_float(gripper_servo_position_pub_, twig.get_gripper_servo_position());

    // Effort Publishers
    publish_float(shoulder_servo_effort_pub_, twig.get_shoulder_servo_effort());
    publish_float(wrist_servo_effort_pub_, twig.get_wrist_servo_effort());
    publish_float(gripper_servo_effort_pub_, twig.get_gripper_servo_effort());
  }

  int get_total_subscriber_count()
  {
    int count = 0;

    // Activation Status Publishers
    count += shoulder_servo_activation_status_pub_->get_subscription_count();
    count += wrist_servo_activation_status_pub_->get_subscription_count();
    count += gripper_servo_activation_status_pub_->get_subscription_count();

    // Current Publishers
    count += shoulder_servo_current_pub_->get_subscription_count();
    count += wrist_servo_current_pub_->get_subscription_count();
    count += gripper_servo_current_pub_->get_subscription_count();

    // Voltage Publishers
    count += shoulder_servo_voltage_pub_->get_subscription_count();
    count += wrist_servo_voltage_pub_->get_subscription_count();

    // Velocity Publishers
    count += shoulder_servo_velocity_pub_->get_subscription_count();
    count += wrist_servo_velocity_pub_->get_subscription_count();
    count += gripper_servo_velocity_pub_->get_subscription_count();

    // Position Publishers
    count += shoulder_servo_position_pub_->get_subscription_count();
    count += wrist_servo_position_pub_->get_subscription_count();
    count += gripper_servo_position_pub_->get_subscription_count();

    // Effort Publishers
    count += shoulder_servo_effort_pub_->get_subscription_count();
    count += wrist_servo_effort_pub_->get_subscription_count();
    count += gripper_servo_effort_pub_->get_subscription_count();

    return count;
  }

  void publish_timer_callback()
  {
    bool has_subscribers = get_total_subscriber_count() > 0;
    if (has_subscribers || read_without_subscribers_) {
      if (twig.read_state(3)) {
        if (has_subscribers || publish_without_subscribers_) {
          publish_state();
        }
      } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to read state from hardware");
      }
    }
  }

  void push_timer_callback()
  {
      if (!twig.write_command(3)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to write command to hardware");
      }
      has_new_shoulder_command_ = false;
      has_new_wrist_command_ = false;
      has_new_gripper_command_ = false;
  }

  void command_timeout_callback()
  {
    if (!has_new_shoulder_command_) {
      twig.set_shoulder_servo_velocity(0);
    }

    if (!has_new_wrist_command_) {
      twig.set_wrist_servo_velocity(0);
    }

    if (!has_new_gripper_command_) {
      twig.set_gripper_servo_velocity(0);
    }
  }

public:
  TwigHardwareNode(const rclcpp::NodeOptions & options)
  : Node(NODE_NAME, options)
  {

    this->declare_parameter(PARAM_PUBLISH_RATE, 50);
    this->declare_parameter(PARAM_PUSH_RATE, 50);
    this->declare_parameter(PARAM_COMMAND_TIMEOUT, 500);

    // Activate Services
    // Black magic that allows running services with class instance methods is based on:
    // https://github.com/ros2/demos/blob/humble/demo_nodes_cpp/src/services/add_two_ints_server.cpp#L36-L48
    shoulder_servo_activate_srv_ = create_service<std_srvs::srv::Trigger>(
      "shoulder/servo/activate",
      [this](
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response
      ) -> void {
        twig.activate_shoulder_servo();

        if (twig.write_command()) {
          response->success = true;
          response->message = "Successfully activated shoulder servo";
        } else {
          response->success = false;
          response->message = "Failed to write command to hardware";
          RCLCPP_ERROR_ONCE(this->get_logger(), "Failed to write command to hardware");
        }
      });

    wrist_servo_activate_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "wrist/servo/activate",
      [this](
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response
      ) -> void {
        twig.activate_wrist_servo();

        if (twig.write_command()) {
          response->success = true;
          response->message = "Successfully activated wrist servo";
        } else {
          response->success = false;
          response->message = "Failed to write command to hardware";
          RCLCPP_ERROR_ONCE(this->get_logger(), "Failed to write command to hardware");
        }
      });

    gripper_servo_activate_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "gripper/servo/activate",
      [this](
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response
      ) -> void {
        twig.activate_gripper_servo();

        if (twig.write_command()) {
          response->success = true;
          response->message = "Successfully activated gripper servo";
        } else {
          response->success = false;
          response->message = "Failed to write command to hardware";
          RCLCPP_ERROR_ONCE(this->get_logger(), "Failed to write command to hardware");
        }
      });

    // Deactivate Services
    shoulder_servo_deactivate_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "shoulder/servo/deactivate",
      [this](
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response
      ) -> void {
        twig.deactivate_shoulder_servo();

        if (twig.write_command()) {
          response->success = true;
          response->message = "Successfully deactivated shoulder servo";
        } else {
          response->success = false;
          response->message = "Failed to write command to hardware";
          RCLCPP_ERROR_ONCE(this->get_logger(), "Failed to write command to hardware");
        }
      });

    wrist_servo_deactivate_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "wrist/servo/deactivate",
      [this](
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response
      ) -> void {
        twig.deactivate_wrist_servo();

        if (twig.write_command()) {
          response->success = true;
          response->message = "Successfully deactivated wrist servo";
        } else {
          response->success = false;
          response->message = "Failed to write command to hardware";
          RCLCPP_ERROR_ONCE(this->get_logger(), "Failed to write command to hardware");
        }
      });

    gripper_servo_deactivate_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "gripper/servo/deactivate",
      [this](
        const std::shared_ptr<rmw_request_id_t> request_header,
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response
      ) -> void {
        twig.deactivate_gripper_servo();

        if (twig.write_command()) {
          response->success = true;
          response->message = "Successfully deactivated gripper servo";
        } else {
          response->success = false;
          response->message = "Failed to write command to hardware";
          RCLCPP_ERROR_ONCE(this->get_logger(), "Failed to write command to hardware");
        }
      });

    // Velocity Subscribers
    shoulder_servo_velocity_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      "shoulder/servo/velocity/cmd",
      rclcpp::SystemDefaultsQoS(),
      [this](const std_msgs::msg::Float32::SharedPtr msg) {
        twig.set_shoulder_servo_velocity(msg->data);
        has_new_shoulder_command_ = true;
      }
    );

    wrist_servo_velocity_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      "wrist/servo/velocity/cmd",
      rclcpp::SystemDefaultsQoS(),
      [this](const std_msgs::msg::Float32::SharedPtr msg) {
        twig.set_wrist_servo_velocity(msg->data);
        has_new_wrist_command_ = true;
      }
    );

    gripper_servo_velocity_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      "gripper/servo/velocity/cmd",
      rclcpp::SystemDefaultsQoS(),
      [this](const std_msgs::msg::Float32::SharedPtr msg) {
        twig.set_gripper_servo_velocity(msg->data);
        has_new_gripper_command_ = true;
      }
    );

    // Activation Status Publishers
    shoulder_servo_activation_status_pub_ =
      create_bool_publisher("shoulder/servo/status/activation");
    wrist_servo_activation_status_pub_ = create_bool_publisher("wrist/servo/status/activation");
    gripper_servo_activation_status_pub_ = create_bool_publisher("gripper/servo/status/activation");

    // Current Publishers
    shoulder_servo_current_pub_ = create_float_publisher("shoulder/servo/current");
    wrist_servo_current_pub_ = create_float_publisher("wrist/servo/current");
    gripper_servo_current_pub_ = create_float_publisher("gripper/servo/current");

    // Voltage Publishers
    shoulder_servo_voltage_pub_ = create_float_publisher("shoulder/servo/voltage");
    wrist_servo_voltage_pub_ = create_float_publisher("wrist/servo/voltage");

    // Velocity Publishers
    shoulder_servo_velocity_pub_ = create_float_publisher("shoulder/servo/velocity");
    wrist_servo_velocity_pub_ = create_float_publisher("wrist/servo/velocity");
    gripper_servo_velocity_pub_ = create_float_publisher("gripper/servo/velocity");

    // Position Publishers
    shoulder_servo_position_pub_ = create_float_publisher("shoulder/servo/position");
    wrist_servo_position_pub_ = create_float_publisher("wrist/servo/position");
    gripper_servo_position_pub_ = create_float_publisher("gripper/servo/position");

    // Effort Publishers
    shoulder_servo_effort_pub_ = create_float_publisher("shoulder/servo/effort");
    wrist_servo_effort_pub_ = create_float_publisher("wrist/servo/effort");
    gripper_servo_effort_pub_ = create_float_publisher("gripper/servo/effort");

    // Timers
    publish_period_ = 1e6 / this->get_parameter(PARAM_PUBLISH_RATE).as_int();
    publish_timer_ = this->create_wall_timer(
      std::chrono::microseconds(publish_period_),
      std::bind(&TwigHardwareNode::publish_timer_callback, this)
    );

    push_period_ = 1e6 / this->get_parameter(PARAM_PUSH_RATE).as_int();
    push_timer_ = this->create_wall_timer(
      std::chrono::microseconds(push_period_),
      std::bind(&TwigHardwareNode::push_timer_callback, this)
    );

    command_reset_timer = this->create_wall_timer(
      std::chrono::milliseconds(this->get_parameter(PARAM_COMMAND_TIMEOUT).as_int()),
      std::bind(&TwigHardwareNode::command_timeout_callback, this)
    );
  }
};
}

// Register the component with class_loader
#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(twig_hardware::TwigHardwareNode)
