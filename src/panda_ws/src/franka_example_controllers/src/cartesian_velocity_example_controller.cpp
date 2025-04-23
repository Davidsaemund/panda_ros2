#include <franka_example_controllers/cartesian_velocity_example_controller.hpp>
#include <franka_example_controllers/default_robot_behavior_utils.hpp>

#include <cassert>
#include <cmath>
#include <exception>
#include <string>
#include <iostream>
#include <Eigen/Eigen>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>

using namespace std::chrono_literals;

namespace franka_example_controllers {

controller_interface::InterfaceConfiguration
CartesianVelocityExampleController::command_interface_configuration() const {
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names = franka_cartesian_velocity_->get_command_interface_names();

  return config;
}

controller_interface::InterfaceConfiguration
CartesianVelocityExampleController::state_interface_configuration() const {
  return controller_interface::InterfaceConfiguration{
      controller_interface::interface_configuration_type::NONE};
}
double y_offset_ = 1.0;
controller_interface::return_type CartesianVelocityExampleController::update(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& period) {
  elapsed_time_ = elapsed_time_ + period;


  static double v_y = 0.0;
  static double v_step = 0.00001*2;
  static double v_max = 0.02*2;

  switch (classify_result_) {
      case 0:
      if (v_y > 0.0) {
        v_y -= v_step;
        if (v_y < 0.0) {
          v_y = 0.0; // Ensure v_y does not go below 0
        }
      } else if (v_y < 0.0) {
        v_y += v_step;
        if (v_y > 0.0) {
          v_y = 0.0; // Ensure v_y does not go above 0
        }
      }
      break;
    case 1:
      if (v_y < v_max) {
        v_y += v_step;
      }
      if (v_y > v_max) {
        v_y = v_max; // Ensure v_y does not exceed v_max
      }
      break;
    case 2:
      if (v_y > 0.0) {
        v_y -= v_step;
        if (v_y < 0.0) {
          v_y = 0.0; // Ensure v_y does not go below 0
        }
      } else if (v_y < 0.0) {
        v_y += v_step;
        if (v_y > 0.0) {
          v_y = 0.0; // Ensure v_y does not go above 0
        }
      }
      break;
    case 3:
      if (v_y > -v_max) {
        v_y -= v_step;
      }
      if (v_y < -v_max) {
        v_y = -v_max; // Ensure v_y does not exceed -v_max
      }
      break;
    default:
      break;
  }

  Eigen::Vector3d cartesian_linear_velocity(0.0, v_y, 0.0);
  Eigen::Vector3d cartesian_angular_velocity(0.0, 0.0, 0.0);

  static int counter = 0;
  if (counter++ % 100 == 0) {
    RCLCPP_INFO_STREAM(get_node()->get_logger(), "The value is: " << v_y << ", Classify results: " << classify_result_ );
  }

  if (franka_cartesian_velocity_->setCommand(cartesian_linear_velocity,
                                             cartesian_angular_velocity)) {
    return controller_interface::return_type::OK;
  } else {
    RCLCPP_FATAL(get_node()->get_logger(),
                 "Set command failed. Did you activate the elbow command interface?");
    return controller_interface::return_type::ERROR;
  }
}

CallbackReturn CartesianVelocityExampleController::on_init() {
  return CallbackReturn::SUCCESS;
}

CallbackReturn CartesianVelocityExampleController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  franka_cartesian_velocity_ =
      std::make_unique<franka_semantic_components::FrankaCartesianVelocityInterface>(
          franka_semantic_components::FrankaCartesianVelocityInterface(k_elbow_activated_));

  auto client = get_node()->create_client<franka_msgs::srv::SetFullCollisionBehavior>(
      "service_server/set_full_collision_behavior");
  auto request = DefaultRobotBehavior::getDefaultCollisionBehaviorRequest();

  auto future_result = client->async_send_request(request);
  future_result.wait_for(1000ms);

  auto success = future_result.get();
  if (!success) {
    RCLCPP_FATAL(get_node()->get_logger(), "Failed to set default collision behavior.");
    return CallbackReturn::ERROR;
  } else {
    RCLCPP_INFO(get_node()->get_logger(), "Default collision behavior set.");
  }

  classify_results_subscriber_ = get_node()->create_subscription<std_msgs::msg::Int32>(
    "classify_results",
    10,
    std::bind(&CartesianVelocityExampleController::classifyResultsCallback, this, std::placeholders::_1)
  );
  return CallbackReturn::SUCCESS;
}

CallbackReturn CartesianVelocityExampleController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  franka_cartesian_velocity_->assign_loaned_command_interfaces(command_interfaces_);
  elapsed_time_ = rclcpp::Duration(0, 0);
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CartesianVelocityExampleController::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  franka_cartesian_velocity_->release_interfaces();
  return CallbackReturn::SUCCESS;
}

void CartesianVelocityExampleController::classifyResultsCallback(const std_msgs::msg::Int32::SharedPtr msg) {
  classify_result_ = msg->data;
}

}  // namespace franka_example_controllers

#include "pluginlib/class_list_macros.hpp"
// NOLINTNEXTLINE
PLUGINLIB_EXPORT_CLASS(franka_example_controllers::CartesianVelocityExampleController,
                       controller_interface::ControllerInterface)
