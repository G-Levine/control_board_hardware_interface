#include "control_board_hardware_interface/control_board_hardware_interface.hpp"

#include <sched.h>
#include <sys/mman.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <time.h>
#include <unistd.h>

#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace control_board_hardware_interface {
ControlBoardHardwareInterface::~ControlBoardHardwareInterface() {
  // Deactivate everything when ctrl-c is pressed
  on_deactivate(rclcpp_lifecycle::State());
}

hardware_interface::CallbackReturn ControlBoardHardwareInterface::on_init(
    const hardware_interface::HardwareInfo &info) {
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  hw_state_positions_.resize(info_.joints.size(), 0.0);
  hw_state_velocities_.resize(info_.joints.size(), 0.0);
  hw_state_efforts_.resize(info_.joints.size(), 0.0);

  hw_command_positions_.resize(info_.joints.size(), 0.0);
  hw_command_velocities_.resize(info_.joints.size(), 0.0);
  hw_command_efforts_.resize(info_.joints.size(), 0.0);
  hw_command_kps_.resize(info_.joints.size(), 0.0);
  hw_command_kds_.resize(info_.joints.size(), 0.0);

  for (const hardware_interface::ComponentInfo &joint : info_.joints) {
    // Set params for each joint
    hw_actuator_can_channels_.push_back(std::stoi(joint.parameters.at("can_channel")));
    hw_actuator_can_ids_.push_back(std::stoi(joint.parameters.at("can_id")));

    // Set limits for each joint
    hw_actuator_position_mins_.push_back(std::stod(joint.parameters.at("position_min")));
    hw_actuator_position_maxs_.push_back(std::stod(joint.parameters.at("position_max")));
    hw_actuator_velocity_maxs_.push_back(std::stod(joint.parameters.at("velocity_max")));
    hw_actuator_effort_maxs_.push_back(std::stod(joint.parameters.at("effort_max")));
    hw_actuator_kp_maxs_.push_back(std::stod(joint.parameters.at("kp_max")));
    hw_actuator_kd_maxs_.push_back(std::stod(joint.parameters.at("kd_max")));

    // Homing parameters
    hw_actuator_homing_stages_.push_back(std::stoi(joint.parameters.at("homing_stage")));
    hw_actuator_homing_velocities_.push_back(std::stod(joint.parameters.at("homing_velocity")));
    hw_actuator_homing_kps_.push_back(std::stod(joint.parameters.at("homing_kp")));
    hw_actuator_homing_kds_.push_back(std::stod(joint.parameters.at("homing_kd")));
    hw_actuator_homed_positions_.push_back(std::stod(joint.parameters.at("homed_position")));
    hw_actuator_post_homing_positions_.push_back(
        std::stod(joint.parameters.at("post_homing_position")));
    hw_actuator_zero_positions_.push_back(0.0);
    hw_actuator_homing_torque_thresholds_.push_back(
        std::stod(joint.parameters.at("homing_torque_threshold")));
    hw_actuator_is_homed_.push_back(false);
  }

  imu_roll_ = std::stod(info_.sensors[0].parameters.at("roll"));
  imu_pitch_ = std::stod(info_.sensors[0].parameters.at("pitch"));
  imu_yaw_ = std::stod(info_.sensors[0].parameters.at("yaw"));

  // Set up the IMU
  //   imu_ = std::make_unique<BNO055>(IMU_I2C_DEVICE_NUMBER);

  // Set up SPI
  init_spi();
  spi_command_ = get_spi_command();
  spi_data_ = get_spi_data();

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
ControlBoardHardwareInterface::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // Add joint state interfaces
  for (auto i = 0u; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_state_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_state_velocities_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_state_efforts_[i]));
  }

  // Add IMU state interfaces
  state_interfaces.emplace_back(hardware_interface::StateInterface("imu_sensor", "orientation.x",
                                                                   &hw_state_imu_orientation_[0]));
  state_interfaces.emplace_back(hardware_interface::StateInterface("imu_sensor", "orientation.y",
                                                                   &hw_state_imu_orientation_[1]));
  state_interfaces.emplace_back(hardware_interface::StateInterface("imu_sensor", "orientation.z",
                                                                   &hw_state_imu_orientation_[2]));
  state_interfaces.emplace_back(hardware_interface::StateInterface("imu_sensor", "orientation.w",
                                                                   &hw_state_imu_orientation_[3]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      "imu_sensor", "angular_velocity.x", &hw_state_imu_angular_velocity_[0]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      "imu_sensor", "angular_velocity.y", &hw_state_imu_angular_velocity_[1]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      "imu_sensor", "angular_velocity.z", &hw_state_imu_angular_velocity_[2]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      "imu_sensor", "linear_acceleration.x", &hw_state_imu_linear_acceleration_[0]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      "imu_sensor", "linear_acceleration.y", &hw_state_imu_linear_acceleration_[1]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
      "imu_sensor", "linear_acceleration.z", &hw_state_imu_linear_acceleration_[2]));

  return state_interfaces;
}

hardware_interface::CallbackReturn ControlBoardHardwareInterface::on_configure(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  // reset values always when configuring hardware
  for (uint i = 0; i < hw_state_positions_.size(); i++) {
    hw_state_positions_[i] = 0.0;
    hw_state_velocities_[i] = 0.0;
    hw_state_efforts_[i] = 0.0;
    hw_command_positions_[i] = 0.0;
    hw_command_velocities_[i] = 0.0;
    hw_command_efforts_[i] = 0.0;
    hw_command_kps_[i] = 0.0;
    hw_command_kds_[i] = 0.0;
  }

  RCLCPP_INFO(rclcpp::get_logger("ControlBoardHardwareInterface"), "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::CommandInterface>
ControlBoardHardwareInterface::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_command_positions_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_command_velocities_[i]));
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_command_efforts_[i]));
    command_interfaces.emplace_back(
        hardware_interface::CommandInterface(info_.joints[i].name, "kp", &hw_command_kps_[i]));
    command_interfaces.emplace_back(
        hardware_interface::CommandInterface(info_.joints[i].name, "kd", &hw_command_kds_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn ControlBoardHardwareInterface::on_activate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  // copy_actuator_commands();
  // for (int i = 0; i < 10; i++)
  // {
  //     // Disable actuators
  //     spi_command_->flags[0] = 0;
  //     spi_command_->flags[1] = 0;
  //     spi_command_->flags[2] = 0;
  //     spi_command_->flags[3] = 0;
  //     spi_driver_run();
  // }

  // std::this_thread::sleep_for(std::chrono::milliseconds(100));

  for (int i = 0; i < 10; i++) {
    // Enable actuators
    spi_command_->flags[0] = 1;
    spi_command_->flags[1] = 1;
    spi_command_->flags[2] = 1;
    spi_command_->flags[3] = 1;
    spi_driver_run();
  }
  copy_actuator_states();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  do_homing();

  RCLCPP_INFO(rclcpp::get_logger("ControlBoardHardwareInterface"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ControlBoardHardwareInterface::on_deactivate(
    const rclcpp_lifecycle::State & /*previous_state*/) {
  // Disable actuators
  spi_command_->flags[0] = 0;
  spi_command_->flags[1] = 0;
  spi_command_->flags[2] = 0;
  spi_command_->flags[3] = 0;
  spi_driver_run();

  RCLCPP_INFO(rclcpp::get_logger("ControlBoardHardwareInterface"), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type ControlBoardHardwareInterface::read(
    const rclcpp::Time & /*time*/, const rclcpp::Duration &period) {
  // Write to and read from the actuators
  spi_driver_run();
  copy_actuator_states();

  // Print joint position
  // RCLCPP_INFO(rclcpp::get_logger("ControlBoardHardwareInterface"), "Joint 0: %f",
  // spi_data_->q_abad[1]);

  // Read the IMU
  //   imu_->sample(imu_output_);

  tf2::Quaternion imu_quat, offset_quat, corrected_quat;
  tf2::Matrix3x3 rotation_matrix;

  // Getting the original IMU quaternion
  //   imu_quat.setValue(imu_output_.quat.x(), imu_output_.quat.y(), imu_output_.quat.z(),
  //                     imu_output_.quat.w());

  // Setting the offset quaternion based on your YAW, PITCH, ROLL offsets
  offset_quat.setRPY(imu_roll_, imu_pitch_, imu_yaw_);

  // Applying the offset to the IMU quaternion
  corrected_quat = imu_quat * offset_quat.inverse();
  corrected_quat.normalize();  // Normalizing the quaternion to ensure it's a valid rotation

  rotation_matrix.setRotation(offset_quat);

  // Rotating the angular velocity
  tf2::Vector3 angular_velocity(imu_output_.gyro.x(), imu_output_.gyro.y(), imu_output_.gyro.z());
  angular_velocity = rotation_matrix * angular_velocity;

  // Rotating the linear acceleration
  tf2::Vector3 linear_acceleration(imu_output_.acc.x(), imu_output_.acc.y(), imu_output_.acc.z());
  linear_acceleration = rotation_matrix * linear_acceleration;

  // Updating the state interfaces with corrected values
  hw_state_imu_orientation_[0] = corrected_quat.x();
  hw_state_imu_orientation_[1] = corrected_quat.y();
  hw_state_imu_orientation_[2] = corrected_quat.z();
  hw_state_imu_orientation_[3] = corrected_quat.w();

  hw_state_imu_angular_velocity_[0] = angular_velocity.x();
  hw_state_imu_angular_velocity_[1] = angular_velocity.y();
  hw_state_imu_angular_velocity_[2] = angular_velocity.z();

  hw_state_imu_linear_acceleration_[0] = linear_acceleration.x();
  hw_state_imu_linear_acceleration_[1] = linear_acceleration.y();
  hw_state_imu_linear_acceleration_[2] = linear_acceleration.z();

  // Print the IMU
  // RCLCPP_INFO(rclcpp::get_logger("ControlBoardHardwareInterface"), "IMU: %f, %f, %f, %f, %f, %f,
  // %f, %f, %f, %f",
  //     imu_output_.quat.x(), imu_output_.quat.y(), imu_output_.quat.z(), imu_output_.quat.w(),
  //     imu_output_.acc.x(), imu_output_.acc.y(), imu_output_.acc.z(),
  //     imu_output_.gyro.x(), imu_output_.gyro.y(), imu_output_.gyro.z());

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ControlBoardHardwareInterface::write(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
  copy_actuator_commands(true);
  return hardware_interface::return_type::OK;
}

void ControlBoardHardwareInterface::do_homing() {
  RCLCPP_INFO(rclcpp::get_logger("ControlBoardHardwareInterface"), "Homing actuators...");

  int dt_ms = 10;

  std::this_thread::sleep_for(std::chrono::milliseconds(dt_ms));
  copy_actuator_commands();
  spi_driver_run();
  copy_actuator_states();
  std::this_thread::sleep_for(std::chrono::milliseconds(dt_ms));

  // Set the initial commands
  for (auto i = 0u; i < hw_state_positions_.size(); i++) {
    hw_command_positions_[i] = hw_state_positions_[i];
    hw_command_velocities_[i] = 0.0;
    hw_command_kps_[i] = 0.0;
    hw_command_kds_[i] = 0.0;
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(dt_ms));
  copy_actuator_commands();
  spi_driver_run();
  copy_actuator_states();
  std::this_thread::sleep_for(std::chrono::milliseconds(dt_ms));

  // Keep track of filtered torques
  std::vector<double> filtered_torques;
  filtered_torques.resize(hw_state_positions_.size(), 0.0);
  double alpha = 0.5;

  // Loop until all actuators are homed
  bool all_homed = false;
  bool all_returned = false;
  while (!all_homed) {
    // Set the homing stage to that of the lowest-stage unhomed actuator
    int homing_stage = 999;
    for (auto i = 0u; i < hw_actuator_homing_stages_.size(); i++) {
      if (!hw_actuator_is_homed_[i] && (hw_actuator_homing_stages_[i] < homing_stage)) {
        homing_stage = hw_actuator_homing_stages_[i];
      }
    }
    // Check if all actuators are homed
    all_homed = true;
    for (auto i = 0u; i < hw_state_positions_.size(); i++) {
      if (homing_stage < hw_actuator_homing_stages_[i]) {
        hw_command_positions_[i] = hw_state_positions_[i];
        hw_command_velocities_[i] = 0.0;
        hw_command_kps_[i] = 0.0;
        hw_command_kds_[i] = 0.0;
        all_homed = false;
        continue;
      } else {
        hw_command_kps_[i] = hw_actuator_homing_kps_[i];
        hw_command_kds_[i] = hw_actuator_homing_kds_[i];
      }
      // std::cout << "Commanded torque: " << filtered_torques[i] << std::endl;
      // std::cout << "Current position: " << hw_state_positions_[i] << std::endl;
      // std::cout << "Commanded position: " << hw_command_positions_[i] << std::endl;
      if (!hw_actuator_is_homed_[i]) {
        filtered_torques[i] =
            (1.0 - alpha) * filtered_torques[i] +
            alpha * ((hw_command_positions_[i] - hw_state_positions_[i]) * hw_command_kps_[i] +
                     (hw_command_velocities_[i] - hw_state_velocities_[i]) * hw_command_kds_[i]);
        if (std::abs(filtered_torques[i]) >= hw_actuator_homing_torque_thresholds_[i]) {
          hw_actuator_zero_positions_[i] = hw_state_positions_[i] - hw_actuator_homed_positions_[i];
          hw_command_positions_[i] = hw_actuator_homed_positions_[i];
          hw_command_velocities_[i] = 0.0;
          hw_actuator_is_homed_[i] = true;
          RCLCPP_INFO(rclcpp::get_logger("ControlBoardHardwareInterface"), "Homed actuator %d", i);
        } else {
          hw_command_positions_[i] += hw_actuator_homing_velocities_[i] * dt_ms * 0.001;
          hw_command_velocities_[i] = hw_actuator_homing_velocities_[i];
          all_homed = false;
        }
      }
    }
    copy_actuator_commands();
    spi_driver_run();
    copy_actuator_states();

    // Sleep for dt
    std::this_thread::sleep_for(std::chrono::milliseconds(dt_ms));
  }
  while (!all_returned) {
    all_returned = true;
    for (auto i = 0u; i < hw_state_positions_.size(); i++) {
      if (hw_command_positions_[i] < hw_actuator_post_homing_positions_[i] - 0.01) {
        hw_command_positions_[i] += std::abs(hw_actuator_homing_velocities_[i]) * dt_ms * 0.001;
        hw_command_velocities_[i] = std::abs(hw_actuator_homing_velocities_[i]);
        all_returned = false;
      } else if (hw_command_positions_[i] > hw_actuator_post_homing_positions_[i] + 0.01) {
        hw_command_positions_[i] -= std::abs(hw_actuator_homing_velocities_[i]) * dt_ms * 0.001;
        hw_command_velocities_[i] = -std::abs(hw_actuator_homing_velocities_[i]);
        all_returned = false;
      } else {
        hw_command_positions_[i] = hw_actuator_post_homing_positions_[i];
        hw_command_velocities_[i] = 0.0;
      }
    }
    copy_actuator_commands();
    spi_driver_run();
    copy_actuator_states();

    // Sleep for dt
    std::this_thread::sleep_for(std::chrono::milliseconds(dt_ms));
  }
  RCLCPP_INFO(rclcpp::get_logger("ControlBoardHardwareInterface"), "Finished homing!");
}

void ControlBoardHardwareInterface::copy_actuator_commands(bool use_position_limits) {
  // Iterate through the joints
  for (auto i = 0u; i < hw_state_positions_.size(); i++) {
    double cmd_pos = hw_command_positions_[i];
    double cmd_vel = std::clamp(hw_command_velocities_[i], -hw_actuator_velocity_maxs_[i],
                                hw_actuator_velocity_maxs_[i]);
    double cmd_eff = std::clamp(hw_command_efforts_[i], -hw_actuator_effort_maxs_[i],
                                hw_actuator_effort_maxs_[i]);
    double cmd_kp = std::clamp(hw_command_kps_[i], 0.0, hw_actuator_kp_maxs_[i]);
    double cmd_kd = std::clamp(hw_command_kds_[i], 0.0, hw_actuator_kd_maxs_[i]);

    if (use_position_limits && cmd_kp > 0.0) {
      if (cmd_pos < hw_actuator_position_mins_[i]) {
        cmd_pos = hw_actuator_position_mins_[i];
        cmd_vel = std::clamp(cmd_vel, 0.0, hw_actuator_velocity_maxs_[i]);
        cmd_eff = std::clamp(cmd_eff, 0.0, hw_actuator_effort_maxs_[i]);
      } else if (cmd_pos > hw_actuator_position_maxs_[i]) {
        cmd_pos = hw_actuator_position_maxs_[i];
        cmd_vel = std::clamp(cmd_vel, -hw_actuator_velocity_maxs_[i], 0.0);
        cmd_eff = std::clamp(cmd_eff, -hw_actuator_effort_maxs_[i], 0.0);
      }
      cmd_pos = std::clamp(cmd_pos, hw_actuator_position_mins_[i], hw_actuator_position_maxs_[i]);
    }

    cmd_pos += hw_actuator_zero_positions_[i];

    uint can_channel = hw_actuator_can_channels_[i] - 1;
    // ID 1: abad, ID 2: hip, ID 3: knee (not corresponding to the actual joint names, just used to
    // make the Cheetah code send to the CAN IDs we want)
    switch (hw_actuator_can_ids_[i]) {
      case 1:
        spi_command_->q_des_abad[can_channel] = cmd_pos;
        spi_command_->qd_des_abad[can_channel] = cmd_vel;
        spi_command_->kp_abad[can_channel] = cmd_kp;
        spi_command_->kd_abad[can_channel] = cmd_kd;
        spi_command_->tau_abad_ff[can_channel] = cmd_eff;
        break;
      case 2:
        spi_command_->q_des_hip[can_channel] = cmd_pos;
        spi_command_->qd_des_hip[can_channel] = cmd_vel;
        spi_command_->kp_hip[can_channel] = cmd_kp;
        spi_command_->kd_hip[can_channel] = cmd_kd;
        spi_command_->tau_hip_ff[can_channel] = cmd_eff;
        break;
      case 3:
        spi_command_->q_des_knee[can_channel] = cmd_pos;
        spi_command_->qd_des_knee[can_channel] = cmd_vel;
        spi_command_->kp_knee[can_channel] = cmd_kp;
        spi_command_->kd_knee[can_channel] = cmd_kd;
        spi_command_->tau_knee_ff[can_channel] = cmd_eff;
        break;
    }
  }
}

void ControlBoardHardwareInterface::copy_actuator_states() {
  // Iterate through the joints
  for (auto i = 0u; i < hw_state_positions_.size(); i++) {
    float state_pos = hw_state_positions_[i];
    float state_vel = hw_state_velocities_[i];

    uint can_channel = hw_actuator_can_channels_[i] - 1;
    // ID 1: abad, ID 2: hip, ID 3: knee (not corresponding to the actual joint names, just used to
    // make the Cheetah code send to the CAN IDs we want)
    switch (hw_actuator_can_ids_[i]) {
      case 1:
        state_pos = spi_data_->q_abad[can_channel];
        state_vel = spi_data_->qd_abad[can_channel];
        break;
      case 2:
        state_pos = spi_data_->q_hip[can_channel];
        state_vel = spi_data_->qd_hip[can_channel];
        break;
      case 3:
        state_pos = spi_data_->q_knee[can_channel];
        state_vel = spi_data_->qd_knee[can_channel];
        break;
    }
    hw_state_positions_[i] = state_pos - hw_actuator_zero_positions_[i];
    hw_state_velocities_[i] = state_vel;

    // Estimate actuator efforts based on motor driver PD control
    hw_state_efforts_[i] =
        (hw_command_positions_[i] - hw_state_positions_[i]) * hw_command_kps_[i] +
        (hw_command_velocities_[i] - hw_state_velocities_[i]) * hw_command_kds_[i];
  }
}

}  // namespace control_board_hardware_interface

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(control_board_hardware_interface::ControlBoardHardwareInterface,
                       hardware_interface::SystemInterface)
