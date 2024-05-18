#ifndef CONTROL_BOARD_HARDWARE_INTERFACE__CONTROL_BOARD_HARDWARE_INTERFACE_HPP_
#define CONTROL_BOARD_HARDWARE_INTERFACE__CONTROL_BOARD_HARDWARE_INTERFACE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "rt/rt_bno055.h"
#include "rt/rt_spi.h"

#define IMU_I2C_DEVICE_NUMBER 1

namespace control_board_hardware_interface {
class ControlBoardHardwareInterface : public hardware_interface::SystemInterface {
 public:
  RCLCPP_SHARED_PTR_DEFINITIONS(ControlBoardHardwareInterface)

  virtual ~ControlBoardHardwareInterface();

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;

  hardware_interface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State &previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State &previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State &previous_state) override;

  hardware_interface::return_type read(const rclcpp::Time &time,
                                       const rclcpp::Duration &period) override;

  hardware_interface::return_type write(const rclcpp::Time &time,
                                        const rclcpp::Duration &period) override;

 private:
  void copy_actuator_commands(bool use_position_limits = false);
  void copy_actuator_states();
  void do_homing();

  BNO055 *imu_;
  BNO055::Output imu_output_;
  spi_command_t *spi_command_;
  spi_data_t *spi_data_;

  // IMU state
  std::array<double, 4> hw_state_imu_orientation_;          // x, y, z, w
  std::array<double, 3> hw_state_imu_angular_velocity_;     // x, y, z
  std::array<double, 3> hw_state_imu_linear_acceleration_;  // x, y, z
  double imu_roll_, imu_pitch_, imu_yaw_;

  // Actuator CAN config
  std::vector<int> hw_actuator_can_channels_;
  std::vector<int> hw_actuator_can_ids_;

  // Actuator homing
  std::vector<int> hw_actuator_homing_stages_;
  std::vector<double> hw_actuator_homing_velocities_;
  std::vector<double> hw_actuator_homing_kps_;
  std::vector<double> hw_actuator_homing_kds_;
  std::vector<double> hw_actuator_homed_positions_;
  std::vector<double> hw_actuator_zero_positions_;
  std::vector<double> hw_actuator_homing_torque_thresholds_;
  std::vector<bool> hw_actuator_is_homed_;

  // Actuator limits
  std::vector<double> hw_actuator_position_mins_;
  std::vector<double> hw_actuator_position_maxs_;
  std::vector<double> hw_actuator_velocity_maxs_;
  std::vector<double> hw_actuator_effort_maxs_;
  std::vector<double> hw_actuator_kp_maxs_;
  std::vector<double> hw_actuator_kd_maxs_;

  // Actuator states
  std::vector<double> hw_state_positions_;
  std::vector<double> hw_state_velocities_;
  std::vector<double> hw_state_efforts_;

  // Actuator commands
  std::vector<double> hw_command_positions_;
  std::vector<double> hw_command_velocities_;
  std::vector<double> hw_command_efforts_;
  std::vector<double> hw_command_kps_;
  std::vector<double> hw_command_kds_;
};

}  // namespace control_board_hardware_interface

#endif  // CONTROL_BOARD_HARDWARE_INTERFACE_PUBLIC__CONTROL_BOARD_HARDWARE_INTERFACE_HPP_