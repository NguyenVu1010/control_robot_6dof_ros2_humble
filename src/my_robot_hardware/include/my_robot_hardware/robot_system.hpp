#ifndef MY_ROBOT_HARDWARE__ROBOT_SYSTEM_HPP_
#define MY_ROBOT_HARDWARE__ROBOT_SYSTEM_HPP_

#include <vector>
#include <string>
#include <libserial/SerialPort.h>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "my_robot_hardware/protocol.hpp"

namespace my_robot_hardware
{
class RobotSystem : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(RobotSystem)

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  LibSerial::SerialPort serial_port_;
  std::string port_name_;
  int baud_rate_value_;
  // Dữ liệu nội bộ cho ROS 2 Control
  std::vector<double> hw_commands_;
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
};
}
#endif