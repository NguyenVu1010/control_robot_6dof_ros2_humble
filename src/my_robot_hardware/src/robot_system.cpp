#include "my_robot_hardware/robot_system.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include <pluginlib/class_list_macros.hpp>
#include "rclcpp/rclcpp.hpp"

namespace my_robot_hardware
{

hardware_interface::CallbackReturn RobotSystem::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  port_name_ = info_.hardware_parameters["device"];
  if (info_.hardware_parameters.find("baud_rate") != info_.hardware_parameters.end()) {
      baud_rate_value_ = std::stoi(info_.hardware_parameters["baud_rate"]);
  } else {
      baud_rate_value_ = 115200; 
  }
  hw_positions_.resize(info_.joints.size(), 0.0);
  hw_velocities_.resize(info_.joints.size(), 0.0);
  hw_commands_.resize(info_.joints.size(), 0.0);

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> RobotSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> RobotSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); i++) {
    if (info_.joints[i].name.find("gripper") != std::string::npos) {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
    } else {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]));
    }
  }
  return command_interfaces;
}

hardware_interface::CallbackReturn RobotSystem::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  try {
    serial_port_.Open(port_name_);
    LibSerial::BaudRate baud;
    switch(baud_rate_value_) {
        case 9600: baud = LibSerial::BaudRate::BAUD_9600; break;
        case 115200: baud = LibSerial::BaudRate::BAUD_115200; break;
        case 921600: baud = LibSerial::BaudRate::BAUD_921600; break;
        default: baud = LibSerial::BaudRate::BAUD_115200;
    }
    serial_port_.SetBaudRate(baud);
    serial_port_.SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8); // FIX LỖI 1
    serial_port_.SetParity(LibSerial::Parity::PARITY_NONE);               // FIX LỖI 2
    serial_port_.SetStopBits(LibSerial::StopBits::STOP_BITS_1);           // FIX LỖI 3
    serial_port_.FlushIOBuffers();
    RCLCPP_INFO(rclcpp::get_logger("RobotSystem"), "Successfully connected to %s", port_name_.c_str());
  } catch (...) {
    RCLCPP_FATAL(rclcpp::get_logger("RobotSystem"), "Failed to open port %s", port_name_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RobotSystem::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (serial_port_.IsOpen()) {
    serial_port_.Close();
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type RobotSystem::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!serial_port_.IsOpen()) {
      return hardware_interface::return_type::ERROR;
  }

  // --- LOGIC ĐỌC BINARY TỐI ƯU ---
  try {
    // Kiểm tra xem có đủ dữ liệu tối thiểu của 1 gói feedback không
    if (static_cast<size_t>(serial_port_.GetNumberOfBytesAvailable()) >= sizeof(RobotFeedback)) {
      
      uint8_t first_byte;
      serial_port_.ReadByte(first_byte, 10);
      
      // Tìm Header 0x55 (byte đầu của 0xAA55 trong Little Endian hoặc tùy protocol)
      // Giả sử MCU gửi Header 2 byte là 0xAA, 0x55
      if (first_byte == 0xAA) {
          uint8_t second_byte;
          serial_port_.ReadByte(second_byte, 10);
          
          if (second_byte == 0x55) {
              // Đã khớp Header, đọc phần còn lại của struct (trừ 2 byte header)
              RobotFeedback feedback;
              feedback.header = 0x55AA; // Gán lại cho đúng

              size_t remaining_size = sizeof(RobotFeedback) - 2;
              uint8_t* data_ptr = reinterpret_cast<uint8_t*>(&feedback) + 2;
              
              for (size_t i = 0; i < remaining_size; ++i) {
                  serial_port_.ReadByte(data_ptr[i], 10);
              }

              // Kiểm tra Checksum
              uint8_t* full_ptr = reinterpret_cast<uint8_t*>(&feedback);
              uint8_t calc_sum = calculate_checksum(full_ptr, sizeof(RobotFeedback) - 1);
              
              if (calc_sum == feedback.checksum) {
                  // Đổ dữ liệu vào state
                  for(int i=0; i<6; ++i) {
                      hw_positions_[i] = feedback.joint_positions[i];
                      hw_velocities_[i] = feedback.joint_velocities[i];
                  }
                  // Giả sử khớp cuối là gripper
                  if (hw_positions_.size() > 6) {
                      hw_positions_[6] = feedback.gripper_position;
                  }
              }
          }
      }
    }
  } catch (const std::exception &e) {
      // RCLCPP_DEBUG(rclcpp::get_logger("RobotSystem"), "Read timeout or error");
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RobotSystem::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!serial_port_.IsOpen()) return hardware_interface::return_type::ERROR;

  RobotCommand cmd;
  cmd.header = 0x55AA; // Header PC -> Robot
  
  for(int i=0; i<6; ++i) {
      cmd.joint_velocities[i] = static_cast<float>(hw_commands_[i]);
  }
  
  if (hw_commands_.size() > 6) {
      cmd.gripper_position = static_cast<float>(hw_commands_[6]);
  } else {
      cmd.gripper_position = 0.0f;
  }
  
  cmd.checksum = calculate_checksum(reinterpret_cast<uint8_t*>(&cmd), sizeof(RobotCommand) - 1);

  // Gửi Binary qua Serial
  uint8_t* ptr = reinterpret_cast<uint8_t*>(&cmd);
  for (size_t i = 0; i < sizeof(RobotCommand); ++i) {
      serial_port_.WriteByte(ptr[i]);
  }
  serial_port_.DrainWriteBuffer();

  return hardware_interface::return_type::OK;
}

} // namespace my_robot_hardware

PLUGINLIB_EXPORT_CLASS(my_robot_hardware::RobotSystem, hardware_interface::SystemInterface)