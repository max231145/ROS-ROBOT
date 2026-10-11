// hardware_interface สำหรับบอร์ด esp32_dm556_stepper
//
// หน้าที่มีอย่างเดียวคือแปลงระหว่างหน่วยความจำที่ ros2_control ใช้ กับพอร์ต serial
// ไม่มีตรรกะควบคุมอยู่ในนี้เลย -- ตัวคิด trajectory คือ controller ชั้นบน

#ifndef STEPPER_HARDWARE__STEPPER_SYSTEM_HPP_
#define STEPPER_HARDWARE__STEPPER_SYSTEM_HPP_

#include <cstddef>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "stepper_hardware/serial_link.hpp"

namespace stepper_hardware
{

class StepperSystem : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(StepperSystem)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  struct JointCfg
  {
    std::string name;
    std::size_t axis{0};
    double steps_per_unit{1.0};   // ติดลบได้ ถ้าตั้ง invert
    double offset{0.0};           // ค่า joint ที่ตรงกับ step 0 (คือจุด home)
    double max_vel{1.0};
    double max_accel{2.0};
    double limit_lo{0.0};
    double limit_hi{0.0};         // lo == hi = ไม่ใช้ soft limit
    bool export_velocity{false};  // URDF ประกาศ state interface 'velocity' ไว้ไหม
  };

  std::int32_t to_steps(const JointCfg & j, double value) const;
  double to_units(const JointCfg & j, double steps) const;
  void push_board_config();
  void drain_board_log();

  std::vector<JointCfg> joints_;
  std::vector<double> pos_;
  std::vector<double> vel_;
  std::vector<double> cmd_;

  SerialLink link_;
  Snapshot last_good_{};

  std::string port_{"/dev/ttyUSB0"};
  int baud_{115200};
  bool stream_{true};
  int status_hz_{50};
  double stale_timeout_{0.5};

  std::uint64_t last_dropped_{0};
  std::uint64_t last_drop_log_ns_{0};
  int stale_cycles_{0};
};

}  // namespace stepper_hardware

#endif  // STEPPER_HARDWARE__STEPPER_SYSTEM_HPP_
