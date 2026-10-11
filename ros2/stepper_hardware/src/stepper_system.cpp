#include "stepper_hardware/stepper_system.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <limits>
#include <stdexcept>
#include <thread>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace stepper_hardware
{

namespace
{

rclcpp::Logger logger() { return rclcpp::get_logger("StepperSystem"); }

std::uint64_t now_ns()
{
  return static_cast<std::uint64_t>(
    std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::steady_clock::now().time_since_epoch()).count());
}

template<typename Map>
std::string param_str(const Map & m, const std::string & key, const std::string & def)
{
  const auto it = m.find(key);
  return (it == m.end() || it->second.empty()) ? def : it->second;
}

template<typename Map>
double param_num(const Map & m, const std::string & key, double def, bool & ok)
{
  const auto it = m.find(key);
  if (it == m.end() || it->second.empty()) {
    return def;
  }
  try {
    return std::stod(it->second);
  } catch (const std::exception &) {
    RCLCPP_ERROR(logger(), "พารามิเตอร์ '%s' = '%s' ไม่ใช่ตัวเลข", key.c_str(), it->second.c_str());
    ok = false;
    return def;
  }
}

template<typename Map>
bool param_bool(const Map & m, const std::string & key, bool def)
{
  const std::string v = param_str(m, key, def ? "true" : "false");
  return v == "true" || v == "True" || v == "1";
}

}  // namespace

// ==========================================================================
// on_init -- อ่านค่าจากแท็ก <ros2_control> ใน URDF แล้วเตรียมที่เก็บข้อมูล
// ยังไม่แตะฮาร์ดแวร์ในขั้นนี้
// ==========================================================================
hardware_interface::CallbackReturn StepperSystem::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  bool ok = true;
  const auto & hw = info_.hardware_parameters;
  port_ = param_str(hw, "port", "/dev/ttyUSB0");
  baud_ = static_cast<int>(param_num(hw, "baud", 115200.0, ok));
  status_hz_ = static_cast<int>(param_num(hw, "status_hz", 50.0, ok));
  stale_timeout_ = param_num(hw, "stale_timeout", 0.5, ok);
  stream_ = param_bool(hw, "stream", true);

  if (info_.joints.size() > kAxisCount) {
    RCLCPP_FATAL(
      logger(), "บอร์ดรองรับ %zu แกน แต่ URDF ประกาศมา %zu joint",
      kAxisCount, info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  joints_.clear();
  joints_.reserve(info_.joints.size());

  for (const auto & j : info_.joints) {
    if (j.command_interfaces.size() != 1 ||
      j.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        logger(), "joint '%s' ต้องมี command interface เป็น position อย่างเดียว",
        j.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    bool has_position_state = false;
    bool has_velocity_state = false;
    for (const auto & si : j.state_interfaces) {
      if (si.name == hardware_interface::HW_IF_POSITION) has_position_state = true;
      if (si.name == hardware_interface::HW_IF_VELOCITY) has_velocity_state = true;
    }
    if (!has_position_state) {
      RCLCPP_FATAL(logger(), "joint '%s' ต้องมี state interface 'position'", j.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    JointCfg cfg;
    cfg.name = j.name;
    cfg.axis = static_cast<std::size_t>(param_num(j.parameters, "axis", 0.0, ok));
    if (cfg.axis >= kAxisCount) {
      RCLCPP_FATAL(
        logger(), "joint '%s' ตั้ง axis=%zu แต่บอร์ดมีแค่แกน 0-%zu",
        j.name.c_str(), cfg.axis, kAxisCount - 1);
      return hardware_interface::CallbackReturn::ERROR;
    }

    // step ต่อ 1 รอบด้านออก = steps_per_rev x microsteps x gear_ratio
    //   units_per_rev : แกนหมุน = 2*pi (rad ต่อรอบ), แกนเลื่อน = ระยะ lead หน่วยเมตร
    const double steps_per_rev = param_num(j.parameters, "steps_per_rev", 200.0, ok);
    const double microsteps = param_num(j.parameters, "microsteps", 8.0, ok);
    const double gear_ratio = param_num(j.parameters, "gear_ratio", 1.0, ok);
    const double units_per_rev = param_num(j.parameters, "units_per_rev", 2.0 * M_PI, ok);
    if (units_per_rev == 0.0) {
      RCLCPP_FATAL(logger(), "joint '%s' ตั้ง units_per_rev เป็น 0 ไม่ได้", j.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    cfg.steps_per_unit = steps_per_rev * microsteps * gear_ratio / units_per_rev;
    if (param_bool(j.parameters, "invert", false)) {
      cfg.steps_per_unit = -cfg.steps_per_unit;
    }
    cfg.offset = param_num(j.parameters, "offset", 0.0, ok);
    cfg.max_vel = param_num(j.parameters, "max_vel", 1.0, ok);
    cfg.max_accel = param_num(j.parameters, "max_accel", 2.0, ok);
    cfg.limit_lo = param_num(j.parameters, "limit_lo", 0.0, ok);
    cfg.limit_hi = param_num(j.parameters, "limit_hi", 0.0, ok);
    cfg.export_velocity = has_velocity_state;

    joints_.push_back(cfg);
  }

  if (!ok) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  const std::size_t n = joints_.size();
  pos_.assign(n, 0.0);
  vel_.assign(n, 0.0);
  cmd_.assign(n, std::numeric_limits<double>::quiet_NaN());

  RCLCPP_INFO(
    logger(), "%zu joint, พอร์ต %s @ %d, โหมด %s",
    n, port_.c_str(), baud_, stream_ ? "streaming" : "point-to-point");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ==========================================================================
// export_* -- ส่งคืนพอยเตอร์ให้ framework เขียน/อ่าน double ตรง ๆ ทุกรอบลูป
// ==========================================================================
std::vector<hardware_interface::StateInterface> StepperSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> out;
  for (std::size_t i = 0; i < joints_.size(); ++i) {
    out.emplace_back(joints_[i].name, hardware_interface::HW_IF_POSITION, &pos_[i]);
    if (joints_[i].export_velocity) {
      out.emplace_back(joints_[i].name, hardware_interface::HW_IF_VELOCITY, &vel_[i]);
    }
  }
  return out;
}

std::vector<hardware_interface::CommandInterface> StepperSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> out;
  for (std::size_t i = 0; i < joints_.size(); ++i) {
    out.emplace_back(joints_[i].name, hardware_interface::HW_IF_POSITION, &cmd_[i]);
  }
  return out;
}

// ==========================================================================
// วงจรชีวิต
// ==========================================================================
hardware_interface::CallbackReturn StepperSystem::on_configure(const rclcpp_lifecycle::State &)
{
  std::string err;
  if (!link_.open(port_, baud_, err)) {
    RCLCPP_FATAL(logger(), "%s", err.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // ESP32 รีเซ็ตตัวเองตอนเปิดพอร์ต (DTR) ต้องรอบูตให้จบก่อนค่อยยิงคำสั่ง
  std::this_thread::sleep_for(std::chrono::milliseconds(2200));
  push_board_config();
  drain_board_log();

  RCLCPP_INFO(logger(), "ต่อกับบอร์ดที่ %s แล้ว", port_.c_str());
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn StepperSystem::on_cleanup(const rclcpp_lifecycle::State &)
{
  if (link_.is_open()) {
    link_.send("S\n");
    link_.send("E 0\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  link_.close();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn StepperSystem::on_activate(const rclcpp_lifecycle::State &)
{
  if (!link_.is_open()) {
    RCLCPP_FATAL(logger(), "ยังไม่ได้ต่อกับบอร์ด");
    return hardware_interface::CallbackReturn::ERROR;
  }

  link_.send("C\n");
  push_board_config();
  link_.send("E 1\n");

  // รอ status ก้อนแรกก่อน จะได้รู้ตำแหน่งจริงของทุกแกน
  Snapshot snap;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
  do {
    snap = link_.latest();
    if (snap.valid) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  } while (std::chrono::steady_clock::now() < deadline);

  if (!snap.valid) {
    RCLCPP_FATAL(logger(), "ไม่ได้รับ status จากบอร์ดภายใน 3 วินาที");
    return hardware_interface::CallbackReturn::ERROR;
  }
  last_good_ = snap;

  // สำคัญ: ตั้งค่าสั่งให้เท่ากับตำแหน่งจริง ไม่งั้นพอ controller เริ่มทำงาน
  // มันจะเห็นค่าสั่งเก่า (หรือ 0) แล้วสั่งวิ่งไปที่นั่นทันทีตั้งแต่รอบแรก
  bool all_homed = true;
  for (std::size_t i = 0; i < joints_.size(); ++i) {
    const auto & j = joints_[i];
    pos_[i] = to_units(j, static_cast<double>(snap.pos[j.axis]));
    vel_[i] = 0.0;
    cmd_[i] = pos_[i];
    if (!(snap.flags[j.axis] & kAfHomed)) {
      all_homed = false;
    }
  }
  if (!all_homed) {
    RCLCPP_WARN(
      logger(),
      "ยังมีแกนที่ไม่ได้ home -- ตำแหน่งที่อ่านได้เป็นค่าสัมพัทธ์จากตอนเปิดบอร์ด "
      "ไม่ใช่พิกัดจริง");
  }

  stale_cycles_ = 0;
  RCLCPP_INFO(logger(), "พร้อมทำงาน จ่ายไฟมอเตอร์แล้ว");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn StepperSystem::on_deactivate(const rclcpp_lifecycle::State &)
{
  if (link_.is_open()) {
    link_.send("S\n");
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    link_.send("E 0\n");
  }
  RCLCPP_INFO(logger(), "หยุดและตัดไฟมอเตอร์แล้ว");
  return hardware_interface::CallbackReturn::SUCCESS;
}

// ==========================================================================
// read / write -- ถูกเรียกทุกรอบของลูป real-time ห้ามบล็อก ห้ามจองหน่วยความจำก้อนใหญ่
// ==========================================================================
hardware_interface::return_type StepperSystem::read(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  const Snapshot snap = link_.latest();
  if (snap.valid) {
    last_good_ = snap;
    stale_cycles_ = 0;
  } else if (!last_good_.valid) {
    if (++stale_cycles_ > 200) {
      RCLCPP_ERROR(logger(), "ไม่เคยได้รับ status จากบอร์ดเลย");
      return hardware_interface::return_type::ERROR;
    }
    return hardware_interface::return_type::OK;
  }

  const double age = static_cast<double>(now_ns() - last_good_.host_ns) * 1e-9;
  if (age > stale_timeout_) {
    RCLCPP_ERROR(logger(), "ขาดการติดต่อกับบอร์ดมา %.2f วินาที", age);
    return hardware_interface::return_type::ERROR;
  }

  for (std::size_t i = 0; i < joints_.size(); ++i) {
    const auto & j = joints_[i];
    pos_[i] = to_units(j, static_cast<double>(last_good_.pos[j.axis]));
    vel_[i] = static_cast<double>(last_good_.vel[j.axis]) / j.steps_per_unit;
  }

  if (last_good_.sys & kSysFault) {
    RCLCPP_ERROR(logger(), "บอร์ดอยู่ในสถานะ E-stop");
    return hardware_interface::return_type::ERROR;
  }
  for (const auto & j : joints_) {
    if (last_good_.flags[j.axis] & kAfFault) {
      RCLCPP_ERROR(logger(), "joint '%s' เข้าสถานะ fault (ส่วนใหญ่คือชนลิมิตสวิตช์)",
        j.name.c_str());
      return hardware_interface::return_type::ERROR;
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type StepperSystem::write(
  const rclcpp::Time &, const rclcpp::Duration & period)
{
  if (!link_.is_open()) {
    return hardware_interface::return_type::ERROR;
  }

  // รอบแรก ๆ controller ยังไม่ได้เขียนค่าสั่ง -- อย่าเพิ่งส่งอะไรออกไป
  for (const double c : cmd_) {
    if (!std::isfinite(c)) {
      return hardware_interface::return_type::OK;
    }
  }

  char targets[kAxisCount][16];
  for (std::size_t a = 0; a < kAxisCount; ++a) {
    targets[a][0] = '*';
    targets[a][1] = '\0';
  }

  for (std::size_t i = 0; i < joints_.size(); ++i) {
    const auto & j = joints_[i];
    double c = cmd_[i];
    if (j.limit_lo != j.limit_hi) {
      const double lo = std::min(j.limit_lo, j.limit_hi);
      const double hi = std::max(j.limit_lo, j.limit_hi);
      c = std::min(std::max(c, lo), hi);
    }
    std::snprintf(targets[j.axis], sizeof(targets[j.axis]), "%d", to_steps(j, c));
  }

  char line[128];
  if (stream_) {
    long dt_us = period.nanoseconds() / 1000;
    if (dt_us < 500) dt_us = 500;
    if (dt_us > 500000) dt_us = 500000;
    std::snprintf(
      line, sizeof(line), "T %ld %s %s %s %s\n",
      dt_us, targets[0], targets[1], targets[2], targets[3]);
  } else {
    std::snprintf(
      line, sizeof(line), "MA %s %s %s %s\n",
      targets[0], targets[1], targets[2], targets[3]);
  }
  link_.send(line);

  // เตือนเมื่อเขียนพอร์ตไม่ทัน แต่จำกัดไว้ 5 วินาทีต่อครั้ง
  // (ใช้นาฬิกาของตัวเอง ไม่ใช่ RCLCPP_*_THROTTLE เพราะในนี้ไม่มี node clock ให้ใช้)
  const std::uint64_t dropped = link_.dropped_writes();
  if (dropped != last_dropped_) {
    const std::uint64_t t = now_ns();
    if (t - last_drop_log_ns_ > 5000000000ULL) {
      last_drop_log_ns_ = t;
      RCLCPP_WARN(
        logger(),
        "ส่งคำสั่งไม่ทัน ทิ้งไปแล้ว %llu บรรทัด -- baud ต่ำไปสำหรับ update_rate ที่ตั้งไว้",
        static_cast<unsigned long long>(dropped));
    }
    last_dropped_ = dropped;
  }

  return hardware_interface::return_type::OK;
}

// ==========================================================================
// ตัวช่วย
// ==========================================================================
std::int32_t StepperSystem::to_steps(const JointCfg & j, double value) const
{
  return static_cast<std::int32_t>(std::lround((value - j.offset) * j.steps_per_unit));
}

double StepperSystem::to_units(const JointCfg & j, double steps) const
{
  return steps / j.steps_per_unit + j.offset;
}

void StepperSystem::push_board_config()
{
  char line[96];
  std::snprintf(line, sizeof(line), "F %d\n", status_hz_);
  link_.send(line);

  for (const auto & j : joints_) {
    const long vmax = std::max(1L, std::lround(std::fabs(j.max_vel * j.steps_per_unit)));
    const long accel = std::max(1L, std::lround(std::fabs(j.max_accel * j.steps_per_unit)));
    std::snprintf(line, sizeof(line), "V %zu %ld %ld\n", j.axis, vmax, accel);
    link_.send(line);

    if (j.limit_lo != j.limit_hi) {
      const std::int32_t s1 = to_steps(j, j.limit_lo);
      const std::int32_t s2 = to_steps(j, j.limit_hi);
      std::snprintf(
        line, sizeof(line), "L %zu %d %d\n", j.axis, std::min(s1, s2), std::max(s1, s2));
      link_.send(line);
    }
  }
}

void StepperSystem::drain_board_log()
{
  const std::string logs = link_.take_log();
  if (!logs.empty()) {
    RCLCPP_INFO(logger(), "บอร์ดแจ้ง: %s", logs.c_str());
  }
}

}  // namespace stepper_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(stepper_hardware::StepperSystem, hardware_interface::SystemInterface)
