// ตัวจัดการพอร์ต serial สำหรับ StepperSystem
//
// ข้อกำหนดสำคัญ: read() / write() ของ ros2_control ถูกเรียกในลูป real-time
// จะบล็อกรอข้อมูลไม่ได้เด็ดขาด คลาสนี้จึงแยก thread ไว้อ่านพอร์ตต่างหาก
// แล้วเก็บค่าล่าสุดไว้ใน seqlock ให้ read() มาหยิบไปโดยไม่ต้องรอ lock

#ifndef STEPPER_HARDWARE__SERIAL_LINK_HPP_
#define STEPPER_HARDWARE__SERIAL_LINK_HPP_

#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>

namespace stepper_hardware
{

inline constexpr std::size_t kAxisCount = 4;

// บิตสถานะต่อแกน ต้องตรงกับ AF_* ในไฟล์ esp32_dm556_stepper.ino
inline constexpr std::uint8_t kAfMoving = 0x01;
inline constexpr std::uint8_t kAfHomed = 0x02;
inline constexpr std::uint8_t kAfLimit = 0x04;
inline constexpr std::uint8_t kAfFault = 0x08;
inline constexpr std::uint8_t kAfHoming = 0x10;
inline constexpr std::uint8_t kAfStream = 0x20;

// บิตสถานะรวมของบอร์ด
inline constexpr std::uint8_t kSysEnabled = 0x01;
inline constexpr std::uint8_t kSysFault = 0x02;

struct Snapshot
{
  std::int32_t pos[kAxisCount]{};
  std::int32_t vel[kAxisCount]{};
  std::uint8_t flags[kAxisCount]{};
  std::uint8_t sys{0};
  std::uint64_t board_ms{0};
  std::uint64_t host_ns{0};
  bool valid{false};
};

class SerialLink
{
public:
  SerialLink() = default;
  ~SerialLink();

  SerialLink(const SerialLink &) = delete;
  SerialLink & operator=(const SerialLink &) = delete;

  bool open(const std::string & port, int baud, std::string & error);
  void close();
  bool is_open() const { return fd_ >= 0; }

  // เขียนแบบ non-blocking ถ้าบัฟเฟอร์เต็มจะทิ้งบรรทัดนั้นแล้วนับไว้
  // (ยอมทิ้งดีกว่าให้ลูป real-time ค้าง เดี๋ยวคาบถัดไปก็ส่งใหม่อยู่แล้ว)
  bool send(const std::string & line);

  // ปลอดภัยสำหรับเรียกในลูป real-time: ไม่ล็อก ไม่จองหน่วยความจำ
  // ถ้า thread อ่านกำลังเขียนค่าอยู่พอดีจะคืน valid=false ผู้เรียกใช้ค่าเดิมต่อได้
  Snapshot latest() const;

  std::uint64_t dropped_writes() const { return dropped_.load(std::memory_order_relaxed); }

  // ดึงข้อความ '#' / 'ERR' ที่บอร์ดส่งมา (เรียกนอกลูป real-time เท่านั้น)
  std::string take_log();

private:
  void reader();
  void handle_line(const std::string & line);
  void publish(const Snapshot & snap);

  int fd_{-1};
  std::thread thread_;
  std::atomic<bool> running_{false};

  // seqlock: เลขคี่ = กำลังเขียนอยู่
  std::atomic<unsigned> seq_{0};
  Snapshot data_{};

  std::atomic<std::uint64_t> dropped_{0};
  std::string rx_;

  mutable std::mutex log_mutex_;
  std::string log_;
};

}  // namespace stepper_hardware

#endif  // STEPPER_HARDWARE__SERIAL_LINK_HPP_
