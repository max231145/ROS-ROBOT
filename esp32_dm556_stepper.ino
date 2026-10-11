/* ==========================================================================
 * ESP32 + DM556 -- 4-Axis Stepper Controller (position / angle control)
 * --------------------------------------------------------------------------
 * ใช้เป็น motion controller ให้ ROS 2 โดยคุยผ่าน USB serial
 * ฝั่ง Pi 5 รัน node ชื่อ stepper_bridge (อยู่ในโฟลเดอร์ ros2/) แปลง
 * radian <-> step ให้เอง เฟิร์มแวร์ตัวนี้คิดเป็น "step" ล้วน ๆ อย่างเดียว
 *
 * บอร์ด   : ESP32 Dev Module,  Serial 115200
 * ไลบรารี : ไม่ต้องลงอะไรเพิ่ม (ปั่นพัลส์เองด้วย hardware timer)
 *           รองรับทั้ง arduino-esp32 core 2.x และ 3.x
 *
 * หลักการ:
 *   - Timer ISR 20 kHz : ปั่นพัลส์อย่างเดียว ใช้ phase accumulator เลขจำนวนเต็ม
 *                        (ห้ามใช้ float ใน ISR ของ ESP32 -- FPU context ไม่ถูก save)
 *   - Planner 1 kHz    : คิด trapezoidal ramp ด้วย float ใน loop() แล้วโยนค่า
 *                        inc (ความถี่พัลส์) ให้ ISR
 *   ความเร็วสูงสุด = ISR_HZ / 2 = 10,000 step/s ต่อแกน
 *   ถ้าต้องการเร็วกว่านี้ ให้ลด microstep ที่ DIP ของ DM556 หรือดัน ISR_HZ เป็น 40000
 * ========================================================================== */

#include <Arduino.h>

// ==========================================================================
// 1. ขา GPIO   (ต่อแบบ common-anode -- ดูวิธีต่อสายใน README.md)
// ==========================================================================
#define AXIS_COUNT 4

static const uint8_t PIN_STEP[AXIS_COUNT]  = { 25, 27, 32, 18 };
static const uint8_t PIN_DIR [AXIS_COUNT]  = { 26, 14, 33, 19 };
static const int8_t  PIN_LIMIT[AXIS_COUNT] = { 16, 17,  4, 13 };   // -1 = ไม่มีลิมิตสวิตช์

#define PIN_ENABLE          23   // ENA ร่วมทั้ง 4 ตัว (ต่อขนานกัน)

// --- ขั้วสัญญาณ: ค่าเริ่มต้นนี้ตั้งไว้สำหรับการต่อแบบ common-anode ---
//     (PUL+/DIR+/ENA+ ของ DM556 ทุกตัวต่อรวมเข้า 3V3 ของ ESP32
//      แล้ว GPIO ไปเข้า PUL-/DIR-/ENA-  ดังนั้น GPIO LOW = ออปโตติด)
#define STEP_ACTIVE_LOW      1   // 1 = พัลส์เป็นช่วง LOW (ขาพักไว้ที่ HIGH)
#define ENABLE_ACTIVE_LOW    0   // DM556 ออปโต ENA ติด = "ปิด" ไดรเวอร์
                                 // ต่อ common-anode จึงต้องดึง HIGH เพื่อให้มอเตอร์มีแรงบิด
                                 // ถ้าของจริงกลับด้าน (สั่ง E 1 แล้วมอเตอร์ฟรี) ให้เปลี่ยนเป็น 1
#define LIMIT_ACTIVE_LOW     1   // สวิตช์ต่อลง GND ใช้ pull-up ในตัว ESP32

// ทิศ: ถ้าแกนไหนหมุนกลับด้าน แก้ตรงนี้บรรทัดเดียว ไม่ต้องสลับสาย
static const bool DIR_INVERT[AXIS_COUNT] = { false, false, false, false };

// ==========================================================================
// 2. ค่าที่ปรับได้ (รวมไว้ที่เดียว)
// ==========================================================================
#define ISR_HZ              20000UL   // ความถี่ timer ที่ปั่นพัลส์
#define PLANNER_PERIOD_US    1000UL   // คาบการคิด ramp (1 kHz)
#define SERIAL_BAUD         115200UL  // ต้องตรงกับ param 'baud' ฝั่ง ROS

#define DEF_VMAX            4000.0f   // step/s   ค่าเริ่มต้นต่อแกน (สั่งทับด้วยคำสั่ง V ได้)
#define DEF_ACCEL           8000.0f   // step/s^2
#define CREEP_RATE            60.0f   // step/s  ความเร็วคืบเข้าเป้า กันค้างห่างเป้า 1-2 step
#define FINAL_APPROACH_STEPS   2.0f   // ระยะสุดท้ายที่ห้ามเร่ง ให้คืบอย่างเดียว
#define BRAKE_MARGIN           1.5f   // ตัวเผื่อระยะเบรก (ดูคำอธิบายใน planAxis)

#define HOME_SEEK_RATE       800.0f   // step/s  วิ่งหาลิมิตรอบแรก
#define HOME_SLOW_RATE       100.0f   // step/s  แตะลิมิตรอบสอง (กำหนดความแม่นของ home)
#define HOME_BACKOFF_STEPS     400L   // ถอยออกจากลิมิตเท่าไหร่ก่อนแตะซ้ำ
#define HOME_TIMEOUT_MS      30000UL

#define STATUS_HZ_DEFAULT       20    // ความถี่ส่ง status (0 = ปิด, เปลี่ยนด้วยคำสั่ง F)
#define HOST_TIMEOUT_MS       2000UL  // ไม่ได้ยินเสียงจาก host เกินนี้ = เบรกทุกแกน (0 = ปิด)

// --- โหมด streaming (ใช้กับ ros2_control) ---
// คำสั่ง T ทำให้แกนเลิกใช้พลานเนอร์ในบอร์ด แล้ววิ่งตาม setpoint ที่ host ป้อนมาทุกคาบแทน
// ถ้าไม่มีโหมดนี้ พลานเนอร์ในบอร์ดจะ ramp ซ้อนกับ trajectory ที่ controller คิดมาแล้ว
// กลายเป็น ramp ซ้อน ramp -> ตามหลังตลอด แขนอืดและเลยเป้า
#define STREAM_DT_MIN_US       500UL  // คาบควบคุมที่ยอมรับได้ (2 kHz)
#define STREAM_DT_MAX_US    500000UL  // (2 Hz)
#define STREAM_TIMEOUT_MULT      3    // ขาด setpoint กี่คาบถึงจะชะลอลงศูนย์
#define STREAM_TIMEOUT_MIN_MS   50UL

#define RX_BUF_LEN              96

// ==========================================================================
// 3. โครงสร้างข้อมูลต่อแกน
// ==========================================================================
enum HomeState : uint8_t {
  HOME_IDLE = 0, HOME_SEEK, HOME_BACKOFF, HOME_SLOW, HOME_SETTLE
};

// bit ของ flag ต่อแกน (ส่งขึ้นไปกับ status)
#define AF_MOVING  0x01
#define AF_HOMED   0x02
#define AF_LIMIT   0x04
#define AF_FAULT   0x08
#define AF_HOMING  0x10
#define AF_STREAM  0x20

struct Axis {
  // --- ใช้ร่วมกับ ISR ---
  volatile int32_t  pos;         // ตำแหน่งปัจจุบัน (step)
  volatile uint32_t inc;         // ค่าที่บวกเข้า accumulator ทุก tick (0 = หยุด)
  volatile uint32_t acc;
  volatile int8_t   dirSign;     // +1 / -1
  volatile bool     limitHit;    // ISR ตั้งให้เมื่อสวิตช์ทำงาน
  volatile bool     ignoreLimit; // true ตอน home (ต้องวิ่งชนลิมิตเป็นเรื่องปกติ)

  // --- ใช้เฉพาะใน planner ---
  int32_t target;
  float   v;                     // ความเร็วปัจจุบัน (step/s, มีเครื่องหมาย)
  float   vmax;
  float   accel;
  int32_t softMin, softMax;      // ลิมิตซอฟต์แวร์ (step) -- min==max = ปิด
  bool    softLimitOn;
  bool    homed;
  bool    fault;

  uint8_t  homeState;
  int8_t   homeDir;              // ทิศที่วิ่งไปหาลิมิต
  int32_t  homeZero;             // ค่าที่จะเซ็ตให้ pos เมื่อ home เสร็จ
  uint32_t homeStartMs;

  // --- โหมด streaming ---
  bool    streamMode;
  int32_t streamTarget;          // setpoint ล่าสุดที่ host ป้อนมา
  float   streamFF;              // ความเร็ว feed-forward = ส่วนต่าง setpoint / คาบควบคุม
};

static Axis ax[AXIS_COUNT];

// มาสก์ GPIO สำหรับเขียนพัลส์ทีเดียวทั้ง 4 แกน (ESP32 แบ่ง register เป็น 2 ธนาคาร)
static uint32_t stepMaskAll0 = 0, stepMaskAll1 = 0;
static uint32_t stepBit0[AXIS_COUNT], stepBit1[AXIS_COUNT];

static hw_timer_t *stepTimer = nullptr;
static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

static bool     drvEnabled   = false;
static bool     sysFault     = false;
static uint16_t statusHz     = STATUS_HZ_DEFAULT;
static uint32_t lastHostMs   = 0;
static float    ctrlDt       = 0.02f;   // คาบควบคุมของ host ในโหมด streaming (วินาที)
static uint32_t lastStreamMs = 0;

// อัตราส่วนแปลง step/s -> ค่า inc ของ accumulator 32 บิต
static const float INC_PER_HZ = 4294967296.0f / (float)ISR_HZ;
static const float RATE_MAX   = (float)ISR_HZ * 0.5f;

// ==========================================================================
// 4. อ่าน GPIO แบบตรง register (เร็วพอที่จะเรียกใน ISR)
// ==========================================================================
static inline bool IRAM_ATTR fastRead(int8_t pin) {
  if (pin < 32) return (GPIO.in >> pin) & 0x1;
  else          return (GPIO.in1.val >> (pin - 32)) & 0x1;
}

// ยก/ปล่อยขา STEP ตามขั้วที่ตั้งไว้ (เขียนทีเดียวทั้ง 4 แกน ไม่ต้องวนทีละขา)
#if STEP_ACTIVE_LOW
static inline void IRAM_ATTR stepAssert(uint32_t m0, uint32_t m1) {
  if (m0) GPIO.out_w1tc = m0;
  if (m1) GPIO.out1_w1tc.val = m1;
}
static inline void IRAM_ATTR stepRelease(uint32_t m0, uint32_t m1) {
  if (m0) GPIO.out_w1ts = m0;
  if (m1) GPIO.out1_w1ts.val = m1;
}
#define STEP_IDLE_LEVEL HIGH
#else
static inline void IRAM_ATTR stepAssert(uint32_t m0, uint32_t m1) {
  if (m0) GPIO.out_w1ts = m0;
  if (m1) GPIO.out1_w1ts.val = m1;
}
static inline void IRAM_ATTR stepRelease(uint32_t m0, uint32_t m1) {
  if (m0) GPIO.out_w1tc = m0;
  if (m1) GPIO.out1_w1tc.val = m1;
}
#define STEP_IDLE_LEVEL LOW
#endif

// ==========================================================================
// 5. Timer ISR -- ปั่นพัลส์อย่างเดียว ห้ามมี float / Serial / delay ในนี้
// --------------------------------------------------------------------------
// จังหวะ: เคลียร์ขา STEP ที่ค้างจาก tick ที่แล้วก่อน แล้วค่อยยกขาที่ต้องก้าว
// ทำให้ความกว้างพัลส์ = 1 tick (50 us) โดยไม่ต้อง busy-wait ใน ISR เลย
// DM556 ต้องการพัลส์กว้างอย่างน้อย 2.5 us -> เหลือเฟือ
// ==========================================================================
static void IRAM_ATTR onStepTimer() {
  stepRelease(stepMaskAll0, stepMaskAll1);

  uint32_t fire0 = 0, fire1 = 0;

  for (uint8_t i = 0; i < AXIS_COUNT; i++) {
    Axis &a = ax[i];

    // ลิมิตสวิตช์: เช็คทุก tick เพื่อให้หยุดได้ภายใน 50 us
    if (PIN_LIMIT[i] >= 0) {
      bool hit = fastRead(PIN_LIMIT[i]);
#if LIMIT_ACTIVE_LOW
      hit = !hit;
#endif
      a.limitHit = hit;
      if (hit && !a.ignoreLimit) { a.inc = 0; a.acc = 0; continue; }
    }

    uint32_t inc = a.inc;
    if (!inc) continue;

    uint32_t prev = a.acc;
    a.acc = prev + inc;
    if (a.acc < prev) {                 // accumulator ล้น = ถึงเวลาก้าว 1 step
      a.pos += a.dirSign;
      fire0 |= stepBit0[i];
      fire1 |= stepBit1[i];
    }
  }

  stepAssert(fire0, fire1);
}

// ==========================================================================
// 6. ตัวช่วยฝั่ง planner
// ==========================================================================
static void setDrvEnable(bool on) {
  drvEnabled = on;
#if ENABLE_ACTIVE_LOW
  digitalWrite(PIN_ENABLE, on ? LOW : HIGH);
#else
  digitalWrite(PIN_ENABLE, on ? HIGH : LOW);
#endif
}

static int32_t axisGetPos(uint8_t i) {
  portENTER_CRITICAL(&mux);
  int32_t p = ax[i].pos;
  portEXIT_CRITICAL(&mux);
  return p;
}

// หยุดแกนทันที ไม่มี ramp (ใช้ตอน E-stop / ชนลิมิต / home เจอสวิตช์)
static void axisHardStop(uint8_t i) {
  portENTER_CRITICAL(&mux);
  ax[i].inc = 0;
  ax[i].acc = 0;
  ax[i].target = ax[i].pos;
  portEXIT_CRITICAL(&mux);
  ax[i].v = 0.0f;
}

static void axisSetPos(uint8_t i, int32_t p) {
  portENTER_CRITICAL(&mux);
  ax[i].pos = p;
  ax[i].target = p;
  ax[i].acc = 0;
  ax[i].inc = 0;
  portEXIT_CRITICAL(&mux);
  ax[i].v = 0.0f;
}

static void axisSetTarget(uint8_t i, int32_t t) {
  Axis &a = ax[i];
  if (a.softLimitOn) {
    if (t < a.softMin) t = a.softMin;
    if (t > a.softMax) t = a.softMax;
  }
  a.target = t;
}

// เบรกทุกแกนแบบมี ramp: ตั้งเป้าไว้ที่ระยะที่เบรกทันพอดี ปล่อยให้ planner ชะลอเอง
static void stopAll() {
  for (uint8_t i = 0; i < AXIS_COUNT; i++) {
    Axis &a = ax[i];
    a.homeState   = HOME_IDLE;
    a.ignoreLimit = false;
    a.streamMode  = false;
    float d = (a.v * a.v) / (2.0f * a.accel);
    a.target = axisGetPos(i) + (int32_t)((a.v >= 0 ? d : -d) + 0.5f);
  }
}

static void estopAll(const char *why) {
  for (uint8_t i = 0; i < AXIS_COUNT; i++) {
    axisHardStop(i);
    ax[i].homeState   = HOME_IDLE;
    ax[i].ignoreLimit = false;
    ax[i].streamMode  = false;
  }
  sysFault = true;
  setDrvEnable(false);
  Serial.print(F("# ESTOP "));
  Serial.println(why);
}

// ==========================================================================
// 7. Planner (เรียกทุก 1 ms)
// --------------------------------------------------------------------------
// มีสองโหมด:
//   planAxis   -- trapezoidal ramp ในบอร์ด ใช้กับคำสั่ง M / R / MA และตอน home
//   planStream -- วิ่งตาม setpoint ที่ host ป้อนมาทุกคาบ ใช้กับ ros2_control
// ทั้งสองจบด้วย applyRate() ที่แปลง a.v เป็นค่า inc ให้ ISR
// ==========================================================================

// แปลงความเร็ว (step/s มีเครื่องหมาย) เป็นค่า inc + จัดการการเปลี่ยนทิศ
static void applyRate(uint8_t i) {
  Axis &a = ax[i];
  float vabs = fabsf(a.v);
  uint32_t inc = (vabs < 1.0f) ? 0 : (uint32_t)(vabs * INC_PER_HZ);
  int8_t   dir = (a.v > 0) ? 1 : ((a.v < 0) ? -1 : a.dirSign);

  portENTER_CRITICAL(&mux);
  if (inc && dir != a.dirSign) {
    // เปลี่ยนทิศ: เขียนขา DIR รอบนี้ แล้วเริ่มก้าวรอบถัดไป
    // DM556 ต้องการ DIR นิ่งก่อนพัลส์ >= 5 us -- เว้นให้ 1 ms ไปเลย
    a.dirSign = dir;
    a.inc = 0;
    a.acc = 0;
    portEXIT_CRITICAL(&mux);
    bool level = (dir > 0);
    if (DIR_INVERT[i]) level = !level;
    digitalWrite(PIN_DIR[i], level ? HIGH : LOW);
    return;
  }
  a.inc = inc;
  portEXIT_CRITICAL(&mux);
}

// เงื่อนไขที่ทำให้แกนต้องหยุดนิ่ง ใช้ร่วมกันทั้งสองโหมด
// คืน true = จัดการให้แล้ว ผู้เรียกไม่ต้องคิดต่อ
static bool axisBlocked(uint8_t i) {
  Axis &a = ax[i];
  if (sysFault || !drvEnabled || a.fault) {
    a.v = 0.0f;
    portENTER_CRITICAL(&mux);
    a.inc = 0;
    portEXIT_CRITICAL(&mux);
    return true;
  }
  return false;
}

static void planAxis(uint8_t i, float dt) {
  Axis &a = ax[i];

  if (axisBlocked(i)) return;

  int32_t pos = axisGetPos(i);
  int32_t rem = a.target - pos;

  // ชนลิมิตขณะเคลื่อนที่ปกติ = ผิดปกติ ตั้ง fault ให้แกนนั้น รอ host สั่ง C เคลียร์
  if (a.limitHit && !a.ignoreLimit && rem != 0) {
    a.fault = true;
    axisHardStop(i);
    Serial.print(F("# LIMIT axis ")); Serial.println(i);
    return;
  }

  float  vabs = fabsf(a.v);
  int8_t want = (rem > 0) ? 1 : ((rem < 0) ? -1 : 0);

  if (want == 0) {
    // ถึงเป้าแล้ว: ถ้าเหลือแค่ความเร็วคืบ ให้ตัดเป็นศูนย์ทันที
    // ถ้าปล่อยให้ ramp ลงตามปกติ มันจะเลยเป้าไป 1 step แล้วถอยกลับสลับไปมาไม่จบ
    if (vabs <= 2.0f * CREEP_RATE) {
      vabs = 0.0f;
      portENTER_CRITICAL(&mux);
      a.acc = 0;
      portEXIT_CRITICAL(&mux);
    } else {
      vabs -= a.accel * dt;      // ถูกสั่งให้หยุดกลางทางตอนยังวิ่งเร็ว -> ชะลอตามปกติ
      if (vabs < 0.0f) vabs = 0.0f;
    }
    a.v = (a.v >= 0 ? vabs : -vabs);
  } else {
    bool  toward = (a.v == 0.0f) || ((a.v > 0.0f) == (rem > 0));
    float dist   = fabsf((float)rem);

    if (!toward) {
      // วิ่งผิดทาง (เป้าถูกเปลี่ยนกลางทาง) -> เบรกให้ถึงศูนย์ก่อนแล้วค่อยกลับ
      vabs -= a.accel * dt;
      if (vabs < 0.0f) vabs = 0.0f;
      a.v = (a.v > 0 ? vabs : -vabs);
    } else {
      // ระยะเบรก: สูตร v^2/2a คิดบนเวลาต่อเนื่อง แต่ของจริง planner เดินเป็นขั้นละ 1 ms
      // จึงต้องเผื่อระยะที่วิ่งไประหว่างขั้นด้วย ค่า 1.5 ได้จากการกวาดพารามิเตอร์
      // (vmax 500-9000, accel 200-200000, ระยะ 1-20000 step) แล้วไม่เลยเป้าเลยสักเคส
      float brakeDist = (vabs * vabs) / (2.0f * a.accel) + BRAKE_MARGIN * vabs * dt;
      if (dist <= FINAL_APPROACH_STEPS || dist <= brakeDist) {
        // ช่วง 2 step สุดท้ายห้ามเร่งเด็ดขาด ถ้าปล่อยให้เร่งไล่ตามระยะที่เหลือ
        // มันจะเลยเป้า 1 step แล้วกลับทิศมาแก้ สลับไปมาไม่จบ (มอเตอร์สั่นค้าง)
        vabs -= a.accel * dt;
        if (vabs < CREEP_RATE) vabs = CREEP_RATE;   // คืบเข้าเป้า ไม่งั้นค้างห่าง 1-2 step
      } else {
        vabs += a.accel * dt;
        if (vabs > a.vmax) vabs = a.vmax;
      }
      if (vabs > RATE_MAX) vabs = RATE_MAX;
      a.v = want * vabs;
    }
  }

  applyRate(i);
}

// --------------------------------------------------------------------------
// โหมด streaming: host (ros2_control) ส่ง setpoint มาให้ทุกคาบควบคุม
// ความเร็วที่สั่ง = feed-forward + ตัวแก้ค่าคลาด
//   feed-forward  = ส่วนต่างของ setpoint สองครั้งล่าสุด หารด้วยคาบ
//                   ตัวนี้ทำให้ไม่มีค่าคลาดค้าง (steady-state lag) ตอนวิ่งความเร็วคงที่
//   ตัวแก้ค่าคลาด = (setpoint - ตำแหน่งจริง) / คาบ  -> ค่าคลาดสลายตัวแบบ first-order
//                   ด้วยค่าคงตัวเวลาเท่ากับคาบควบคุมพอดี และไม่มีทางเลยเป้า
// แล้วจำกัดอัตราการเปลี่ยนความเร็วด้วย accel อีกชั้น กัน setpoint กระโดดจนสเต็ปหลุด
// --------------------------------------------------------------------------
static void planStream(uint8_t i, float dt) {
  Axis &a = ax[i];
  if (axisBlocked(i)) return;

  if (a.limitHit && !a.ignoreLimit) {
    a.fault = true;
    axisHardStop(i);
    Serial.print(F("# LIMIT axis ")); Serial.println(i);
    return;
  }

  uint32_t toMs = (uint32_t)(ctrlDt * 1000.0f) * STREAM_TIMEOUT_MULT;
  if (toMs < STREAM_TIMEOUT_MIN_MS) toMs = STREAM_TIMEOUT_MIN_MS;

  float want;
  if (millis() - lastStreamMs > toMs) {
    want = 0.0f;                       // host หยุดป้อน setpoint -> ชะลอลงศูนย์
  } else {
    float err = (float)(a.streamTarget - axisGetPos(i));
    want = a.streamFF + err / ctrlDt;
  }

  float cap = (a.vmax < RATE_MAX) ? a.vmax : RATE_MAX;
  if (want >  cap) want =  cap;
  if (want < -cap) want = -cap;

  float dv = want - a.v;
  float dvMax = a.accel * dt;
  if (dv >  dvMax) want = a.v + dvMax;
  if (dv < -dvMax) want = a.v - dvMax;

  a.v = want;
  applyRate(i);
}

// เข้า/ออกโหมด streaming
static void streamEnter(uint8_t i) {
  Axis &a = ax[i];
  if (a.streamMode) return;
  a.streamMode   = true;
  a.homeState    = HOME_IDLE;
  a.ignoreLimit  = false;
  a.streamTarget = axisGetPos(i);     // เริ่มจากตำแหน่งปัจจุบัน กันกระตุกตอนเข้าโหมด
  a.streamFF     = 0.0f;
}

static void streamExit(uint8_t i) {
  Axis &a = ax[i];
  if (!a.streamMode) return;
  a.streamMode = false;
  a.target     = axisGetPos(i);
}

// ==========================================================================
// 8. Homing state machine (เรียกทุก 1 ms เหมือนกัน)
// ==========================================================================
static void startHome(uint8_t i, int8_t dir) {
  Axis &a = ax[i];
  if (PIN_LIMIT[i] < 0) { Serial.println(F("ERR nolimit")); return; }
  a.streamMode  = false;
  a.fault       = false;
  a.homed       = false;
  a.homeDir     = (dir >= 0) ? 1 : -1;
  a.homeStartMs = millis();
  a.ignoreLimit = true;
  a.softLimitOn = false;                 // ระหว่าง home ยังไม่รู้ระบบพิกัด
  a.v           = 0.0f;
  a.vmax        = HOME_SEEK_RATE;

  if (a.limitHit) {
    // เริ่มมาก็ทับสวิตช์อยู่แล้ว -> ข้ามไปถอยออกเลย
    a.homeState = HOME_BACKOFF;
    axisSetTarget(i, axisGetPos(i) - a.homeDir * HOME_BACKOFF_STEPS);
  } else {
    a.homeState = HOME_SEEK;
    axisSetTarget(i, axisGetPos(i) + a.homeDir * 100000000L);
  }
}

static void homeTick(uint8_t i) {
  Axis &a = ax[i];
  if (a.homeState == HOME_IDLE) return;

  if (millis() - a.homeStartMs > HOME_TIMEOUT_MS) {
    a.homeState   = HOME_IDLE;
    a.ignoreLimit = false;
    a.fault       = true;
    a.vmax        = DEF_VMAX;      // คืนความเร็วปกติ ไม่งั้นค้างอยู่ที่ HOME_SLOW_RATE
    axisHardStop(i);
    Serial.print(F("# HOMEFAIL axis ")); Serial.println(i);
    return;
  }

  switch (a.homeState) {
    case HOME_SEEK:
      if (a.limitHit) {
        axisHardStop(i);
        a.homeState = HOME_BACKOFF;
        axisSetTarget(i, axisGetPos(i) - a.homeDir * HOME_BACKOFF_STEPS);
      }
      break;

    case HOME_BACKOFF:
      if (!a.limitHit && a.inc == 0 && a.target == axisGetPos(i)) {
        a.homeState = HOME_SLOW;
        a.vmax      = HOME_SLOW_RATE;
        axisSetTarget(i, axisGetPos(i) + a.homeDir * (HOME_BACKOFF_STEPS * 3));
      }
      break;

    case HOME_SLOW:
      if (a.limitHit) {
        axisHardStop(i);
        axisSetPos(i, a.homeZero);
        a.homeState   = HOME_SETTLE;
        a.ignoreLimit = false;
      }
      break;

    case HOME_SETTLE:
      a.homeState = HOME_IDLE;
      a.homed     = true;
      a.vmax      = DEF_VMAX;      // คืนความเร็วปกติ (host สั่ง V ทับได้)
      Serial.print(F("# HOMED axis ")); Serial.println(i);
      break;
  }
}

// ==========================================================================
// 9. โปรโตคอลกับ ROS  (ASCII บรรทัดละคำสั่ง จบด้วย \n)
// --------------------------------------------------------------------------
//  host -> esp32
//    E 1|0                  เปิด/ปิดไฟเลี้ยงมอเตอร์ (ENA)
//    M <ax> <step>          สั่งตำแหน่งสัมบูรณ์
//    R <ax> <step>          สั่งตำแหน่งสัมพัทธ์
//    MA <p0> <p1> <p2> <p3> สั่งพร้อมกันทั้ง 4 แกน ('*' = แกนนั้นไม่เปลี่ยน)
//    T <dt_us> <p0>..<p3>   setpoint โหมด streaming สำหรับ ros2_control
//                           dt_us = คาบควบคุมของ host, ไม่ตอบ OK กลับ
//                           แกนจะออกจากโหมดนี้เมื่อได้รับ M / R / MA / H / S / X
//    V <ax> <vmax> <accel>  ตั้ง step/s และ step/s^2
//    L <ax> <min> <max>     ตั้งลิมิตซอฟต์แวร์ (min==max = ปิด)
//    Z <ax> [step]          เซ็ตตำแหน่งปัจจุบันเป็นค่าที่ให้ (ไม่ใส่ = 0)
//    H <ax> [dir]           home แกนนั้น (dir -1/+1, ไม่ใส่ = -1)
//    S                      เบรกทุกแกนแบบมี ramp
//    X                      หยุดทันที + ตัด ENA (E-stop)
//    C                      เคลียร์ fault
//    F <hz>                 ความถี่ status stream (0 = ปิด)
//    Q                      ขอ status 1 บรรทัด
//    P                      heartbeat (กัน watchdog ตัด)
//  esp32 -> host
//    S <ms> <sys> [<pos> <vel> <flag>] x4
//    OK <cmd> / ERR <why> / # ข้อความ log
// ==========================================================================
static char    rxBuf[RX_BUF_LEN];
static uint8_t rxLen = 0;

static bool argInt32(char *tok, int32_t &out) {
  if (!tok || !*tok) return false;
  char *end;
  long v = strtol(tok, &end, 10);
  if (end == tok) return false;
  out = (int32_t)v;
  return true;
}

static void sendStatus() {
  uint8_t sys = 0;
  if (drvEnabled) sys |= 0x01;
  if (sysFault)   sys |= 0x02;

  Serial.print('S');           Serial.print(' ');
  Serial.print(millis());      Serial.print(' ');
  Serial.print(sys);

  for (uint8_t i = 0; i < AXIS_COUNT; i++) {
    Axis &a = ax[i];
    uint8_t f = 0;
    if (a.inc)                    f |= AF_MOVING;
    if (a.homed)                  f |= AF_HOMED;
    if (a.limitHit)               f |= AF_LIMIT;
    if (a.fault)                  f |= AF_FAULT;
    if (a.homeState != HOME_IDLE) f |= AF_HOMING;
    if (a.streamMode)             f |= AF_STREAM;

    Serial.print(' '); Serial.print(axisGetPos(i));
    Serial.print(' '); Serial.print((int32_t)a.v);
    Serial.print(' '); Serial.print(f);
  }
  Serial.print('\n');
}

static void handleLine(char *line) {
  lastHostMs = millis();

  char *save = nullptr;
  char *cmd  = strtok_r(line, " \t", &save);
  if (!cmd) return;

  int32_t a0, a1, a2;

  if (!strcmp(cmd, "P")) {                       // heartbeat เฉย ๆ ไม่ต้องตอบ
    return;
  }
  else if (!strcmp(cmd, "Q")) {
    sendStatus();
    return;
  }
  else if (!strcmp(cmd, "E")) {
    if (!argInt32(strtok_r(nullptr, " \t", &save), a0)) { Serial.println(F("ERR arg")); return; }
    if (a0 && sysFault) { Serial.println(F("ERR fault")); return; }
    setDrvEnable(a0 != 0);
  }
  else if (!strcmp(cmd, "M") || !strcmp(cmd, "R")) {
    if (!argInt32(strtok_r(nullptr, " \t", &save), a0) ||
        !argInt32(strtok_r(nullptr, " \t", &save), a1) ||
        a0 < 0 || a0 >= AXIS_COUNT) { Serial.println(F("ERR arg")); return; }
    if (ax[a0].homeState != HOME_IDLE) { Serial.println(F("ERR homing")); return; }
    streamExit((uint8_t)a0);
    axisSetTarget((uint8_t)a0, (cmd[0] == 'M') ? a1 : axisGetPos((uint8_t)a0) + a1);
  }
  else if (!strcmp(cmd, "MA")) {
    for (uint8_t i = 0; i < AXIS_COUNT; i++) {
      char *t = strtok_r(nullptr, " \t", &save);
      if (!t) break;
      if (t[0] == '*') continue;
      int32_t v;
      if (!argInt32(t, v)) { Serial.println(F("ERR arg")); return; }
      if (ax[i].homeState != HOME_IDLE) continue;
      streamExit(i);
      axisSetTarget(i, v);
    }
  }
  else if (!strcmp(cmd, "T")) {
    // T <dt_us> <p0> <p1> <p2> <p3>  -- setpoint จาก ros2_control
    int32_t dtus;
    if (!argInt32(strtok_r(nullptr, " \t", &save), dtus) ||
        dtus < (int32_t)STREAM_DT_MIN_US || dtus > (int32_t)STREAM_DT_MAX_US) {
      Serial.println(F("ERR arg")); return;
    }
    ctrlDt = (float)dtus / 1000000.0f;
    for (uint8_t i = 0; i < AXIS_COUNT; i++) {
      char *t = strtok_r(nullptr, " \t", &save);
      if (!t) break;
      if (t[0] == '*') continue;
      int32_t v;
      if (!argInt32(t, v)) { Serial.println(F("ERR arg")); return; }
      Axis &a = ax[i];
      if (a.homeState != HOME_IDLE) continue;
      streamEnter(i);
      if (a.softLimitOn) {
        if (v < a.softMin) v = a.softMin;
        if (v > a.softMax) v = a.softMax;
      }
      a.streamFF     = (float)(v - a.streamTarget) / ctrlDt;
      a.streamTarget = v;
      a.target       = v;              // ให้ status/ลิมิตยังอ่านความหมายเดียวกัน
    }
    lastStreamMs = millis();
    return;                            // ไม่ตอบ OK ไม่งั้นกินแบนด์วิดท์ทุกคาบ
  }
  else if (!strcmp(cmd, "V")) {
    if (!argInt32(strtok_r(nullptr, " \t", &save), a0) ||
        !argInt32(strtok_r(nullptr, " \t", &save), a1) ||
        !argInt32(strtok_r(nullptr, " \t", &save), a2) ||
        a0 < 0 || a0 >= AXIS_COUNT || a1 <= 0 || a2 <= 0) { Serial.println(F("ERR arg")); return; }
    ax[a0].vmax  = (a1 > (int32_t)RATE_MAX) ? RATE_MAX : (float)a1;
    ax[a0].accel = (float)a2;
  }
  else if (!strcmp(cmd, "L")) {
    if (!argInt32(strtok_r(nullptr, " \t", &save), a0) ||
        !argInt32(strtok_r(nullptr, " \t", &save), a1) ||
        !argInt32(strtok_r(nullptr, " \t", &save), a2) ||
        a0 < 0 || a0 >= AXIS_COUNT) { Serial.println(F("ERR arg")); return; }
    ax[a0].softMin     = a1;
    ax[a0].softMax     = a2;
    ax[a0].softLimitOn = (a1 != a2);
  }
  else if (!strcmp(cmd, "Z")) {
    if (!argInt32(strtok_r(nullptr, " \t", &save), a0) || a0 < 0 || a0 >= AXIS_COUNT) {
      Serial.println(F("ERR arg")); return;
    }
    if (!argInt32(strtok_r(nullptr, " \t", &save), a1)) a1 = 0;
    axisSetPos((uint8_t)a0, a1);
  }
  else if (!strcmp(cmd, "H")) {
    if (!argInt32(strtok_r(nullptr, " \t", &save), a0) || a0 < 0 || a0 >= AXIS_COUNT) {
      Serial.println(F("ERR arg")); return;
    }
    if (!drvEnabled) { Serial.println(F("ERR disabled")); return; }
    if (!argInt32(strtok_r(nullptr, " \t", &save), a1)) a1 = -1;
    startHome((uint8_t)a0, (int8_t)a1);
  }
  else if (!strcmp(cmd, "S")) {
    stopAll();
  }
  else if (!strcmp(cmd, "X")) {
    estopAll("host");
  }
  else if (!strcmp(cmd, "C")) {
    sysFault = false;
    for (uint8_t i = 0; i < AXIS_COUNT; i++) {
      ax[i].fault = false;
      axisHardStop(i);
    }
  }
  else if (!strcmp(cmd, "F")) {
    if (!argInt32(strtok_r(nullptr, " \t", &save), a0) || a0 < 0 || a0 > 200) {
      Serial.println(F("ERR arg")); return;
    }
    statusHz = (uint16_t)a0;
  }
  else {
    Serial.println(F("ERR cmd"));
    return;
  }

  Serial.print(F("OK ")); Serial.println(cmd);
}

static void pollSerial() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      rxBuf[rxLen] = '\0';
      if (rxLen) handleLine(rxBuf);
      rxLen = 0;
    } else if (rxLen < RX_BUF_LEN - 1) {
      rxBuf[rxLen++] = c;
    } else {
      rxLen = 0;                       // บรรทัดยาวเกิน = ทิ้งทั้งบรรทัด
      Serial.println(F("ERR long"));
    }
  }
}

// ==========================================================================
// 10. setup / loop
// ==========================================================================
void setup() {
  Serial.begin(SERIAL_BAUD);

  pinMode(PIN_ENABLE, OUTPUT);
  setDrvEnable(false);

  for (uint8_t i = 0; i < AXIS_COUNT; i++) {
    pinMode(PIN_STEP[i], OUTPUT);
    digitalWrite(PIN_STEP[i], STEP_IDLE_LEVEL);
    pinMode(PIN_DIR[i], OUTPUT);
    digitalWrite(PIN_DIR[i], DIR_INVERT[i] ? LOW : HIGH);
    if (PIN_LIMIT[i] >= 0) {
      pinMode(PIN_LIMIT[i], LIMIT_ACTIVE_LOW ? INPUT_PULLUP : INPUT_PULLDOWN);
    }

    stepBit0[i]   = (PIN_STEP[i] <  32) ? (1UL <<  PIN_STEP[i])       : 0;
    stepBit1[i]   = (PIN_STEP[i] >= 32) ? (1UL << (PIN_STEP[i] - 32)) : 0;
    stepMaskAll0 |= stepBit0[i];
    stepMaskAll1 |= stepBit1[i];

    Axis &a = ax[i];
    a.pos = a.target = 0;
    a.inc = a.acc = 0;
    a.dirSign     = 1;
    a.v           = 0.0f;
    a.vmax        = DEF_VMAX;
    a.accel       = DEF_ACCEL;
    a.softMin     = a.softMax = 0;
    a.softLimitOn = false;
    a.homed       = a.fault = false;
    a.limitHit    = a.ignoreLimit = false;
    a.homeState   = HOME_IDLE;
    a.homeDir     = -1;
    a.homeZero    = 0;
    a.homeStartMs = 0;
    a.streamMode   = false;
    a.streamTarget = 0;
    a.streamFF     = 0.0f;
  }

  // timer นับที่ 1 MHz แล้วยิงทุก (1e6 / ISR_HZ) us
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  stepTimer = timerBegin(1000000);
  timerAttachInterrupt(stepTimer, &onStepTimer);
  timerAlarm(stepTimer, 1000000UL / ISR_HZ, true, 0);
#else
  stepTimer = timerBegin(0, 80, true);
  timerAttachInterrupt(stepTimer, &onStepTimer, true);
  timerAlarmWrite(stepTimer, 1000000UL / ISR_HZ, true);
  timerAlarmEnable(stepTimer);
#endif

  lastHostMs = millis();
  Serial.println(F("# esp32_dm556_stepper ready axes=4"));
}

void loop() {
  static uint32_t tPlan = 0, tStat = 0;
  uint32_t now = micros();

  pollSerial();

  if ((uint32_t)(now - tPlan) >= PLANNER_PERIOD_US) {
    tPlan = now;
    const float dt = PLANNER_PERIOD_US / 1000000.0f;

    for (uint8_t i = 0; i < AXIS_COUNT; i++) {
      homeTick(i);
      if (ax[i].streamMode) planStream(i, dt);
      else                  planAxis(i, dt);
    }

    // host เงียบนานเกินไป = สายหลุด / node ตาย -> เบรกไว้ก่อน
    if (HOST_TIMEOUT_MS && (millis() - lastHostMs > HOST_TIMEOUT_MS)) {
      bool moving = false;
      for (uint8_t i = 0; i < AXIS_COUNT; i++) if (ax[i].inc) { moving = true; break; }
      if (moving) {
        stopAll();
        Serial.println(F("# host timeout, braking"));
      }
      lastHostMs = millis();            // เบรกครั้งเดียว ไม่ยิงซ้ำทุก ms
    }
  }

  if (statusHz) {
    uint32_t period = 1000000UL / statusHz;
    if ((uint32_t)(now - tStat) >= period) {
      tStat = now;
      sendStatus();
    }
  }
}
