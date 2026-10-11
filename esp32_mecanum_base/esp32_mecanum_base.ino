/* ==========================================================================
 * esp32_mecanum_base -- ฐานขับล้อ mecanum 4 ล้อ (velocity control)
 * --------------------------------------------------------------------------
 * คุยกับ node mecanum_base บน Pi 5 ผ่าน USB serial (อยู่ใน ros2/mecanum_base)
 * node คิด kinematics แล้วส่งความเร็วล้อเป็น step/s ลงมา เฟิร์มแวร์ตัวนี้ปั่นพัลส์
 * นับ step ที่ปล่อยออกไปจริง แล้วส่งกลับขึ้นไปให้ node คิด odometry
 *
 * บอร์ด   : ESP32 Dev Module,  Serial 115200
 * ไลบรารี : ไม่ต้องลงอะไรเพิ่ม  รองรับทั้ง arduino-esp32 core 2.x และ 3.x
 *
 * หลักการ (ยก ISR + planner มาจาก esp32_dm556_stepper.ino ของแขนกล):
 *   - Timer ISR 20 kHz : ปั่นพัลส์ด้วย phase accumulator เลขจำนวนเต็ม
 *                        (ห้ามใช้ float ใน ISR ของ ESP32 -- FPU context ไม่ถูก save)
 *   - Planner 1 kHz    : ramp ความเร็ว 4 ล้อ "พร้อมกัน" ให้ถึงเป้าในจังหวะเดียวกัน
 *                        ถ้าแต่ละล้อ ramp แยกกัน ล้อที่เป้าต่ำจะถึงก่อน หุ่นเป๋ออกข้างระหว่างเร่ง
 *   - ไม่ได้รับคำสั่ง V เกิน CMD_TIMEOUT_MS = ชะลอหยุดเอง (node ตาย / สาย USB หลุด)
 *
 * ลำดับล้อ : 0 = หน้าซ้าย   1 = หน้าขวา   2 = หลังซ้าย   3 = หลังขวา
 * ความเร็วเป็นบวก = ล้อหมุนพาหุ่นไปข้างหน้า
 *
 * !! ตอนนี้มี DM556 2 ตัว (2026-09-16):
 *      ช่อง 0 (STEP 25 / DIR 26) = ไดรเวอร์ซ้าย  -> มอเตอร์หน้าซ้าย + หลังซ้าย ขนานกัน
 *      ช่อง 1 (STEP 27 / DIR 14) = ไดรเวอร์ขวา   -> มอเตอร์หน้าขวา + หลังขวา ขนานกัน
 *      ช่อง 2 (32/33) และช่อง 3 (13/4) ยังไม่มีไดรเวอร์ต่อ  ENA ร่วมที่ GPIO 23
 *    เฟิร์มแวร์ไม่ต้องแก้: เมื่อ vy = 0 สมการให้ FL = RL และ FR = RR อยู่แล้ว
 *    ค่าที่ส่งไปช่อง 0/1 จึงถูกต้อง  เดินหน้า/ถอยหลัง/หมุนซ้าย/หมุนขวาได้ แต่เดินข้างไม่ได้ !!
 *
 * --------------------------------------------------------------------------
 * โปรโตคอล (ASCII บรรทัดละคำสั่ง จบด้วย \n)
 *   host -> บอร์ด
 *     V <w0> <w1> <w2> <w3>   ความเร็วเป้าหมาย step/s มีเครื่องหมาย
 *                             node ส่งต่อเนื่อง ~30 Hz เป็น heartbeat ไปในตัว (ไม่ตอบ OK)
 *     E <0|1>                 ปิด/เปิดไดรเวอร์ (ENA)
 *     S                       ชะลอหยุดทุกล้อ
 *     X                       E-stop: หยุดทันที ตัด ENA ตั้ง fault
 *     C                       เคลียร์ fault (ไดรเวอร์ยังปิดอยู่ ต้องสั่ง E 1 อีกที)
 *     A <vmax> <accel>        เพดานความเร็ว step/s และความเร่ง step/s^2 (ใช้ร่วมทุกล้อ)
 *     F <hz>                  ความถี่ส่ง status 0-200 (0 = ปิด)
 *     Z                       รีเซ็ตตัวนับ step เป็น 0
 *     Q                       ขอ status ทันที
 *   บอร์ด -> host
 *     W <ms> <sys> <p0> <p1> <p2> <p3> <v0> <v1> <v2> <v3>
 *         p = step สะสมของแต่ละล้อ   v = ความเร็วที่กำลังปั่นอยู่ step/s
 *         sys bit0 = ไดรเวอร์เปิด   bit1 = fault   bit2 = หยุดเพราะไม่มีคำสั่ง V
 *     OK <cmd>   ERR <เหตุผล>   # ข้อความ log
 *   ขึ้นต้นด้วย W ไม่ใช่ S เพื่อให้ node แยกออกจากเฟิร์มแวร์แขนกลได้ ถ้าเสียบผิดบอร์ด
 * ========================================================================== */

#include <Arduino.h>

// ==========================================================================
// 1. ขา GPIO   (ต่อแบบ common-anode เหมือนเฟิร์มแวร์แขนกล -- ดู README.md)
// ==========================================================================
#define WHEEL_COUNT 4

// ช่อง 3 (ยังไม่มีไดรเวอร์) ย้ายจาก 18/19 -> 13/4 เมื่อ 2026-09-25 เพื่อให้ 18/19 ใช้กับอัลตราโซนิก
static const uint8_t PIN_STEP[WHEEL_COUNT] = { 25, 27, 32, 13 };
static const uint8_t PIN_DIR [WHEEL_COUNT] = { 26, 14, 33, 4 };

#define PIN_ENABLE          23   // ENA ร่วมทุกไดรเวอร์ (ต่อขนานกัน)

// อัลตราโซนิก HC-SR04 หน้า/หลัง (2026-09-25) -- Echo 5V ผ่านตัวแบ่ง 1k/2k
// ติดกลางหน้า/กลางท้าย สูง ~8 cm เอียงลง ~45° -> ปกติวัดถึงพื้น ~11 cm
// เฟส 1: วัดแล้วส่งบรรทัด "U <หน้า mm> <หลัง mm>" (-1 = ไม่มีเสียงสะท้อน) ยังไม่หยุดล้อเอง
// ขาตามที่ผู้ใช้เทสผ่านมาแล้ว (Trig 5 / Echo 18) -- 16/17/34/35 ต่อแล้วไม่ได้สัญญาณ (25 ก.ย.)
#define PIN_US_TRIG_F        5
#define PIN_US_ECHO_F       18
#define PIN_US_TRIG_R       21
#define PIN_US_ECHO_R       19
#define US_PERIOD_US     50000UL   // ยิงสลับหน้า/หลังทุก 50 ms = ตัวละ 10 Hz (กันเสียงตีกัน)
#define US_TIMEOUT_US    25000UL   // ไม่มีเสียงกลับใน 25 ms (~4 m) = -1

#define STEP_ACTIVE_LOW      1   // 1 = พัลส์เป็นช่วง LOW (ขาพักไว้ที่ HIGH)
#define ENABLE_ACTIVE_LOW    0   // DM556 common-anode: ดึง HIGH = มอเตอร์มีแรงบิด
                                 // (ตรงกับ ENABLE_LEVEL HIGH ใน test_one_driver_4motors)

// ระดับขา DIR ที่ทำให้ล้อพาหุ่นไปข้างหน้า
// test_one_driver_4motors เดินหน้าด้วย DIR = LOW (มอเตอร์ฝั่งขวาสลับสาย A+/A- ไว้แล้ว)
// ล้อไหนหมุนกลับด้าน อย่าแก้ตรงนี้ ให้แก้ param invert_wheels ฝั่ง node แทน ไม่ต้องแฟลชใหม่
#define DIR_FORWARD_LEVEL  LOW

// ==========================================================================
// 2. ค่าที่ปรับได้
// ==========================================================================
#define ISR_HZ              20000UL   // ความถี่ timer ที่ปั่นพัลส์ -> ความเร็วสูงสุด 10,000 step/s
#define PLANNER_PERIOD_US    1000UL   // คาบการคิด ramp (1 kHz)
#define PLANNER_DT_MAX          0.02f // loop ค้างนานกว่านี้ (s) ไม่ต้องไล่ ramp ตามทีเดียว
#define SERIAL_BAUD         115200UL  // ต้องตรงกับ param 'baud' ฝั่ง ROS

#define DEF_VMAX            4000.0f   // step/s    node สั่งทับด้วยคำสั่ง A ตอนต่อเข้ามา
#define DEF_ACCEL           2000.0f   // step/s^2

#define CMD_TIMEOUT_MS        300UL   // node ส่ง V ทุก ~33 ms ขาดหายเกินนี้ = ชะลอหยุด
#define STATUS_HZ_DEFAULT       20    // ความถี่ส่ง status ตอนบูต (node เปลี่ยนด้วยคำสั่ง F)
#define RX_BUF_LEN              96

#define SYS_ENABLED   0x01
#define SYS_FAULT     0x02
#define SYS_TIMEOUT   0x04

// ==========================================================================
// 3. สถานะต่อล้อ
// ==========================================================================
struct Wheel {
  // --- ใช้ร่วมกับ ISR ---
  volatile int32_t  pos;         // step สะสม (มีเครื่องหมาย)
  volatile uint32_t inc;         // ค่าที่บวกเข้า accumulator ทุก tick (0 = หยุด)
  volatile uint32_t acc;
  volatile int8_t   dirSign;     // +1 / -1  ทิศที่ขา DIR ตั้งไว้ตอนนี้

  // --- ใช้เฉพาะใน planner ---
  float v;                       // ความเร็วที่กำลังปั่น (step/s มีเครื่องหมาย)
  float target;                  // ความเร็วเป้าหมายจากคำสั่ง V
};

static Wheel wh[WHEEL_COUNT];

// มาสก์ GPIO สำหรับเขียนพัลส์ทีเดียวทุกล้อ (ESP32 แบ่ง register เป็น 2 ธนาคาร)
static uint32_t stepMaskAll0 = 0, stepMaskAll1 = 0;
static uint32_t stepBit0[WHEEL_COUNT], stepBit1[WHEEL_COUNT];

static hw_timer_t *stepTimer = nullptr;
static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

static bool     drvEnabled  = false;
static bool     sysFault    = false;
static bool     cmdTimedOut = true;     // ตอนบูตยังไม่มีคำสั่ง ถือว่าหยุดรออยู่
static uint16_t statusHz    = STATUS_HZ_DEFAULT;
static uint32_t lastCmdMs   = 0;
static float    vmax        = DEF_VMAX;
static float    accel       = DEF_ACCEL;

static const float INC_PER_HZ = 4294967296.0f / (float)ISR_HZ;
static const float RATE_MAX   = (float)ISR_HZ * 0.5f;

// ==========================================================================
// 4. เขียนขา STEP ตรง register (เร็วพอที่จะเรียกใน ISR)
// ==========================================================================
#if STEP_ACTIVE_LOW
static inline void IRAM_ATTR stepAssert(uint32_t m0, uint32_t m1) {
  if (m0) GPIO.out_w1tc = m0;
  if (m1) GPIO.out1_w1tc.val = m1;
}
static inline void IRAM_ATTR stepRelease(uint32_t m0, uint32_t m1) {
  if (m0) GPIO.out_w1ts = m0;
  if (m1) GPIO.out1_w1ts.val = m1;
}
#else
static inline void IRAM_ATTR stepAssert(uint32_t m0, uint32_t m1) {
  if (m0) GPIO.out_w1ts = m0;
  if (m1) GPIO.out1_w1ts.val = m1;
}
static inline void IRAM_ATTR stepRelease(uint32_t m0, uint32_t m1) {
  if (m0) GPIO.out_w1tc = m0;
  if (m1) GPIO.out1_w1tc.val = m1;
}
#endif
#define STEP_IDLE_LEVEL (STEP_ACTIVE_LOW ? HIGH : LOW)

// ==========================================================================
// 5. Timer ISR -- ปั่นพัลส์อย่างเดียว ห้ามมี float / Serial / delay ในนี้
// --------------------------------------------------------------------------
// เคลียร์ขา STEP ที่ค้างจาก tick ที่แล้วก่อน แล้วค่อยยกขาที่ต้องก้าว
// ความกว้างพัลส์ = 1 tick (50 us) DM556 ต้องการแค่ 2.5 us
// ==========================================================================
static void IRAM_ATTR onStepTimer() {
  stepRelease(stepMaskAll0, stepMaskAll1);

  uint32_t fire0 = 0, fire1 = 0;

  for (uint8_t i = 0; i < WHEEL_COUNT; i++) {
    Wheel &w = wh[i];
    uint32_t inc = w.inc;
    if (!inc) continue;

    uint32_t prev = w.acc;
    w.acc = prev + inc;
    if (w.acc < prev) {                 // accumulator ล้น = ถึงเวลาก้าว 1 step
      w.pos += w.dirSign;
      fire0 |= stepBit0[i];
      fire1 |= stepBit1[i];
    }
  }

  stepAssert(fire0, fire1);
}

// ==========================================================================
// 6. ตัวช่วย
// ==========================================================================
static void setDrvEnable(bool on) {
  drvEnabled = on;
#if ENABLE_ACTIVE_LOW
  digitalWrite(PIN_ENABLE, on ? LOW : HIGH);
#else
  digitalWrite(PIN_ENABLE, on ? HIGH : LOW);
#endif
}

// หยุดทุกล้อทันที ไม่มี ramp
static void hardStopAll() {
  portENTER_CRITICAL(&mux);
  for (uint8_t i = 0; i < WHEEL_COUNT; i++) {
    wh[i].inc = 0;
    wh[i].acc = 0;
  }
  portEXIT_CRITICAL(&mux);
  for (uint8_t i = 0; i < WHEEL_COUNT; i++) {
    wh[i].v = 0.0f;
    wh[i].target = 0.0f;
  }
}

static void estopAll(const char *why) {
  hardStopAll();
  sysFault = true;
  setDrvEnable(false);
  Serial.print(F("# ESTOP "));
  Serial.println(why);
}

// แปลงความเร็ว (step/s มีเครื่องหมาย) เป็นค่า inc + จัดการการเปลี่ยนทิศ
static void applyRate(uint8_t i) {
  Wheel &w = wh[i];
  float vabs = fabsf(w.v);
  uint32_t inc = (vabs < 1.0f) ? 0 : (uint32_t)(vabs * INC_PER_HZ);
  int8_t   dir = (w.v > 0) ? 1 : ((w.v < 0) ? -1 : w.dirSign);

  portENTER_CRITICAL(&mux);
  if (inc && dir != w.dirSign) {
    // เปลี่ยนทิศ: เขียนขา DIR รอบนี้ แล้วเริ่มก้าวรอบถัดไป (DIR นิ่งก่อนพัลส์ >= 1 ms)
    w.dirSign = dir;
    w.inc = 0;
    w.acc = 0;
    portEXIT_CRITICAL(&mux);
    digitalWrite(PIN_DIR[i], (dir > 0) ? DIR_FORWARD_LEVEL : !DIR_FORWARD_LEVEL);
    return;
  }
  w.inc = inc;
  portEXIT_CRITICAL(&mux);
}

// ==========================================================================
// 7. Planner (เรียกทุก ~1 ms)
// --------------------------------------------------------------------------
// ramp แบบซิงค์: ล้อที่ความเร็วต้องเปลี่ยนมากที่สุดเร่งด้วย accel เต็มที่
// ล้ออื่นเปลี่ยนตามสัดส่วนเดียวกัน ทุกล้อจึงถึงเป้าพร้อมกัน
// kinematics ของ mecanum เป็นเชิงเส้น การไล่ในพื้นที่ความเร็วล้อเป็นเส้นตรง
// จึงเท่ากับไล่ความเร็วตัวหุ่น (vx, vy, wz) เป็นเส้นตรงด้วย ทิศการเคลื่อนที่ไม่เพี้ยนระหว่างเร่ง
// ==========================================================================
static void planWheels(float dt) {
  // เช็ค timeout ก่อนเช็คไดรเวอร์ปิด ไม่งั้นตอนไดรเวอร์ปิด บิต SYS_TIMEOUT จะค้างเป็น 0 ตลอด
  if (CMD_TIMEOUT_MS && !cmdTimedOut && (millis() - lastCmdMs > CMD_TIMEOUT_MS)) {
    cmdTimedOut = true;
    bool moving = false;
    for (uint8_t i = 0; i < WHEEL_COUNT; i++) {
      if (wh[i].target != 0.0f || wh[i].v != 0.0f) moving = true;
      wh[i].target = 0.0f;
    }
    if (moving) Serial.println(F("# cmd timeout, braking"));
  }

  if (sysFault || !drvEnabled) {
    hardStopAll();
    return;
  }

  float maxDv = 0.0f;
  for (uint8_t i = 0; i < WHEEL_COUNT; i++) {
    float dv = fabsf(wh[i].target - wh[i].v);
    if (dv > maxDv) maxDv = dv;
  }

  float dvLimit = accel * dt;
  float scale   = (maxDv > dvLimit) ? dvLimit / maxDv : 1.0f;

  for (uint8_t i = 0; i < WHEEL_COUNT; i++) {
    Wheel &w = wh[i];
    w.v = (scale >= 1.0f) ? w.target : w.v + (w.target - w.v) * scale;
    applyRate(i);
  }
}

// ==========================================================================
// 8. Serial
// ==========================================================================
static char    rxBuf[RX_BUF_LEN];
static uint8_t rxLen = 0;

static bool argInt32(char *tok, int32_t &out) {
  if (!tok || !*tok) return false;
  char *end;
  long v = strtol(tok, &end, 10);
  if (end == tok || *end) return false;
  out = (int32_t)v;
  return true;
}

static void sendStatus() {
  uint8_t sys = 0;
  if (drvEnabled)  sys |= SYS_ENABLED;
  if (sysFault)    sys |= SYS_FAULT;
  if (cmdTimedOut) sys |= SYS_TIMEOUT;

  // อ่านตัวนับทั้ง 4 ล้อในจังหวะเดียวกัน ไม่งั้น odometry เห็นล้อคนละช่วงเวลา
  int32_t p[WHEEL_COUNT];
  portENTER_CRITICAL(&mux);
  for (uint8_t i = 0; i < WHEEL_COUNT; i++) p[i] = wh[i].pos;
  portEXIT_CRITICAL(&mux);

  Serial.print('W');      Serial.print(' ');
  Serial.print(millis()); Serial.print(' ');
  Serial.print(sys);
  for (uint8_t i = 0; i < WHEEL_COUNT; i++) { Serial.print(' '); Serial.print(p[i]); }
  for (uint8_t i = 0; i < WHEEL_COUNT; i++) { Serial.print(' '); Serial.print((int32_t)wh[i].v); }
  Serial.print('\n');
}

static void handleLine(char *line) {
  char *save = nullptr;
  char *cmd  = strtok_r(line, " \t", &save);
  if (!cmd) return;

  int32_t a0, a1;

  if (!strcmp(cmd, "V")) {
    int32_t v[WHEEL_COUNT];
    for (uint8_t i = 0; i < WHEEL_COUNT; i++) {
      if (!argInt32(strtok_r(nullptr, " \t", &save), v[i])) { Serial.println(F("ERR arg")); return; }
    }
    lastCmdMs   = millis();
    cmdTimedOut = false;
    if (sysFault || !drvEnabled) return;     // planner บังคับศูนย์อยู่แล้ว

    // เกินเพดาน: ย่อทุกล้อด้วยสัดส่วนเดียวกัน ทิศการเคลื่อนที่ของหุ่นจะไม่เพี้ยน
    float peak = 0.0f;
    for (uint8_t i = 0; i < WHEEL_COUNT; i++) {
      float a = fabsf((float)v[i]);
      if (a > peak) peak = a;
    }
    float cap   = (vmax < RATE_MAX) ? vmax : RATE_MAX;
    float scale = (peak > cap) ? cap / peak : 1.0f;
    for (uint8_t i = 0; i < WHEEL_COUNT; i++) wh[i].target = (float)v[i] * scale;
    return;                                   // ไม่ตอบ OK ไม่งั้นกินแบนด์วิดท์ทุกคาบ
  }
  else if (!strcmp(cmd, "E")) {
    if (!argInt32(strtok_r(nullptr, " \t", &save), a0)) { Serial.println(F("ERR arg")); return; }
    if (a0 && sysFault) { Serial.println(F("ERR fault")); return; }
    hardStopAll();                            // เปิดใหม่ต้องเริ่มจากศูนย์เสมอ
    setDrvEnable(a0 != 0);
  }
  else if (!strcmp(cmd, "S")) {
    for (uint8_t i = 0; i < WHEEL_COUNT; i++) wh[i].target = 0.0f;
  }
  else if (!strcmp(cmd, "X")) {
    estopAll("host");
  }
  else if (!strcmp(cmd, "C")) {
    sysFault = false;
    hardStopAll();
  }
  else if (!strcmp(cmd, "A")) {
    if (!argInt32(strtok_r(nullptr, " \t", &save), a0) ||
        !argInt32(strtok_r(nullptr, " \t", &save), a1) ||
        a0 <= 0 || a1 <= 0) { Serial.println(F("ERR arg")); return; }
    vmax  = ((float)a0 > RATE_MAX) ? RATE_MAX : (float)a0;
    accel = (float)a1;
  }
  else if (!strcmp(cmd, "F")) {
    if (!argInt32(strtok_r(nullptr, " \t", &save), a0) || a0 < 0 || a0 > 200) {
      Serial.println(F("ERR arg")); return;
    }
    statusHz = (uint16_t)a0;
  }
  else if (!strcmp(cmd, "Z")) {
    portENTER_CRITICAL(&mux);
    for (uint8_t i = 0; i < WHEEL_COUNT; i++) wh[i].pos = 0;
    portEXIT_CRITICAL(&mux);
  }
  else if (!strcmp(cmd, "Q")) {
    sendStatus();
    return;
  }
  else {
    Serial.println(F("ERR cmd"));
    return;
  }

  Serial.print(F("OK ")); Serial.println(cmd);
}

static void pollSerial() {
  static bool discard = false;         // กำลังทิ้งส่วนที่เหลือของบรรทัดที่ยาวเกิน
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      rxBuf[rxLen] = '\0';
      if (rxLen && !discard) handleLine(rxBuf);
      rxLen = 0;
      discard = false;
    } else if (discard) {
      continue;
    } else if (rxLen < RX_BUF_LEN - 1) {
      rxBuf[rxLen++] = c;
    } else {
      // บรรทัดยาวเกิน = ทิ้งทั้งบรรทัดจนถึง \n ถ้าตัดแค่หัว ส่วนท้ายจะกลายเป็นคำสั่งใหม่
      // (เช่นท้ายบรรทัดขยะเป็น "E 1" แล้วไดรเวอร์เปิดเอง)
      rxLen = 0;
      discard = true;
      Serial.println(F("ERR long"));
    }
  }
}

// ==========================================================================
// 8b. อัลตราโซนิก (วัดด้วย interrupt ไม่บล็อก loop ของ planner)
// ==========================================================================
static volatile uint32_t usRise[2] = {0, 0}, usWidth[2] = {0, 0};
static volatile bool usDone[2] = {false, false};
static int32_t usMm[2] = {-1, -1};

static void IRAM_ATTR usEdge(uint8_t k, uint8_t pin) {
  uint32_t t = micros();
  if (digitalRead(pin)) { usRise[k] = t; }
  else { usWidth[k] = t - usRise[k]; usDone[k] = true; }
}
static void IRAM_ATTR usEdgeF() { usEdge(0, PIN_US_ECHO_F); }
static void IRAM_ATTR usEdgeR() { usEdge(1, PIN_US_ECHO_R); }

static void usSetup() {
  pinMode(PIN_US_TRIG_F, OUTPUT); digitalWrite(PIN_US_TRIG_F, LOW);
  pinMode(PIN_US_TRIG_R, OUTPUT); digitalWrite(PIN_US_TRIG_R, LOW);
  pinMode(PIN_US_ECHO_F, INPUT);  pinMode(PIN_US_ECHO_R, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_US_ECHO_F), usEdgeF, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_US_ECHO_R), usEdgeR, CHANGE);
}

// เรียกทุกรอบ loop: ครบคาบ -> สรุปผลตัวที่ยิงรอบก่อน แล้วยิงอีกตัว
static void usPoll(uint32_t now) {
  static uint32_t tFire = 0;
  static uint8_t cur = 0;
  if ((uint32_t)(now - tFire) < US_PERIOD_US) return;
  if (usDone[cur]) {
    uint32_t w = usWidth[cur];
    usMm[cur] = (w > 0 && w < US_TIMEOUT_US) ? (int32_t)(w * 343UL / 2000UL) : -1;   // us -> mm (343 m/s ไป-กลับ)
  } else {
    usMm[cur] = -1;
  }
  if (cur == 1) {                                   // ครบคู่ -> ส่งบรรทัดเดียว 10 Hz
    Serial.print('U'); Serial.print(' ');
    Serial.print(usMm[0]); Serial.print(' '); Serial.print(usMm[1]); Serial.print('\n');
  }
  cur ^= 1;
  usDone[cur] = false;
  uint8_t trig = cur ? PIN_US_TRIG_R : PIN_US_TRIG_F;
  digitalWrite(trig, HIGH); delayMicroseconds(10); digitalWrite(trig, LOW);
  tFire = now;
}

// ==========================================================================
// 9. setup / loop
// ==========================================================================
void setup() {
  Serial.setTxBufferSize(512);         // status ไม่ต้องรอ UART ส่งทีละ byte จน planner สะดุด
  Serial.begin(SERIAL_BAUD);

  pinMode(PIN_ENABLE, OUTPUT);
  setDrvEnable(false);                 // เปิดไดรเวอร์เมื่อ node สั่ง E 1 เท่านั้น

  for (uint8_t i = 0; i < WHEEL_COUNT; i++) {
    pinMode(PIN_STEP[i], OUTPUT);
    digitalWrite(PIN_STEP[i], STEP_IDLE_LEVEL);
    pinMode(PIN_DIR[i], OUTPUT);
    digitalWrite(PIN_DIR[i], DIR_FORWARD_LEVEL);

    stepBit0[i]   = (PIN_STEP[i] <  32) ? (1UL <<  PIN_STEP[i])       : 0;
    stepBit1[i]   = (PIN_STEP[i] >= 32) ? (1UL << (PIN_STEP[i] - 32)) : 0;
    stepMaskAll0 |= stepBit0[i];
    stepMaskAll1 |= stepBit1[i];

    Wheel &w = wh[i];
    w.pos     = 0;
    w.inc     = 0;
    w.acc     = 0;
    w.dirSign = 1;
    w.v       = 0.0f;
    w.target  = 0.0f;
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

  usSetup();
  lastCmdMs = millis();
  Serial.println(F("# esp32_mecanum_base ready wheels=4 ultrasonic=2"));
}

void loop() {
  static uint32_t tPlan = micros(), tStat = 0;
  uint32_t now = micros();

  pollSerial();
  usPoll(now);

  uint32_t elapsed = now - tPlan;
  if (elapsed >= PLANNER_PERIOD_US) {
    tPlan = now;
    // ใช้เวลาจริงที่ผ่านไป ไม่ใช่ 1 ms ตายตัว ไม่งั้นช่วงที่ Serial ทำให้ loop ช้า ความเร่งจะต่ำกว่าที่ตั้ง
    float dt = (float)elapsed / 1000000.0f;
    if (dt > PLANNER_DT_MAX) dt = PLANNER_DT_MAX;
    planWheels(dt);
  }

  if (statusHz) {
    uint32_t period = 1000000UL / statusHz;
    if ((uint32_t)(now - tStat) >= period) {
      tStat = now;
      sendStatus();
    }
  }
}
