/* ==========================================================================
 * test_stepper_only -- ทดสอบสเต็ปปิงมอเตอร์ + DM556 ล้วน ๆ
 * --------------------------------------------------------------------------
 * สเก็ตช์นี้ไม่เกี่ยวกับ ROS และไม่มีโปรโตคอลอะไรทั้งสิ้น
 * มีแค่ "ยิงพัลส์ออกไปแล้วเพลาหมุนไหม" เพื่อไล่ปัญหาการต่อสายทีละจุด
 * ให้รันตัวนี้ให้ผ่านก่อน แล้วค่อยอัปโหลด esp32_dm556_stepper.ino ตัวจริง
 *
 * บอร์ด : ESP32 Dev Module,  Serial Monitor 115200 (ตั้ง line ending เป็น Newline)
 * ไลบรารี : ไม่ต้องลงอะไรเพิ่ม
 *
 * ต่างจากเฟิร์มแวร์ตัวจริงตรงที่นี่ปั่นพัลส์แบบบล็อกด้วย delayMicroseconds()
 * ตรงไปตรงมา อ่านง่าย ตรวจสอบด้วยตาได้ ไม่มี timer ISR ไม่มี accumulator
 * เพราะจุดประสงค์คือแยกให้ชัดว่าปัญหาอยู่ที่สายไฟหรืออยู่ที่ตรรกะในโค้ด
 *
 * !! ขาต้องตรงกับ esp32_dm556_stepper.ino  ถ้าแก้ที่นั่นต้องแก้ที่นี่ด้วย !!
 * ========================================================================== */

#include <Arduino.h>

// ==========================================================================
// 1. ขา GPIO (ก๊อปมาจากเฟิร์มแวร์ตัวจริง)
// ==========================================================================
#define AXIS_COUNT 4

static const uint8_t PIN_STEP[AXIS_COUNT]  = { 25, 27, 32, 18 };
static const uint8_t PIN_DIR [AXIS_COUNT]  = { 26, 14, 33, 19 };
static const int8_t  PIN_LIMIT[AXIS_COUNT] = { 16, 17,  4, 13 };   // -1 = ไม่มีลิมิตสวิตช์

#define PIN_ENABLE          23
#define STEP_ACTIVE_LOW      1   // ต่อ common-anode -> พัลส์เป็นช่วง LOW
#define ENABLE_ACTIVE_LOW    0   // DM556 ออปโต ENA ติด = ปิดไดรเวอร์
#define LIMIT_ACTIVE_LOW     1

static const bool DIR_INVERT[AXIS_COUNT] = { false, false, false, false };

// ==========================================================================
// 2. ค่าตั้งต้น (แก้ระหว่างรันได้จากเมนู ไม่ต้องคอมไพล์ใหม่)
// ==========================================================================
#define PULSE_US            5     // ความกว้างพัลส์ DM556 ต้องการอย่างน้อย 2.5 us
#define DIR_SETUP_US       20     // เว้นเวลาหลังเปลี่ยนขา DIR ก่อนยิงพัลส์ (สเปก >= 5 us)

#define DEF_MICROSTEP    1600L    // pulse ต่อรอบ ตามที่ตั้ง DIP SW1-SW3 ของ DM556
#define DEF_RATE          800.0f  // step/s
#define DEF_START_RATE    200.0f  // step/s ความเร็วตอนเริ่มออกตัว
#define DEF_RAMP_STEPS    400L    // ใช้กี่ step ในการไต่จาก start ไปถึง rate

static uint8_t axis        = 0;
static long    microstep   = DEF_MICROSTEP;
static float   rate        = DEF_RATE;
static float   startRate   = DEF_START_RATE;
static long    rampSteps   = DEF_RAMP_STEPS;
static long    moveCount   = DEF_MICROSTEP;   // จำนวน step ต่อการสั่งหนึ่งครั้ง
static bool    enabled     = false;
static bool    ignoreLimit = false;           // เปิดไว้ตอนยังไม่ได้ต่อลิมิตสวิตช์

static long    pulseTotal  = 0;               // นับพัลส์สะสม ใช้เทียบกับรอบที่นับด้วยตา

// ==========================================================================
// 3. ระดับล่างสุด -- ยิงพัลส์ / ตั้งทิศ / จ่ายไฟ / อ่านลิมิต
// ==========================================================================
static inline void stepPulse(uint8_t a) {
#if STEP_ACTIVE_LOW
  digitalWrite(PIN_STEP[a], LOW);
  delayMicroseconds(PULSE_US);
  digitalWrite(PIN_STEP[a], HIGH);
#else
  digitalWrite(PIN_STEP[a], HIGH);
  delayMicroseconds(PULSE_US);
  digitalWrite(PIN_STEP[a], LOW);
#endif
  pulseTotal++;
}

static void setDir(uint8_t a, bool positive) {
  bool level = positive;
  if (DIR_INVERT[a]) level = !level;
  digitalWrite(PIN_DIR[a], level ? HIGH : LOW);
  delayMicroseconds(DIR_SETUP_US);
}

static void setEnable(bool on) {
  enabled = on;
#if ENABLE_ACTIVE_LOW
  digitalWrite(PIN_ENABLE, on ? LOW : HIGH);
#else
  digitalWrite(PIN_ENABLE, on ? HIGH : LOW);
#endif
  delay(50);                        // DM556 ต้องใช้เวลาตั้งตัวหลังเปลี่ยนสถานะ ENA
}

static bool limitHit(uint8_t a) {
  if (PIN_LIMIT[a] < 0) return false;
  bool v = digitalRead(PIN_LIMIT[a]);
#if LIMIT_ACTIVE_LOW
  v = !v;
#endif
  return v;
}

// ==========================================================================
// 4. สั่งเดิน n step พร้อม ramp ขึ้น-ลง
// --------------------------------------------------------------------------
// ต้องมี ramp เพราะสเต็ปปิงออกตัวที่ความเร็วสูงทันทีไม่ได้ มันจะครวญอยู่กับที่
// ถ้าเห็นอาการนั้นแล้วนึกว่าสายผิด ทั้งที่จริงแค่ ramp สั้นไป
// คืน false = ถูกขัดจังหวะ (กดปุ่ม หรือชนลิมิต)
// ==========================================================================
static bool moveSteps(uint8_t a, long n, bool positive) {
  if (!enabled) {
    Serial.println(F("!! ยังไม่ได้จ่ายไฟมอเตอร์ กด e ก่อน"));
    return false;
  }
  if (n <= 0) return true;

  setDir(a, positive);

  long ramp = rampSteps;
  if (ramp > n / 2) ramp = n / 2;       // ระยะสั้นเกินกว่าจะไต่เต็มที่

  const unsigned long t0 = millis();
  for (long i = 0; i < n; i++) {
    if (!ignoreLimit && limitHit(a)) {
      Serial.print(F("\n!! ชนลิมิตสวิตช์ที่ step ")); Serial.println(i);
      return false;
    }
    if (Serial.available()) {
      while (Serial.available()) Serial.read();
      Serial.print(F("\n-- ถูกขัดจังหวะที่ step ")); Serial.println(i);
      return false;
    }

    float v;
    const long remain = n - i;
    if (ramp > 0 && i < ramp)            v = startRate + (rate - startRate) * (float)i / (float)ramp;
    else if (ramp > 0 && remain <= ramp) v = startRate + (rate - startRate) * (float)remain / (float)ramp;
    else                                 v = rate;
    if (v < 1.0f) v = 1.0f;

    unsigned long period = (unsigned long)(1000000.0f / v);
    if (period <= PULSE_US) period = PULSE_US + 1;

    stepPulse(a);
    delayMicroseconds(period - PULSE_US);
  }

  const unsigned long dt = millis() - t0;
  Serial.print(F("   เดิน ")); Serial.print(n);
  Serial.print(F(" step ทิศ ")); Serial.print(positive ? '+' : '-');
  Serial.print(F("  ใช้เวลา ")); Serial.print(dt); Serial.print(F(" ms"));
  if (dt > 0) {
    Serial.print(F("  = ")); Serial.print((long)(1000.0f * n / dt)); Serial.print(F(" step/s จริง"));
  }
  Serial.println();
  return true;
}

// ==========================================================================
// 5. ชุดทดสอบอัตโนมัติ -- ไล่ทีละข้อตามลำดับที่ควรตรวจ
// ==========================================================================
static void selfTest() {
  Serial.println();
  Serial.print(F("=========== ชุดทดสอบแกน ")); Serial.println(axis);

  Serial.println(F("[1] ตัดไฟมอเตอร์ -- ลองหมุนเพลาด้วยมือ ต้องหมุนได้ฟรี"));
  setEnable(false);
  delay(3000);

  Serial.println(F("[2] จ่ายไฟมอเตอร์ -- ลองหมุนเพลาด้วยมือ ต้องล็อกแน่นหมุนไม่ได้"));
  Serial.println(F("    ถ้ายังหมุนฟรีอยู่ = ขั้ว ENA กลับ ให้แก้ ENABLE_ACTIVE_LOW เป็น 1"));
  setEnable(true);
  delay(4000);

  Serial.print(F("[3] หมุนไปข้างหน้า 1 รอบ (")); Serial.print(microstep);
  Serial.println(F(" step) -- ดูว่าหมุนจริงและนับรอบด้วยตา"));
  if (!moveSteps(axis, microstep, true)) { Serial.println(F("    ยกเลิก")); return; }
  delay(800);

  Serial.println(F("[4] หมุนกลับ 1 รอบ -- ต้องกลับมาที่จุดเดิมพอดี"));
  Serial.println(F("    ถ้ากลับมาไม่ตรงจุดเดิม = สเต็ปหลุด ให้ลดความเร็วหรือเพิ่มกระแสที่ DIP"));
  if (!moveSteps(axis, microstep, false)) { Serial.println(F("    ยกเลิก")); return; }
  delay(800);

  Serial.println(F("[5] เดินสั้น ๆ 20 step เพื่อดูความละเอียดของ microstep"));
  moveSteps(axis, 20, true);

  Serial.println(F("[6] สถานะลิมิตสวิตช์ตอนนี้:"));
  for (uint8_t i = 0; i < AXIS_COUNT; i++) {
    Serial.print(F("    แกน ")); Serial.print(i); Serial.print(F(": "));
    if (PIN_LIMIT[i] < 0) Serial.println(F("ไม่ได้ตั้งขาไว้"));
    else { Serial.print(limitHit(i) ? F("กดอยู่") : F("ปล่อย"));
           Serial.print(F("  (GPIO")); Serial.print(PIN_LIMIT[i]); Serial.println(F(")")); }
  }
  Serial.println(F("    ลองกดสวิตช์ค้างแล้วกด l เพื่อดูว่าค่าเปลี่ยนตามจริงไหม"));

  setEnable(false);
  Serial.println(F("=========== จบชุดทดสอบ ตัดไฟมอเตอร์แล้ว"));
  Serial.println();
}

// ==========================================================================
// 6. เมนู
// ==========================================================================
static void help() {
  Serial.println();
  Serial.println(F("---------------- คำสั่ง ----------------"));
  Serial.println(F("  a <0-3>   เลือกแกนที่จะทดสอบ"));
  Serial.println(F("  e         สลับจ่าย/ตัดไฟเลี้ยงมอเตอร์"));
  Serial.println(F("  f         เดินไปข้างหน้าตามจำนวนที่ตั้งไว้"));
  Serial.println(F("  b         เดินถอยหลัง"));
  Serial.println(F("  1         ยิงพัลส์เดียว (ดูว่าขยับ 1 microstep จริงไหม)"));
  Serial.println(F("  r         หมุนต่อเนื่องจนกว่าจะกดปุ่มอะไรก็ได้"));
  Serial.println(F("  n <step>  ตั้งจำนวน step ต่อการสั่งหนึ่งครั้ง"));
  Serial.println(F("  s <hz>    ตั้งความเร็วปลาย (step/s)"));
  Serial.println(F("  j <hz>    ตั้งความเร็วตอนออกตัว"));
  Serial.println(F("  p <step>  ตั้งระยะ ramp"));
  Serial.println(F("  m <step>  บอกว่า DIP ตั้ง microstep ไว้เท่าไหร่ (pulse ต่อรอบ)"));
  Serial.println(F("  l         อ่านลิมิตสวิตช์ทุกแกน"));
  Serial.println(F("  i         สลับเปิด/ปิดการสนใจลิมิตสวิตช์"));
  Serial.println(F("  t         ชุดทดสอบอัตโนมัติ (แนะนำให้เริ่มจากอันนี้)"));
  Serial.println(F("  c         ล้างตัวนับพัลส์"));
  Serial.println(F("  ?         แสดงเมนูนี้"));
  Serial.println(F("---------------------------------------"));
}

static void status() {
  Serial.print(F("[แกน ")); Serial.print(axis);
  Serial.print(F("  STEP=GPIO")); Serial.print(PIN_STEP[axis]);
  Serial.print(F(" DIR=GPIO")); Serial.print(PIN_DIR[axis]);
  Serial.print(F("]  ไฟมอเตอร์:")); Serial.print(enabled ? F("จ่าย") : F("ตัด"));
  Serial.print(F("  ")); Serial.print(moveCount); Serial.print(F(" step/ครั้ง"));
  Serial.print(F("  ")); Serial.print((long)rate); Serial.print(F(" step/s"));
  Serial.print(F("  ramp ")); Serial.print(rampSteps);
  Serial.print(F("  ลิมิต:")); Serial.print(ignoreLimit ? F("ไม่สนใจ") : F("สนใจ"));
  Serial.print(F("  พัลส์สะสม ")); Serial.print(pulseTotal);
  Serial.print(F(" = ")); Serial.print((float)pulseTotal / (float)microstep, 2);
  Serial.println(F(" รอบ"));
}

static void runContinuous(bool positive) {
  if (!enabled) { Serial.println(F("!! ยังไม่ได้จ่ายไฟมอเตอร์ กด e ก่อน")); return; }
  Serial.println(F("   หมุนต่อเนื่อง -- กดปุ่มอะไรก็ได้แล้ว Enter เพื่อหยุด"));
  setDir(axis, positive);

  float v = startRate;
  const float stepUp = (rate - startRate) / (float)(rampSteps > 0 ? rampSteps : 1);
  for (;;) {
    if (Serial.available()) { while (Serial.available()) Serial.read(); break; }
    if (!ignoreLimit && limitHit(axis)) { Serial.println(F("   !! ชนลิมิตสวิตช์")); break; }
    if (v < rate) { v += stepUp; if (v > rate) v = rate; }

    unsigned long period = (unsigned long)(1000000.0f / v);
    if (period <= PULSE_US) period = PULSE_US + 1;
    stepPulse(axis);
    delayMicroseconds(period - PULSE_US);
  }
  Serial.println(F("   หยุดแล้ว"));
}

static void handleLine(char *line) {
  char *save = nullptr;
  char *cmd  = strtok_r(line, " \t", &save);
  if (!cmd) return;
  char *arg  = strtok_r(nullptr, " \t", &save);
  const long v = arg ? strtol(arg, nullptr, 10) : 0;

  switch (cmd[0]) {
    case 'a':
      if (arg && v >= 0 && v < AXIS_COUNT) axis = (uint8_t)v;
      else Serial.println(F("!! ต้องเป็น 0-3"));
      break;
    case 'e': setEnable(!enabled); break;
    case 'f': moveSteps(axis, moveCount, true); break;
    case 'b': moveSteps(axis, moveCount, false); break;
    case '1':
      if (!enabled) { Serial.println(F("!! ยังไม่ได้จ่ายไฟมอเตอร์ กด e ก่อน")); break; }
      setDir(axis, true);
      stepPulse(axis);
      Serial.println(F("   ยิง 1 พัลส์"));
      break;
    case 'r': runContinuous(true); break;
    case 'n': if (arg && v > 0) moveCount = v; break;
    case 's': if (arg && v > 0) rate = (float)v; break;
    case 'j': if (arg && v > 0) startRate = (float)v; break;
    case 'p': if (arg && v >= 0) rampSteps = v; break;
    case 'm': if (arg && v > 0) microstep = v; break;
    case 'i':
      ignoreLimit = !ignoreLimit;
      Serial.print(F("   ลิมิตสวิตช์: "));
      Serial.println(ignoreLimit ? F("ไม่สนใจแล้ว (ระวังชนกลไก)") : F("สนใจแล้ว"));
      break;
    case 'c': pulseTotal = 0; break;
    case 'l':
      for (uint8_t i = 0; i < AXIS_COUNT; i++) {
        Serial.print(F("   แกน ")); Serial.print(i); Serial.print(F(": "));
        if (PIN_LIMIT[i] < 0) Serial.println(F("ไม่ได้ตั้งขาไว้"));
        else { Serial.print(limitHit(i) ? F("กดอยู่") : F("ปล่อย"));
               Serial.print(F("  (GPIO")); Serial.print(PIN_LIMIT[i]); Serial.println(F(")")); }
      }
      break;
    case 't': selfTest(); break;
    case '?': help(); break;
    default:  Serial.println(F("!! ไม่รู้จักคำสั่งนี้ กด ? ดูเมนู")); return;
  }
  status();
}

// ==========================================================================
// 7. setup / loop
// ==========================================================================
void setup() {
  Serial.begin(115200);

  pinMode(PIN_ENABLE, OUTPUT);
  for (uint8_t i = 0; i < AXIS_COUNT; i++) {
    pinMode(PIN_STEP[i], OUTPUT);
    digitalWrite(PIN_STEP[i], STEP_ACTIVE_LOW ? HIGH : LOW);
    pinMode(PIN_DIR[i], OUTPUT);
    digitalWrite(PIN_DIR[i], LOW);
    if (PIN_LIMIT[i] >= 0) {
      pinMode(PIN_LIMIT[i], LIMIT_ACTIVE_LOW ? INPUT_PULLUP : INPUT_PULLDOWN);
    }
  }
  setEnable(false);

  delay(300);
  Serial.println();
  Serial.println(F("=========================================="));
  Serial.println(F(" test_stepper_only -- ทดสอบสเต็ปปิง + DM556"));
  Serial.println(F("=========================================="));
  Serial.println(F("ตั้ง Serial Monitor ให้ส่งท้ายบรรทัดเป็น Newline ก่อนใช้งาน"));
  Serial.println(F("เริ่มจากกด t เพื่อรันชุดทดสอบอัตโนมัติ"));
  help();
  status();
}

void loop() {
  static char buf[48];
  static uint8_t len = 0;

  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      buf[len] = '\0';
      if (len) handleLine(buf);
      len = 0;
    } else if (len < sizeof(buf) - 1) {
      buf[len++] = c;
    } else {
      len = 0;
    }
  }
}
