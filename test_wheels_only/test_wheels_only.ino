/* ==========================================================================
 * test_wheels_only -- ทดสอบล้อขับ 2 ฝั่ง (DM556 ฝั่งละ 1 ตัว ฝั่งละ 2 ล้อ)
 * --------------------------------------------------------------------------
 * ใช้ไล่ปัญหาฐานขับแบบ differential drive ทีละฝั่ง ไม่มี ROS ไม่มีโปรโตคอล
 * ยิงพัลส์ตรง ๆ ด้วย delayMicroseconds() อ่านไล่ด้วยตาได้ทั้งไฟล์
 *
 * บอร์ด : ESP32 Dev Module,  Serial Monitor 115200 (ตั้ง line ending เป็น Newline)
 *
 * !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 *   ยกล้อให้ลอยพ้นพื้นก่อนทดสอบทุกครั้ง ไม่งั้นหุ่นวิ่งหนีตั้งแต่คำสั่งแรก
 * !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 *
 * ต่างจาก test_stepper_only ตรงที่ตัวนี้คิดเป็น "รอบล้อ" กับ "ระยะทาง"
 * ไม่ใช่ step เปล่า ๆ เพราะค่าที่ได้จากตรงนี้จะเอาไปใช้ทำ odometry ต่อ
 * ========================================================================== */

#include <Arduino.h>

// ==========================================================================
// 1. ขา GPIO
// --------------------------------------------------------------------------
// ใช้ขาชุดเดียวกับแกน 0 และ 1 ของ esp32_dm556_stepper.ino เพื่อให้สายเดิมใช้ต่อได้
// ต่อแบบ common-anode เหมือนกัน (PUL+/DIR+/ENA+ รวมเข้า 3V3, GPIO ไปเข้าขา -)
// ==========================================================================
#define PIN_STEP_L   25
#define PIN_DIR_L    26
#define PIN_STEP_R   27
#define PIN_DIR_R    14
#define PIN_ENABLE   23      // ENA ของทั้งสองตัวต่อขนานกัน

#define STEP_ACTIVE_LOW    1   // ต่อ common-anode -> พัลส์เป็นช่วง LOW
#define ENABLE_ACTIVE_LOW  0   // DM556 ออปโต ENA ติด = ปิดไดรเวอร์
                               // ถ้าสั่ง e แล้วล้อยังหมุนฟรี ให้เปลี่ยนเป็น 1

// มอเตอร์ฝั่งขวาติดตั้งกลับด้านกับฝั่งซ้ายเสมอ (หันหน้าเข้าหากัน)
// ถ้าไม่กลับทิศฝั่งใดฝั่งหนึ่ง สั่ง "เดินหน้า" แล้วหุ่นจะหมุนอยู่กับที่แทน
#define DIR_INVERT_L   false
#define DIR_INVERT_R   true

// ==========================================================================
// 2. ค่ากลไก -- ต้องกรอกให้ตรงของจริง ไม่งั้นระยะทางที่พิมพ์ออกมาไม่มีความหมาย
// ==========================================================================
#define STEPS_PER_REV     200L     // สเปกมอเตอร์ 1.8 องศา = 200
#define MICROSTEP           8L     // ตามที่ตั้ง DIP SW1-SW3 ของ DM556
#define GEAR_RATIO         1.0f    // มอเตอร์หมุนกี่รอบ ต่อล้อ 1 รอบ (ต่อตรง = 1.0)
#define WHEEL_DIAMETER_MM 100.0f   // เส้นผ่านศูนย์กลางล้อ

// ==========================================================================
// 3. ค่าตั้งต้น (แก้ระหว่างรันได้จากเมนู)
// ==========================================================================
#define PULSE_US            5      // DM556 ต้องการพัลส์กว้างอย่างน้อย 2.5 us
#define DIR_SETUP_US       20      // เว้นเวลาหลังเปลี่ยนขา DIR (สเปก >= 5 us)

#define DEF_RATE         1600.0f   // step/s
#define DEF_START_RATE    300.0f   // step/s ตอนออกตัว
#define DEF_RAMP_STEPS    800L

enum Side : uint8_t { SIDE_LEFT = 0, SIDE_RIGHT, SIDE_BOTH, SIDE_SPIN };
static const char *SIDE_NAME[] = { "ซ้าย", "ขวา", "สองฝั่งพร้อมกัน", "หมุนอยู่กับที่" };

static uint8_t sel        = SIDE_LEFT;
static float   rate       = DEF_RATE;
static float   startRate  = DEF_START_RATE;
static long    rampSteps  = DEF_RAMP_STEPS;
static float   revs       = 1.0f;      // จำนวนรอบล้อต่อการสั่งหนึ่งครั้ง
static bool    enabled    = false;

static long    pulseL = 0, pulseR = 0;  // ตัวนับพัลส์สะสมแยกฝั่ง

// step ต่อการหมุนล้อ 1 รอบ
static inline long stepsPerWheelRev() {
  return (long)((float)(STEPS_PER_REV * MICROSTEP) * GEAR_RATIO);
}
static inline float mmPerWheelRev() {
  return 3.14159265f * WHEEL_DIAMETER_MM;
}

// ==========================================================================
// 4. ระดับล่างสุด
// ==========================================================================
static inline void pulseHigh(bool useL, bool useR) {
#if STEP_ACTIVE_LOW
  if (useL) digitalWrite(PIN_STEP_L, LOW);
  if (useR) digitalWrite(PIN_STEP_R, LOW);
  delayMicroseconds(PULSE_US);
  if (useL) digitalWrite(PIN_STEP_L, HIGH);
  if (useR) digitalWrite(PIN_STEP_R, HIGH);
#else
  if (useL) digitalWrite(PIN_STEP_L, HIGH);
  if (useR) digitalWrite(PIN_STEP_R, HIGH);
  delayMicroseconds(PULSE_US);
  if (useL) digitalWrite(PIN_STEP_L, LOW);
  if (useR) digitalWrite(PIN_STEP_R, LOW);
#endif
  if (useL) pulseL++;
  if (useR) pulseR++;
}

// forward = true คือทิศที่ทำให้หุ่นเดินหน้า (ไม่ใช่ทิศหมุนของเพลา)
static void setDirL(bool forward) {
  bool lv = forward;
  if (DIR_INVERT_L) lv = !lv;
  digitalWrite(PIN_DIR_L, lv ? HIGH : LOW);
}
static void setDirR(bool forward) {
  bool lv = forward;
  if (DIR_INVERT_R) lv = !lv;
  digitalWrite(PIN_DIR_R, lv ? HIGH : LOW);
}

static void setEnable(bool on) {
  enabled = on;
#if ENABLE_ACTIVE_LOW
  digitalWrite(PIN_ENABLE, on ? LOW : HIGH);
#else
  digitalWrite(PIN_ENABLE, on ? HIGH : LOW);
#endif
  delay(50);
}

// ==========================================================================
// 5. สั่งเดิน พร้อม ramp ขึ้น-ลง
// --------------------------------------------------------------------------
// ต้องมี ramp เพราะสเต็ปปิงออกตัวที่ความเร็วสูงทันทีไม่ได้ มันจะครวญอยู่กับที่
// ฐานขับที่มีน้ำหนักหุ่นถ่วงยิ่งต้องการ ramp ยาวกว่าตอนทดสอบเปล่า ๆ
// คืน false = ถูกขัดจังหวะ
// ==========================================================================
static bool runMove(bool useL, bool useR, bool fwdL, bool fwdR, long steps) {
  if (!enabled) {
    Serial.println(F("!! ยังไม่ได้จ่ายไฟมอเตอร์ กด e ก่อน"));
    return false;
  }
  if (steps <= 0) return true;

  if (useL) setDirL(fwdL);
  if (useR) setDirR(fwdR);
  delayMicroseconds(DIR_SETUP_US);

  long ramp = rampSteps;
  if (ramp > steps / 2) ramp = steps / 2;

  const unsigned long t0 = millis();
  for (long i = 0; i < steps; i++) {
    if (Serial.available()) {
      while (Serial.available()) Serial.read();
      Serial.print(F("\n-- ถูกขัดจังหวะที่ step ")); Serial.println(i);
      return false;
    }

    float v;
    const long remain = steps - i;
    if (ramp > 0 && i < ramp)            v = startRate + (rate - startRate) * (float)i / (float)ramp;
    else if (ramp > 0 && remain <= ramp) v = startRate + (rate - startRate) * (float)remain / (float)ramp;
    else                                 v = rate;
    if (v < 1.0f) v = 1.0f;

    unsigned long period = (unsigned long)(1000000.0f / v);
    if (period <= PULSE_US) period = PULSE_US + 1;

    pulseHigh(useL, useR);
    delayMicroseconds(period - PULSE_US);
  }

  const unsigned long dt = millis() - t0;
  const float wheelRev = (float)steps / (float)stepsPerWheelRev();
  Serial.print(F("   "));  Serial.print(steps); Serial.print(F(" step"));
  Serial.print(F("  = ")); Serial.print(wheelRev, 2); Serial.print(F(" รอบล้อ"));
  Serial.print(F("  = ")); Serial.print(wheelRev * mmPerWheelRev(), 1); Serial.print(F(" mm"));
  Serial.print(F("   ใช้เวลา ")); Serial.print(dt); Serial.print(F(" ms"));
  if (dt > 0) {
    Serial.print(F("  (")); Serial.print(wheelRev * 60000.0f / (float)dt, 1);
    Serial.print(F(" rpm ล้อ, ")); Serial.print(wheelRev * mmPerWheelRev() / (float)dt, 3);
    Serial.print(F(" m/s)"));
  }
  Serial.println();
  return true;
}

// แปลงตัวเลือกฝั่งเป็นอาร์กิวเมนต์ของ runMove
static bool moveSelected(bool forward) {
  const long steps = (long)(revs * (float)stepsPerWheelRev());
  switch (sel) {
    case SIDE_LEFT:  return runMove(true,  false, forward,  forward,  steps);
    case SIDE_RIGHT: return runMove(false, true,  forward,  forward,  steps);
    case SIDE_BOTH:  return runMove(true,  true,  forward,  forward,  steps);
    case SIDE_SPIN:  return runMove(true,  true,  forward, !forward,  steps);
  }
  return false;
}

static void runContinuous() {
  if (!enabled) { Serial.println(F("!! ยังไม่ได้จ่ายไฟมอเตอร์ กด e ก่อน")); return; }
  Serial.println(F("   หมุนต่อเนื่อง -- พิมพ์อะไรก็ได้แล้ว Enter เพื่อหยุด"));

  const bool useL = (sel != SIDE_RIGHT);
  const bool useR = (sel != SIDE_LEFT);
  if (useL) setDirL(true);
  if (useR) setDirR(sel != SIDE_SPIN);
  delayMicroseconds(DIR_SETUP_US);

  float v = startRate;
  const float stepUp = (rate - startRate) / (float)(rampSteps > 0 ? rampSteps : 1);
  for (;;) {
    if (Serial.available()) { while (Serial.available()) Serial.read(); break; }
    if (v < rate) { v += stepUp; if (v > rate) v = rate; }
    unsigned long period = (unsigned long)(1000000.0f / v);
    if (period <= PULSE_US) period = PULSE_US + 1;
    pulseHigh(useL, useR);
    delayMicroseconds(period - PULSE_US);
  }
  Serial.println(F("   หยุดแล้ว"));
}

// ==========================================================================
// 6. ชุดทดสอบอัตโนมัติ -- ไล่ทีละฝั่งตามลำดับที่ควรตรวจ
// ==========================================================================
static void selfTestSide(uint8_t side) {
  const bool useL = (side == SIDE_LEFT);
  const bool useR = (side == SIDE_RIGHT);
  const long oneRev = stepsPerWheelRev();

  Serial.println();
  Serial.print(F("=========== ทดสอบฝั่ง ")); Serial.println(SIDE_NAME[side]);

  Serial.println(F("[1] ตัดไฟมอเตอร์ -- หมุนล้อด้วยมือ ต้องหมุนได้ฟรีทั้ง 2 ล้อ"));
  setEnable(false);
  delay(3000);

  Serial.println(F("[2] จ่ายไฟมอเตอร์ -- หมุนล้อด้วยมือ ต้องล็อกแน่นทั้ง 2 ล้อ"));
  Serial.println(F("    ยังหมุนฟรีทั้งคู่  = ขั้ว ENA กลับ ให้แก้ ENABLE_ACTIVE_LOW เป็น 1"));
  Serial.println(F("    ล็อกแค่ล้อเดียว    = ล้ออีกข้างสายหลุด หรือมอเตอร์ต่อขนานไม่ครบเฟส"));
  setEnable(true);
  delay(4000);

  Serial.println(F("[3] เดินหน้า 1 รอบล้อ -- ล้อทั้ง 2 ของฝั่งนี้ต้องหมุนไปทางเดียวกัน"));
  Serial.println(F("    ถ้าหมุนสวนกัน = สายเฟสของมอเตอร์ตัวหนึ่งสลับขั้ว (สลับ A+/A- ตัวนั้น)"));
  if (!runMove(useL, useR, true, true, oneRev)) { Serial.println(F("    ยกเลิก")); return; }
  delay(1200);

  Serial.println(F("[4] ถอยหลัง 1 รอบล้อ -- ต้องกลับมาที่เดิมพอดี (ทำจุดมาร์คไว้ที่ล้อ)"));
  Serial.println(F("    กลับมาไม่ตรง = สเต็ปหลุด ให้ลด rate เพิ่ม ramp หรือเพิ่มกระแสที่ DIP"));
  if (!runMove(useL, useR, false, false, oneRev)) { Serial.println(F("    ยกเลิก")); return; }
  delay(1200);

  Serial.println(F("[5] ทดสอบภาระ -- เดินหน้า 3 รอบที่ความเร็วที่ตั้งไว้"));
  Serial.println(F("    เอามือหน่วงล้อเบา ๆ ดูว่าสเต็ปหลุดง่ายไหม (จะได้ยินเสียงกึก ๆ)"));
  runMove(useL, useR, true, true, oneRev * 3);

  setEnable(false);
  Serial.print(F("=========== จบฝั่ง ")); Serial.print(SIDE_NAME[side]);
  Serial.println(F(" ตัดไฟมอเตอร์แล้ว"));
  Serial.println();
}

// ตรวจว่าสองฝั่งเดินไปทางเดียวกันจริงไหม -- ข้อที่พลาดกันมากที่สุดของ diff drive
static void selfTestPair() {
  Serial.println();
  Serial.println(F("=========== ตรวจทิศของสองฝั่ง"));
  setEnable(true);
  Serial.println(F("[1] สั่งเดินหน้าพร้อมกัน 2 รอบล้อ"));
  Serial.println(F("    ล้อซ้ายกับขวาต้องพาหุ่นไปข้างหน้าทั้งคู่"));
  Serial.println(F("    ถ้าหุ่นหมุนอยู่กับที่แทน = ฝั่งใดฝั่งหนึ่งทิศกลับ"));
  Serial.println(F("    ให้แก้ DIR_INVERT_R (หรือ _L) ในไฟล์นี้ แล้วอัปโหลดใหม่"));
  if (!runMove(true, true, true, true, stepsPerWheelRev() * 2)) { setEnable(false); return; }
  delay(1500);

  Serial.println(F("[2] สั่งหมุนอยู่กับที่ 1 รอบล้อ (ซ้ายเดินหน้า ขวาถอยหลัง)"));
  Serial.println(F("    หุ่นต้องหมุนรอบตัวเอง ไม่ใช่เดินหน้า"));
  runMove(true, true, true, false, stepsPerWheelRev());

  setEnable(false);
  Serial.println(F("=========== จบ ตัดไฟมอเตอร์แล้ว"));
  Serial.println();
}

// ==========================================================================
// 7. เมนู
// ==========================================================================
static void help() {
  Serial.println();
  Serial.println(F("---------------- คำสั่ง ----------------"));
  Serial.println(F("  1 / 2     เลือกฝั่งซ้าย / ฝั่งขวา"));
  Serial.println(F("  3 / 4     สองฝั่งพร้อมกัน / หมุนอยู่กับที่"));
  Serial.println(F("  e         สลับจ่าย/ตัดไฟเลี้ยงมอเตอร์"));
  Serial.println(F("  f / b     เดินหน้า / ถอยหลัง ตามจำนวนรอบที่ตั้งไว้"));
  Serial.println(F("  g         หมุนต่อเนื่องจนกว่าจะพิมพ์อะไรแล้ว Enter"));
  Serial.println(F("  n <รอบ>   ตั้งจำนวนรอบล้อต่อครั้ง (ใส่ทศนิยมได้ เช่น n 0.5)"));
  Serial.println(F("  v <hz>    ตั้งความเร็วปลาย เป็น step/s"));
  Serial.println(F("  w <rpm>   ตั้งความเร็วปลาย เป็น rpm ของล้อ (สะดวกกว่า)"));
  Serial.println(F("  j <hz>    ตั้งความเร็วตอนออกตัว"));
  Serial.println(F("  p <step>  ตั้งระยะ ramp"));
  Serial.println(F("  t         ชุดทดสอบฝั่งที่เลือกอยู่ (เริ่มจากอันนี้)"));
  Serial.println(F("  y         ตรวจทิศของสองฝั่งเทียบกัน"));
  Serial.println(F("  c         ล้างตัวนับพัลส์"));
  Serial.println(F("  ?         แสดงเมนูนี้"));
  Serial.println(F("---------------------------------------"));
}

static void status() {
  const long spr = stepsPerWheelRev();
  Serial.print(F("[ฝั่ง ")); Serial.print(SIDE_NAME[sel]); Serial.print(F("]"));
  Serial.print(F("  ไฟมอเตอร์:")); Serial.print(enabled ? F("จ่าย") : F("ตัด"));
  Serial.print(F("  ")); Serial.print(revs, 2); Serial.print(F(" รอบ/ครั้ง"));
  Serial.print(F("  ")); Serial.print((long)rate); Serial.print(F(" step/s"));
  Serial.print(F(" = ")); Serial.print(rate * 60.0f / (float)spr, 1); Serial.print(F(" rpm"));
  Serial.print(F(" = ")); Serial.print(rate / (float)spr * mmPerWheelRev() / 1000.0f, 3);
  Serial.println(F(" m/s"));
  Serial.print(F("   พัลส์สะสม  ซ้าย ")); Serial.print(pulseL);
  Serial.print(F(" (")); Serial.print((float)pulseL / (float)spr, 2); Serial.print(F(" รอบ)"));
  Serial.print(F("   ขวา ")); Serial.print(pulseR);
  Serial.print(F(" (")); Serial.print((float)pulseR / (float)spr, 2); Serial.println(F(" รอบ)"));
}

static void handleLine(char *line) {
  char *save = nullptr;
  char *cmd  = strtok_r(line, " \t", &save);
  if (!cmd) return;
  char *arg  = strtok_r(nullptr, " \t", &save);
  const float fv = arg ? (float)atof(arg) : 0.0f;
  const long  spr = stepsPerWheelRev();

  switch (cmd[0]) {
    case '1': sel = SIDE_LEFT;  break;
    case '2': sel = SIDE_RIGHT; break;
    case '3': sel = SIDE_BOTH;  break;
    case '4': sel = SIDE_SPIN;  break;
    case 'e': setEnable(!enabled); break;
    case 'f': moveSelected(true);  break;
    case 'b': moveSelected(false); break;
    case 'g': runContinuous(); break;
    case 'n': if (arg && fv > 0.0f) revs = fv; break;
    case 'v': if (arg && fv > 0.0f) rate = fv; break;
    case 'w': if (arg && fv > 0.0f) rate = fv * (float)spr / 60.0f; break;
    case 'j': if (arg && fv > 0.0f) startRate = fv; break;
    case 'p': if (arg && fv >= 0.0f) rampSteps = (long)fv; break;
    case 'c': pulseL = pulseR = 0; break;
    case 't':
      if (sel == SIDE_LEFT || sel == SIDE_RIGHT) selfTestSide(sel);
      else Serial.println(F("!! ชุดทดสอบนี้ต้องเลือกฝั่งเดียวก่อน (กด 1 หรือ 2)"));
      break;
    case 'y': selfTestPair(); break;
    case '?': help(); break;
    default:  Serial.println(F("!! ไม่รู้จักคำสั่งนี้ กด ? ดูเมนู")); return;
  }
  status();
}

// ==========================================================================
// 8. setup / loop
// ==========================================================================
void setup() {
  Serial.begin(115200);

  pinMode(PIN_ENABLE, OUTPUT);
  pinMode(PIN_STEP_L, OUTPUT);
  pinMode(PIN_STEP_R, OUTPUT);
  pinMode(PIN_DIR_L, OUTPUT);
  pinMode(PIN_DIR_R, OUTPUT);
  digitalWrite(PIN_STEP_L, STEP_ACTIVE_LOW ? HIGH : LOW);
  digitalWrite(PIN_STEP_R, STEP_ACTIVE_LOW ? HIGH : LOW);
  setDirL(true);
  setDirR(true);
  setEnable(false);

  delay(300);
  Serial.println();
  Serial.println(F("=============================================="));
  Serial.println(F(" test_wheels_only -- ทดสอบล้อขับ 2 ฝั่ง DM556"));
  Serial.println(F("=============================================="));
  Serial.print(F(" step ต่อรอบล้อ : ")); Serial.println(stepsPerWheelRev());
  Serial.print(F(" ระยะต่อรอบล้อ : ")); Serial.print(mmPerWheelRev(), 1);
  Serial.println(F(" mm"));
  Serial.println(F(" ** ยกล้อให้ลอยพ้นพื้นก่อนทดสอบ **"));
  Serial.println(F(" ตั้ง Serial Monitor ให้ส่งท้ายบรรทัดเป็น Newline"));
  Serial.println(F(" เริ่มจากกด 1 แล้ว t เพื่อทดสอบฝั่งซ้าย"));
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
