/* ==========================================================================
 * test_one_side -- ทดสอบฝั่งเดียว: DM556 1 ตัว ขับ 2 ล้อ
 * --------------------------------------------------------------------------
 * ตัวเล็กที่สุด ต่อสายฝั่งเดียวก็รันได้ ไม่ต้องรอต่อครบทั้งคู่
 * พออันนี้ผ่านแล้วค่อยไปต่อที่ test_wheels_only (ทดสอบสองฝั่งเทียบกัน)
 *
 * บอร์ด : ESP32 Dev Module,  Serial Monitor 115200
 *
 * กดปุ่มเดียวทำงานเลย ไม่ต้องกด Enter และไม่ต้องตั้ง line ending
 * เปิดเครื่องปุ๊บมันเดินเองเลย ถึงยังไม่ได้เปิด Serial Monitor ก็เห็นล้อหมุน
 *
 * !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 *   ยกล้อให้ลอยพ้นพื้นก่อน  มันเริ่มหมุนเองหลังเปิดเครื่อง 5 วินาที
 * !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 * ========================================================================== */

#include <Arduino.h>

// ==========================================================================
// 1. ขา GPIO -- ต่อ common-anode (PUL+/DIR+/ENA+ รวมเข้า 3V3, GPIO ไปเข้าขา -)
// ==========================================================================
#define PIN_STEP     25
#define PIN_DIR      26
#define PIN_ENABLE   23

#define STEP_ACTIVE_LOW    1   // ต่อ common-anode -> พัลส์เป็นช่วง LOW
#define ENABLE_ACTIVE_LOW  0   // ถ้ากด e แล้วล้อยังหมุนฟรี ให้เปลี่ยนเป็น 1
#define DIR_INVERT         false

// ==========================================================================
// 2. ค่ากลไก -- กรอกให้ตรงของจริง ไม่งั้นระยะที่พิมพ์ออกมาไม่มีความหมาย
// ==========================================================================
#define STEPS_PER_REV     200L     // มอเตอร์ 1.8 องศา = 200
#define MICROSTEP           8L     // ตามที่ตั้ง DIP SW1-SW3 ของ DM556
#define GEAR_RATIO         1.0f    // มอเตอร์หมุนกี่รอบ ต่อล้อ 1 รอบ (ต่อตรง = 1.0)
#define WHEEL_DIAMETER_MM 100.0f

// ==========================================================================
// 3. ค่าที่ปรับได้ระหว่างรัน
// ==========================================================================
#define PULSE_US       5           // DM556 ต้องการอย่างน้อย 2.5 us
#define DIR_SETUP_US  20

static float rate      = 1600.0f;  // step/s ความเร็วปลาย
static float startRate =  300.0f;  // step/s ตอนออกตัว
static long  rampSteps =  800L;    // ramp สั้นไปแล้วมอเตอร์จะครวญอยู่กับที่
static float revs      =    1.0f;  // รอบล้อต่อการสั่งหนึ่งครั้ง

static bool enabled = false;
static bool autoRun = true;        // เดินไป-กลับเองอัตโนมัติ
static long pulseTotal = 0;

static inline long stepsPerWheelRev() {
  return (long)((float)(STEPS_PER_REV * MICROSTEP) * GEAR_RATIO);
}
static inline float mmPerWheelRev() { return 3.14159265f * WHEEL_DIAMETER_MM; }

// ==========================================================================
// 4. ระดับล่างสุด
// ==========================================================================
static void setEnable(bool on) {
  enabled = on;
#if ENABLE_ACTIVE_LOW
  digitalWrite(PIN_ENABLE, on ? LOW : HIGH);
#else
  digitalWrite(PIN_ENABLE, on ? HIGH : LOW);
#endif
  delay(50);
  Serial.print(F(">> ไฟเลี้ยงมอเตอร์: "));
  Serial.println(on ? F("จ่ายแล้ว (ล้อต้องล็อกแน่น)") : F("ตัดแล้ว (ล้อต้องหมุนฟรี)"));
}

static void setDir(bool forward) {
  bool level = forward;
  if (DIR_INVERT) level = !level;
  digitalWrite(PIN_DIR, level ? HIGH : LOW);
  delayMicroseconds(DIR_SETUP_US);
}

static inline void stepPulse() {
#if STEP_ACTIVE_LOW
  digitalWrite(PIN_STEP, LOW);
  delayMicroseconds(PULSE_US);
  digitalWrite(PIN_STEP, HIGH);
#else
  digitalWrite(PIN_STEP, HIGH);
  delayMicroseconds(PULSE_US);
  digitalWrite(PIN_STEP, LOW);
#endif
  pulseTotal++;
}

// ==========================================================================
// 5. เดิน n step พร้อม ramp ขึ้น-ลง  (คืน false = มีคนกดปุ่มขัดจังหวะ)
// ==========================================================================
static bool moveSteps(long n, bool forward) {
  if (!enabled) { Serial.println(F("!! ยังไม่ได้จ่ายไฟมอเตอร์ กด e ก่อน")); return false; }
  if (n <= 0) return true;

  setDir(forward);
  long ramp = rampSteps;
  if (ramp > n / 2) ramp = n / 2;

  const unsigned long t0 = millis();
  for (long i = 0; i < n; i++) {
    if (Serial.available()) {          // ไม่อ่านทิ้ง ปล่อยให้ loop() จัดการต่อ
      Serial.println(F("\n-- หยุดกลางคัน"));
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
    stepPulse();
    delayMicroseconds(period - PULSE_US);
  }

  const unsigned long dt = millis() - t0;
  const float wr = (float)n / (float)stepsPerWheelRev();
  Serial.print(forward ? F("   เดินหน้า ") : F("   ถอยหลัง "));
  Serial.print(wr, 2);    Serial.print(F(" รอบล้อ = "));
  Serial.print(wr * mmPerWheelRev(), 0); Serial.print(F(" mm  ใช้เวลา "));
  Serial.print(dt);       Serial.println(F(" ms"));
  return true;
}

static void showSpeed() {
  const long spr = stepsPerWheelRev();
  Serial.print(F(">> ")); Serial.print((long)rate); Serial.print(F(" step/s = "));
  Serial.print(rate * 60.0f / (float)spr, 1); Serial.print(F(" rpm = "));
  Serial.print(rate / (float)spr * mmPerWheelRev() / 1000.0f, 2); Serial.println(F(" m/s"));
}

// ==========================================================================
// วัดว่า DIP ของ DM556 ตั้ง microstep ไว้เท่าไหร่จริง ๆ
// --------------------------------------------------------------------------
// อาการ "ขยับแค่นิดเดียว" เกิดจากค่านี้ไม่ตรงเป็นส่วนใหญ่
// เช่นถ้าโค้ดคิดว่า 1600 พัลส์/รอบ แต่ DIP ตั้งไว้ 25600 พัลส์/รอบ
// สั่ง "1 รอบ" มันจะหมุนจริงแค่ 1/16 รอบ = 22 องศา แล้วก็ถอยกลับเท่าเดิม
// ==========================================================================
static void measurePulsesPerRev() {
  if (!enabled) setEnable(true);
  autoRun = false;

  Serial.println();
  Serial.println(F("== วัดพัลส์ต่อรอบ =="));
  Serial.println(F("1. ทำจุดมาร์คไว้ที่ล้อ ให้ตรงกับจุดอ้างอิงบนตัวถัง"));
  Serial.println(F("2. กด Enter เพื่อเริ่มหมุนช้า ๆ"));
  Serial.println(F("3. พอมาร์คหมุนกลับมาครบ 1 รอบพอดี ให้พิมพ์อะไรก็ได้"));
  while (!Serial.available()) { delay(10); }
  while (Serial.available()) Serial.read();

  setDir(true);
  const float v = 250.0f;                       // ช้าพอที่จะกดหยุดได้ทัน
  const unsigned long period = (unsigned long)(1000000.0f / v);
  long count = 0;
  unsigned long lastPrint = millis();

  while (!Serial.available()) {
    stepPulse();
    count++;
    delayMicroseconds(period - PULSE_US);
    if (millis() - lastPrint > 1000) {
      lastPrint = millis();
      Serial.print(F("   ")); Serial.print(count); Serial.println(F(" พัลส์..."));
    }
    if (count > 300000L) { Serial.println(F("!! เกิน 300000 พัลส์ หยุดเอง")); break; }
  }
  while (Serial.available()) Serial.read();

  Serial.print(F(">> วัดได้ ")); Serial.print(count); Serial.println(F(" พัลส์ต่อ 1 รอบล้อ"));
  Serial.print(F(">> ตอนนี้โค้ดคิดว่า ")); Serial.print(stepsPerWheelRev());
  Serial.println(F(" พัลส์ต่อรอบ"));

  const float ratio = (float)count / (float)stepsPerWheelRev();
  if (ratio > 1.2f || ratio < 0.83f) {
    Serial.print(F("!! ไม่ตรงกัน ต่างกัน ")); Serial.print(ratio, 2); Serial.println(F(" เท่า"));
    Serial.print(F("   ให้แก้ MICROSTEP ในไฟล์นี้เป็น "));
    Serial.println((long)((float)count / (float)STEPS_PER_REV / GEAR_RATIO));
    Serial.println(F("   (หรือไปตั้ง DIP SW1-SW3 ของ DM556 ให้ตรงกับที่โค้ดคิดแทนก็ได้)"));
  } else {
    Serial.println(F(">> ตรงกันแล้ว ปัญหาอยู่ที่อื่น -- ลองกด 0 ทดสอบแรงบิดที่ความเร็วต่ำ"));
  }
  Serial.println();
}

// ทดสอบแรงบิดล้วน ๆ: ช้ามากและไม่มี ramp ถ้าอันนี้ยังหมุนไม่ได้ = ปัญหาไฟ/กระแส/สายเฟส
static void slowMode() {
  rate = 300.0f;
  startRate = 300.0f;
  rampSteps = 0;
  Serial.println(F(">> โหมดช้า: 300 step/s ไม่มี ramp"));
  Serial.println(F("   ถ้าโหมดนี้หมุนได้เรียบ แต่เร็วกว่านี้แล้วครวญ = กระแสที่ DIP ต่ำไป"));
  Serial.println(F("   หรือมอเตอร์ 2 ตัวต่อขนานกันแล้วกระแสถูกแบ่งครึ่ง"));
  showSpeed();
}

static void help() {
  Serial.println();
  Serial.println(F("------- กดปุ่มเดียว ไม่ต้องกด Enter -------"));
  Serial.println(F("  space  เริ่ม/หยุดเดินไป-กลับอัตโนมัติ"));
  Serial.println(F("  e      สลับจ่าย/ตัดไฟเลี้ยงมอเตอร์"));
  Serial.println(F("  f      เดินหน้า 1 ครั้ง"));
  Serial.println(F("  b      ถอยหลัง 1 ครั้ง"));
  Serial.println(F("  1      ยิงพัลส์เดียว (ดู 1 microstep)"));
  Serial.println(F("  m      วัดว่า DIP ตั้ง microstep ไว้เท่าไหร่จริง (แก้อาการขยับนิดเดียว)"));
  Serial.println(F("  0      โหมดช้าไม่มี ramp (ทดสอบแรงบิดล้วน ๆ)"));
  Serial.println(F("  +  -   เพิ่ม/ลดความเร็ว"));
  Serial.println(F("  [  ]   ลด/เพิ่มจำนวนรอบต่อครั้ง"));
  Serial.println(F("  c      ล้างตัวนับพัลส์"));
  Serial.println(F("  ?      เมนูนี้"));
  Serial.println(F("-------------------------------------------"));
  Serial.print(F("  รอบล้อละ ")); Serial.print(stepsPerWheelRev());
  Serial.print(F(" step = ")); Serial.print(mmPerWheelRev(), 1); Serial.println(F(" mm"));
  showSpeed();
}

// ==========================================================================
// 6. setup / loop
// ==========================================================================
void setup() {
  Serial.begin(115200);
  pinMode(PIN_ENABLE, OUTPUT);
  pinMode(PIN_STEP, OUTPUT);
  pinMode(PIN_DIR, OUTPUT);
  digitalWrite(PIN_STEP, STEP_ACTIVE_LOW ? HIGH : LOW);
  digitalWrite(PIN_DIR, LOW);
  setEnable(false);

  delay(300);
  Serial.println();
  Serial.println(F("========================================"));
  Serial.println(F(" test_one_side -- DM556 1 ตัว ขับ 2 ล้อ"));
  Serial.println(F("========================================"));
  help();

  Serial.println(F("\n** ยกล้อให้ลอยพ้นพื้น จะเริ่มเดินเองใน..."));
  for (int i = 5; i > 0; i--) {
    Serial.print(i); Serial.print(F(".. "));
    delay(1000);
    if (Serial.available()) { autoRun = false; Serial.println(F("\nยกเลิก รอคำสั่ง")); break; }
  }
  Serial.println();
  if (autoRun) setEnable(true);
}

void loop() {
  // ---- คำสั่งจากคีย์บอร์ด (ทีละตัวอักษร ไม่ต้องรอ newline) ----
  while (Serial.available()) {
    const char c = (char)Serial.read();
    switch (c) {
      case ' ':
        autoRun = !autoRun;
        Serial.println(autoRun ? F(">> เดินอัตโนมัติ") : F(">> หยุดอัตโนมัติ"));
        if (autoRun && !enabled) setEnable(true);
        break;
      case 'e': setEnable(!enabled); break;
      case 'f': autoRun = false; moveSteps((long)(revs * stepsPerWheelRev()), true);  break;
      case 'b': autoRun = false; moveSteps((long)(revs * stepsPerWheelRev()), false); break;
      case '1':
        autoRun = false;
        if (!enabled) { Serial.println(F("!! กด e ก่อน")); break; }
        setDir(true); stepPulse();
        Serial.println(F(">> ยิง 1 พัลส์"));
        break;
      case '+': case '=':
        rate *= 1.25f; if (rate > 10000.0f) rate = 10000.0f; showSpeed(); break;
      case '-': case '_':
        rate /= 1.25f; if (rate < 50.0f) rate = 50.0f; showSpeed(); break;
      case ']':
        revs *= 2.0f; if (revs > 64.0f) revs = 64.0f;
        Serial.print(F(">> ")); Serial.print(revs, 2); Serial.println(F(" รอบ/ครั้ง")); break;
      case '[':
        revs /= 2.0f; if (revs < 0.0625f) revs = 0.0625f;
        Serial.print(F(">> ")); Serial.print(revs, 2); Serial.println(F(" รอบ/ครั้ง")); break;
      case 'm': measurePulsesPerRev(); break;
      case '0': autoRun = false; slowMode(); break;
      case 'c':
        pulseTotal = 0; Serial.println(F(">> ล้างตัวนับแล้ว")); break;
      case '?': help(); break;
      case '\r': case '\n': break;
      default:
        Serial.println(F("!! กด ? ดูเมนู"));
        break;
    }
  }

  // ---- เดินไป-กลับอัตโนมัติ ----
  if (autoRun) {
    const long n = (long)(revs * stepsPerWheelRev());
    if (!moveSteps(n, true)) return;
    delay(700);
    if (!moveSteps(n, false)) return;
    delay(700);
    Serial.print(F("   พัลส์สะสม ")); Serial.print(pulseTotal);
    Serial.print(F(" -- ถ้าล้อไม่กลับมาที่จุดมาร์คเดิม = สเต็ปหลุด "));
    Serial.println(F("ให้กด - ลดความเร็ว หรือเพิ่มกระแสที่ DIP"));
  }
}
