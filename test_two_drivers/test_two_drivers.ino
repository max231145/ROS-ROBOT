/* ==========================================================================
 * test_two_drivers -- DM556 สองตัว (ซ้าย 1 ตัว ขวา 1 ตัว) ไล่ทีละข้าง ไม่มี ROS
 * --------------------------------------------------------------------------
 * ต่อจากโค้ด moveBoth เดิม แต่เปลี่ยนมาใช้ "ขาชุดเดียวกับเฟิร์มแวร์ ROS"
 * (esp32_mecanum_base ช่อง 0 = ซ้าย, ช่อง 1 = ขวา) เทสผ่านแล้วไม่ต้องย้ายสายอีก
 *
 *   ซ้าย : PUL- GPIO 25   DIR- GPIO 26
 *   ขวา  : PUL- GPIO 27   DIR- GPIO 14
 *   ENA- : GPIO 23  ต่อขนานทั้งสองตัว   (PUL+/DIR+/ENA+ ทุกขารวมเข้า 3V3)
 *
 * ลำดับการเทส (ทำครั้งเดียวแล้วตัดไฟไดรเวอร์ ไม่วนซ้ำ):
 *   1 ล็อกล้อ 10 s -> 2 ซ้ายอย่างเดียว -> 3 ขวาอย่างเดียว -> 4 เดินหน้า -> 5 หมุนซ้าย
 *
 * บอร์ด : ESP32 Dev Module,  Serial Monitor 115200
 *
 * !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 *   ยกล้อให้ลอยพ้นพื้นก่อน  เริ่มหมุนเองหลังเปิดเครื่อง 5 วินาที
 * !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 *
 * เทสเสร็จแล้วต้องแฟลช esp32_mecanum_base กลับก่อนใช้กับ ROS
 * ========================================================================== */

#include <Arduino.h>

// ==========================================================================
// 1. ขา GPIO -- ต่อ common-anode (PUL+/DIR+/ENA+ รวมเข้า 3V3, GPIO ไปเข้าขา -)
// ==========================================================================
#define PIN_STEP_L   25
#define PIN_DIR_L    26
#define PIN_STEP_R   27
#define PIN_DIR_R    14
#define PIN_ENABLE   23          // ENA ของทั้งสองตัวต่อขนานกัน
#define PIN_LED       2          // LED บนบอร์ด -- กะพริบบอกเลขขั้น (ดูได้ตอนไม่มีจอ serial)

#define STEP_ACTIVE_LOW     1    // common-anode: พัลส์เป็นช่วง LOW ขาพักไว้ HIGH
#define ENABLE_ACTIVE_LOW   0    // ดึง HIGH = มอเตอร์มีแรงบิด (ถ้าล้อหมุนฟรีให้เป็น 1)

// ทิศ "เดินหน้า" ของทั้งสองข้างใช้ระดับเดียวกัน -- ตรงกับ DIR_FORWARD_LEVEL ของเฟิร์มแวร์ ROS
// (ชุดสายเดิมสลับ A+/A- ของมอเตอร์ฝั่งขวาไว้แล้ว เพราะมอเตอร์ขวาติดกลับด้าน)
// ห้ามแก้ค่านี้ให้ต่างจากเฟิร์มแวร์ ไม่งั้นเทสผ่านแล้วพอเข้า ROS ทิศจะกลับ
// ถ้าสั่งเดินหน้าแล้วข้างขวาหมุนถอย เลือกทางใดทางหนึ่ง:
//   ก) สลับ A+ กับ A- ของมอเตอร์ขวาที่ขั้วไดรเวอร์  (แนะนำ ไม่ต้องแก้ซอฟต์แวร์)
//   ข) ปล่อยสายไว้ แล้วตั้ง invert_wheels: [false, true, false, true] ใน mecanum_base.yaml
#define DIR_FORWARD_LEVEL  LOW

// ==========================================================================
// 2. ความเร็ว -- ช้ากว่าโค้ดเดิมเพื่อให้ดูทิศหมุนด้วยตาได้ชัด
// ==========================================================================
#define STEPS_PER_REV   1600     // ต้องตรงกับ DIP microstep ของ DM556 ทั้งสองตัว
#define PULSE_US          15     // ความกว้างพัลส์
#define SPEED_DELAY     1200     // คาบปลาย (us) ~820 step/s = ครึ่งรอบ/s
#define START_DELAY     3000     // คาบตอนออกตัวและก่อนหยุด (ค่ามาก = ช้า)
#define RAMP_STEPS       400     // จำนวนสเต็ปที่ใช้เร่งและใช้เบรก
#define PHASE_STEPS     4000     // ต่อขั้น ~5 s = 2.5 รอบล้อ
#define LOCK_SECONDS      10

#define STEP_IDLE_LEVEL (STEP_ACTIVE_LOW ? HIGH : LOW)
#define STEP_PULSE_LEVEL (STEP_ACTIVE_LOW ? LOW : HIGH)

/* กะพริบ n ครั้งช้า ๆ บอกว่ากำลังจะเข้าขั้นที่ n  แล้วค้างดับ 1 วิก่อนเริ่มจริง */
static void blinkPhase(int n) {
  for (int i = 0; i < n; i++) {
    digitalWrite(PIN_LED, HIGH); delay(350);
    digitalWrite(PIN_LED, LOW);  delay(350);
  }
  delay(1000);
}

static void driverEnable(bool on) {
  digitalWrite(PIN_ENABLE, ENABLE_ACTIVE_LOW ? (on ? LOW : HIGH) : (on ? HIGH : LOW));
}

static void countdown() {
  for (int n = 3; n >= 1; n--) {
    Serial.print(F("   เริ่มใน "));
    Serial.println(n);
    delay(1000);
  }
}

/* ยิงพัลส์ให้ข้างที่เลือก useL/useR  fwdL/fwdR = ข้างนั้นหมุนไปหน้าหรือถอย */
static void runPhase(int phaseNo, const char *name, const char *expect,
                     bool useL, bool useR, bool fwdL, bool fwdR, long steps) {
  Serial.print(F("== "));
  Serial.println(name);
  blinkPhase(phaseNo);
  digitalWrite(PIN_LED, HIGH);          // ติดค้าง = กำลังยิงพัลส์
  Serial.print(F("   ต้องเห็น: "));
  Serial.println(expect);
  countdown();

  digitalWrite(PIN_DIR_L, fwdL ? DIR_FORWARD_LEVEL : !DIR_FORWARD_LEVEL);
  digitalWrite(PIN_DIR_R, fwdR ? DIR_FORWARD_LEVEL : !DIR_FORWARD_LEVEL);
  delay(5);                       // รอให้ DIR นิ่งก่อนส่งพัลส์

  for (long i = 0; i < steps; i++) {
    const long edge = (i < steps - 1 - i) ? i : (steps - 1 - i);
    uint32_t d = SPEED_DELAY;
    if (edge < RAMP_STEPS) {
      d = START_DELAY - (uint32_t)(START_DELAY - SPEED_DELAY) * edge / RAMP_STEPS;
    }
    if (useL) digitalWrite(PIN_STEP_L, STEP_PULSE_LEVEL);
    if (useR) digitalWrite(PIN_STEP_R, STEP_PULSE_LEVEL);
    delayMicroseconds(PULSE_US);
    if (useL) digitalWrite(PIN_STEP_L, STEP_IDLE_LEVEL);
    if (useR) digitalWrite(PIN_STEP_R, STEP_IDLE_LEVEL);
    delayMicroseconds(d);
  }
  Serial.print(F("   จบขั้น -- พัลส์ที่ส่ง "));
  Serial.print(steps);
  Serial.print(F(" step = "));
  Serial.print((float)steps / STEPS_PER_REV, 2);
  Serial.println(F(" รอบล้อ"));
  digitalWrite(PIN_LED, LOW);           // ดับ = จบขั้น พักให้จดผล
  delay(3000);
}

void setup() {
  Serial.begin(115200);
  delay(300);

  pinMode(PIN_ENABLE, OUTPUT);
  driverEnable(false);
  pinMode(PIN_STEP_L, OUTPUT);
  pinMode(PIN_DIR_L, OUTPUT);
  pinMode(PIN_STEP_R, OUTPUT);
  pinMode(PIN_DIR_R, OUTPUT);
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);
  digitalWrite(PIN_STEP_L, STEP_IDLE_LEVEL);
  digitalWrite(PIN_STEP_R, STEP_IDLE_LEVEL);
  digitalWrite(PIN_DIR_L, DIR_FORWARD_LEVEL);
  digitalWrite(PIN_DIR_R, DIR_FORWARD_LEVEL);

  Serial.println(F("\n# test_two_drivers -- ซ้าย 25/26  ขวา 27/14  ENA 23"));
  Serial.println(F("# ยกล้อลอยพ้นพื้นทั้ง 4 ล้อ  เริ่มในอีก 5 วินาที"));
  delay(5000);

  driverEnable(true);
  Serial.println(F("เปิดไดรเวอร์แล้ว"));

  runPhase(1, "ขั้น 1 ล็อกล้อ",
           "หมุนล้อด้วยมือทีละล้อ ต้องฝืนทั้ง 4 ล้อ (ข้างไหนหมุนฟรี = ENA/ไฟ 24V ข้างนั้น)",
           false, false, true, true, 0);
  delay((unsigned long)LOCK_SECONDS * 1000UL);

  runPhase(2, "ขั้น 2 ซ้ายอย่างเดียว (ช่อง 0)",
           "หน้าซ้าย + หลังซ้าย หมุนไปหน้าทั้งคู่ · ข้างขวานิ่ง",
           true, false, true, true, PHASE_STEPS);

  runPhase(3, "ขั้น 3 ขวาอย่างเดียว (ช่อง 1)",
           "หน้าขวา + หลังขวา หมุนไปหน้าทั้งคู่ · ข้างซ้ายนิ่ง",
           false, true, true, true, PHASE_STEPS);

  runPhase(4, "ขั้น 4 เดินหน้า",
           "ทั้ง 4 ล้อหมุนไปหน้า ความเร็วเท่ากัน",
           true, true, true, true, PHASE_STEPS);

  runPhase(5, "ขั้น 5 หมุนซ้าย",
           "ข้างซ้ายหมุนถอย · ข้างขวาหมุนไปหน้า",
           true, true, false, true, PHASE_STEPS);

  driverEnable(false);
  Serial.println(F("ปิดไดรเวอร์แล้ว -- จบการเทส (กด EN ที่บอร์ดเพื่อรันใหม่)"));
  Serial.println(F("ผลไม่ตรง: ขั้น 2/3 สลับข้าง = สลับขั้วต่อ PUL/DIR สองตัว"));
  Serial.println(F("          ข้างใดหมุนถอยทั้งข้าง = สลับ A+/A- ที่ไดรเวอร์ข้างนั้น"));
  Serial.println(F("          ข้างเดียวกันหมุนสวนกัน = สลับ A+/A- ของมอเตอร์ตัวที่ถอย"));
}

void loop() {
  delay(1000);                    // จบแล้วอยู่เฉย ๆ ไม่วนซ้ำ ป้องกันหุ่นวิ่งเอง
}
