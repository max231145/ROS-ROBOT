/* ==========================================================================
 * test_one_driver_4motors -- DM556 ตัวเดียว ขับมอเตอร์ 4 ตัวพร้อมกัน
 * --------------------------------------------------------------------------
 * ดัดแปลงจากโค้ดที่เขียนเองมา (moveBoth) ให้เหลือไดรเวอร์ตัวเดียว
 *
 * ข้อจำกัดของการใช้ไดรเวอร์ตัวเดียว:
 *   - มอเตอร์ทั้ง 4 ได้พัลส์และทิศเดียวกันเสมอ -> หุ่นเดินหน้า/ถอยหลังได้อย่างเดียว เลี้ยวไม่ได้
 *   - มอเตอร์ฝั่งขวาติดกลับด้าน สั่งกลับทิศในโค้ดไม่ได้แล้วเพราะ DIR เส้นเดียว
 *     ต้องสลับสาย A+ กับ A- ของมอเตอร์ฝั่งขวาทั้ง 2 ตัวแทน
 *
 * บอร์ด : ESP32 Dev Module,  Serial Monitor 115200
 * !! ยกล้อให้ลอยพ้นพื้นก่อน มันเริ่มหมุนเองหลังเปิดเครื่อง 2 วินาที !!
 * ========================================================================== */

#include <Arduino.h>

#define PIN_STEP     25
#define PIN_DIR      26
#define PIN_ENABLE   23

#define STEPS_PER_REV  1600      // ต้องตรงกับ DIP SW1-SW3 ของ DM556
#define PULSE_US       15        // ความกว้างพัลส์
#define MAX_SPS        1600.0f   // step/s ความเร็วสูงสุด
#define START_SPS      300.0f    // step/s ตอนออกตัวและก่อนหยุด
#define RAMP_STEPS     400       // จำนวนสเต็ปที่ใช้เร่ง และใช้เบรก

// ENA: ถ้าจ่ายไฟแล้วล้อยังหมุนฟรีด้วยมือ ให้สลับ HIGH กับ LOW ในบรรทัดนี้
#define ENABLE_LEVEL   HIGH

void move(bool forward, long steps) {
  digitalWrite(PIN_DIR, forward ? LOW : HIGH);
  delay(5);                      // รอให้ DIR นิ่งก่อนส่งพัลส์

  const unsigned long t0 = millis();
  for (long i = 0; i < steps; i++) {
    const long edge = (i < steps - 1 - i) ? i : (steps - 1 - i);

    // ramp แบบความเร่งคงที่: v = sqrt(v0^2 + 2as)
    // ของเดิมไล่ "ค่าหน่วง" แบบเชิงเส้น ทำให้ความเร็วพุ่งแรงช่วงท้าย ramp
    // (ความเร่งตรงนั้นสูงกว่าช่วงต้นราว 100 เท่า ~24000 step/s^2) ซึ่งเป็นจุดที่สเต็ปปิงหลุดง่ายที่สุด
    float v = MAX_SPS;
    if (edge < RAMP_STEPS) {
      v = sqrtf(START_SPS * START_SPS +
                (MAX_SPS * MAX_SPS - START_SPS * START_SPS) * (float)edge / (float)RAMP_STEPS);
    }
    const uint32_t period = (uint32_t)(1000000.0f / v);

    digitalWrite(PIN_STEP, LOW);
    delayMicroseconds(PULSE_US);
    digitalWrite(PIN_STEP, HIGH);
    delayMicroseconds(period > PULSE_US ? period - PULSE_US : 1);
  }

  Serial.print(forward ? "เดินหน้า " : "ถอยหลัง ");
  Serial.print(steps);
  Serial.print(" step  ใช้เวลา ");
  Serial.print(millis() - t0);
  Serial.println(" ms");
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_ENABLE, OUTPUT);
  pinMode(PIN_STEP, OUTPUT);
  pinMode(PIN_DIR, OUTPUT);

  // ตั้ง STEP ให้อยู่สถานะพักก่อน แล้วค่อยเปิดไดรเวอร์เป็นขั้นสุดท้าย
  digitalWrite(PIN_STEP, HIGH);
  delay(10);
  digitalWrite(PIN_ENABLE, ENABLE_LEVEL);

  Serial.println("test_one_driver_4motors -- ยกล้อให้ลอยก่อน");
  delay(2000);
}

void loop() {
  move(true, STEPS_PER_REV);     // เดินหน้า 1 รอบ
  delay(1000);
  move(false, STEPS_PER_REV);    // ถอยหลัง 1 รอบ
  delay(1000);
}
