/* us_test -- เทสอัลตราโซนิก HC-SR04 2 ตัวแบบง่ายที่สุด (pulseIn) ขาเดียวกับเฟิร์มแวร์ฐานขับ
 *   หน้า: Trig 16 / Echo 34   หลัง: Trig 17 / Echo 35   (Echo ผ่านตัวแบ่ง 1k/2k)
 *   พิมพ์ "T <หน้า mm> <หลัง mm>" 5 ครั้ง/วิ  -1 = ไม่มีเสียงสะท้อน
 * 2026-09-25 เฟิร์มแวร์ฐานขับได้ -1 ตลอด ผู้ใช้บอกเซนเซอร์เดี่ยว ๆ วัดได้ -> แยกว่าเป็นที่โค้ดหรือสาย
 * !! เทสเสร็จต้องแฟลช esp32_mecanum_base กลับก่อนใช้ ROS !! */
const int TRIG[2] = {5, 21};
const int ECHO[2] = {18, 19};

long readMm(int k) {
  digitalWrite(TRIG[k], LOW);  delayMicroseconds(4);
  digitalWrite(TRIG[k], HIGH); delayMicroseconds(10);
  digitalWrite(TRIG[k], LOW);
  unsigned long us = pulseIn(ECHO[k], HIGH, 25000UL);
  return us ? (long)(us * 343UL / 2000UL) : -1;
}

void setup() {
  Serial.begin(115200);
  for (int k = 0; k < 2; k++) { pinMode(TRIG[k], OUTPUT); digitalWrite(TRIG[k], LOW); pinMode(ECHO[k], INPUT); }
  Serial.println("# us_test ready");
}

// โหมดเช็คสาย: ส่ง 'D' ทาง serial -> Trig ค้าง HIGH/LOW สลับทุก 2 วิ (วัดด้วยมิเตอร์ที่ขา Trig ของเซนเซอร์ได้) 'N' = กลับโหมดวัดปกติ
bool diag = false;
void loop() {
  if (Serial.available()) { char c = Serial.read(); if (c == 'D') diag = true; if (c == 'N') diag = false; }
  if (diag) {
    static bool lvl = false; lvl = !lvl;
    for (int k = 0; k < 2; k++) digitalWrite(TRIG[k], lvl);
    Serial.printf("D trig=%s (Trig pin should read %s)  echoLevel %d %d\n", lvl ? "HIGH" : "LOW",
                  lvl ? "~3.3V" : "0V", digitalRead(ECHO[0]), digitalRead(ECHO[1]));
    delay(2000);
    return;
  }
  long f = readMm(0); delay(30);
  long r = readMm(1); delay(30);
  Serial.printf("T %ld %ld  echoLevel %d %d\n", f, r, digitalRead(ECHO[0]), digitalRead(ECHO[1]));
  delay(140);
}
