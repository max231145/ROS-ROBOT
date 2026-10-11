/*
 * polarity_sweep -- รันในตัว ESP32 เอง ไม่ต้องมี Pi หรือคอมสั่ง
 *
 * ไล่ลองทุกชุดขั้วสัญญาณกับทั้งสองช่องพร้อมกัน ทีละชุด ชุดละ 15 วินาที
 *   ช่องซ้าย = GPIO 25 (STEP) / 26 (DIR)     ช่องขวา = GPIO 27 (STEP) / 14 (DIR)
 * ถ้าชุดไหนล้อหมุน = เจอขั้วที่ถูกต้อง แล้วค่อยแก้เฟิร์มแวร์ ROS ให้ตรง
 *
 *   ชุด 1  ENA=HIGH  พัลส์=LOW    <- ชุดที่เฟิร์มแวร์ ROS ใช้อยู่ตอนนี้
 *   ชุด 2  ENA=HIGH  พัลส์=HIGH
 *   ชุด 3  ENA=LOW   พัลส์=LOW
 *   ชุด 4  ENA=LOW   พัลส์=HIGH
 *
 * ยกล้อลอยพ้นพื้นทั้ง 4 ล้อ  วนซ้ำไปเรื่อย ๆ จนกว่าจะถอดไฟ
 */
#define PIN_STEP_L   25
#define PIN_DIR_L    26
#define PIN_STEP_R   27
#define PIN_DIR_R    14
#define PIN_ENABLE   23

#define RATE_HZ      200      // step/s -- ช้า เห็นการคืบชัด
#define PULSE_US      20      // กว้างกว่าเฟิร์มแวร์ ROS (50us) ไม่ได้ แต่พอสำหรับ DM556
#define PHASE_SEC     15

struct Combo { int enaLevel; int pulseLevel; const char *name; };

static const Combo COMBOS[] = {
  { HIGH, LOW,  "ชุด 1  ENA=HIGH  พัลส์=LOW   (ชุดที่ ROS ใช้อยู่)" },
  { HIGH, HIGH, "ชุด 2  ENA=HIGH  พัลส์=HIGH" },
  { LOW,  LOW,  "ชุด 3  ENA=LOW   พัลส์=LOW" },
  { LOW,  HIGH, "ชุด 4  ENA=LOW   พัลส์=HIGH" },
};
static const int NCOMBO = sizeof(COMBOS) / sizeof(COMBOS[0]);

void setup() {
  Serial.begin(115200);
  delay(600);
  pinMode(PIN_STEP_L, OUTPUT);
  pinMode(PIN_DIR_L,  OUTPUT);
  pinMode(PIN_STEP_R, OUTPUT);
  pinMode(PIN_DIR_R,  OUTPUT);
  pinMode(PIN_ENABLE, OUTPUT);
  digitalWrite(PIN_ENABLE, LOW);
  Serial.println(F("\n# polarity_sweep -- ไล่ขั้วสัญญาณทุกชุดกับช่องซ้าย (GPIO 25/26)"));
  Serial.println(F("# ยกล้อลอยพ้นพื้น  ดูล้อหน้าซ้ายและหลังซ้าย  ชุดไหนหมุนให้จำเลขชุดไว้"));
  Serial.println(F("# เริ่มในอีก 5 วินาที"));
  delay(5000);
}

void loop() {
  for (int c = 0; c < NCOMBO; c++) {
    const Combo &k = COMBOS[c];
    Serial.print(F("\n== ")); Serial.println(k.name);

    digitalWrite(PIN_ENABLE, k.enaLevel);
    digitalWrite(PIN_DIR_L, LOW);
    digitalWrite(PIN_DIR_R, LOW);
    digitalWrite(PIN_STEP_L, !k.pulseLevel);   // พักที่ระดับตรงข้ามพัลส์
    digitalWrite(PIN_STEP_R, !k.pulseLevel);
    delay(300);

    const long steps = (long)RATE_HZ * PHASE_SEC;
    const unsigned long period = 1000000UL / RATE_HZ;
    for (long i = 0; i < steps; i++) {
      digitalWrite(PIN_STEP_L, k.pulseLevel);
      digitalWrite(PIN_STEP_R, k.pulseLevel);
      delayMicroseconds(PULSE_US);
      digitalWrite(PIN_STEP_L, !k.pulseLevel);
      digitalWrite(PIN_STEP_R, !k.pulseLevel);
      delayMicroseconds(period - PULSE_US);
    }
    Serial.print(F("   จบชุดนี้ -- ส่งไป ")); Serial.print(steps);
    Serial.print(F(" step = ")); Serial.print((float)steps / 1600.0f, 2);
    Serial.println(F(" รอบล้อ  (ล้อหมุนไหม?)"));

    digitalWrite(PIN_ENABLE, LOW);
    delay(4000);                    // พักให้จดผลก่อนชุดถัดไป
  }
  Serial.println(F("\n# ครบ 4 ชุดแล้ว วนใหม่อีกรอบ"));
}
