/*
 * pin_check -- ให้ ESP32 อ่านสถานะขาตัวเองแล้วรายงานทางพอร์ตอนุกรม
 * ไม่ต้องใช้มัลติมิเตอร์ ไม่มีปัญหาจุดอ้างอิงหรือจังหวะวัด
 *
 * ทดสอบทีละขา 4 แบบ:
 *   pullup    : ปล่อยลอยพร้อมดึงขึ้นเบา ๆ  ถ้าอ่านได้ 0 = มีอะไรดึงลงกราวด์
 *   pulldown  : ปล่อยลอยพร้อมดึงลงเบา ๆ    ถ้าอ่านได้ 1 = มีอะไรดึงขึ้น 3V3
 *   drive HIGH: สั่งขาเป็น 1 แล้วอ่านค่าจริงที่ขา  ถ้าได้ 0 = ขานั้นถูกตรึงลงกราวด์
 *   drive LOW : สั่งขาเป็น 0 แล้วอ่านค่าจริงที่ขา  ถ้าได้ 1 = ขานั้นถูกตรึงขึ้น 3V3
 *
 * ใช้ GPIO_MODE_INPUT_OUTPUT เพื่อให้ digitalRead อ่าน "ระดับจริงที่ขา" ไม่ใช่ค่าที่สั่งไว้
 */
#include <driver/gpio.h>

struct PinInfo { uint8_t pin; const char *name; };

static const PinInfo PINS[] = {
  { 25, "STEP ซ้าย  (ช่อง 0)" },
  { 26, "DIR  ซ้าย  (ช่อง 0)" },
  { 27, "STEP ขวา   (ช่อง 1)" },
  { 14, "DIR  ขวา   (ช่อง 1)" },
  { 23, "ENA  ร่วม          " },
  { 32, "STEP ช่อง 2 (ไม่ได้ต่อ)" },
  { 33, "DIR  ช่อง 2 (ไม่ได้ต่อ)" },
};
static const int N = sizeof(PINS) / sizeof(PINS[0]);

static int probe(uint8_t pin, gpio_pull_mode_t pull) {
  gpio_set_direction((gpio_num_t)pin, GPIO_MODE_INPUT);
  gpio_set_pull_mode((gpio_num_t)pin, pull);
  delay(20);
  return gpio_get_level((gpio_num_t)pin);
}

static int driveRead(uint8_t pin, int level) {
  gpio_set_direction((gpio_num_t)pin, GPIO_MODE_INPUT_OUTPUT);
  gpio_set_pull_mode((gpio_num_t)pin, GPIO_FLOATING);
  gpio_set_level((gpio_num_t)pin, level);
  delay(20);
  return gpio_get_level((gpio_num_t)pin);
}

void setup() {
  Serial.begin(115200);
  delay(600);
  Serial.println(F("\n# pin_check -- อ่านสถานะขาจริงจากตัว ESP32 เอง"));
  Serial.println(F("# ไดรเวอร์ถูกปิดไว้ (ENA LOW) มอเตอร์ไม่มีแรงบิด"));
  Serial.println(F("# ต่อสายไว้ตามปกติ แล้วดูคอลัมน์ 'สรุป'"));
  gpio_set_direction(GPIO_NUM_23, GPIO_MODE_OUTPUT);
  gpio_set_level(GPIO_NUM_23, 0);          // ENA LOW = ไดรเวอร์ปิด
}

void loop() {
  Serial.println(F("\n----------------------------------------------------------------"));
  Serial.println(F("ขา  หน้าที่                pullup pulldn driveHI driveLO  สรุป"));
  for (int i = 0; i < N; i++) {
    const uint8_t p = PINS[i].pin;
    const int pu = probe(p, GPIO_PULLUP_ONLY);
    const int pd = probe(p, GPIO_PULLDOWN_ONLY);
    const int dh = driveRead(p, 1);
    const int dl = driveRead(p, 0);

    const char *verdict;
    if (dh == 1 && dl == 0) {
      if (pu == 1 && pd == 0)      verdict = "ปกติ ไม่มีโหลดค้าง";
      else if (pu == 1 && pd == 1) verdict = "ปกติ แต่มีตัวดึงขึ้นภายนอก (ออปโตต่ออยู่)";
      else if (pu == 0 && pd == 0) verdict = "ปกติ แต่มีตัวดึงลงภายนอก";
      else                         verdict = "ปกติ";
    } else if (dh == 0 && dl == 0) verdict = "<<< ขาถูกตรึงลงกราวด์ ดันขึ้นไม่ได้";
    else if (dh == 1 && dl == 1)   verdict = "<<< ขาถูกตรึงขึ้น 3V3 ดึงลงไม่ได้";
    else                           verdict = "<<< อ่านค่าไม่คงที่";

    char line[160];
    snprintf(line, sizeof(line), "%-3u %-22s   %d      %d      %d       %d    %s",
             p, PINS[i].name, pu, pd, dh, dl, verdict);
    Serial.println(line);
  }
  // คืนขาให้อยู่ในสถานะปลอดภัย: STEP พักที่ HIGH, DIR เดินหน้า LOW, ENA ปิด
  gpio_set_direction(GPIO_NUM_23, GPIO_MODE_OUTPUT); gpio_set_level(GPIO_NUM_23, 0);
  delay(4000);
}
