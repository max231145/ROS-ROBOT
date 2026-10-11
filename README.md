# esp32_dm556_stepper

ESP32 คุมสเต็ปปิงมอเตอร์ 4 ตัวผ่านไดรเวอร์ DM556 แบบ position control
พร้อม ROS 2 package สำหรับรันบน Raspberry Pi 5

```
ROS 2 (Pi 5)                            ESP32                    DM556 x4
┌────────────────────┐   USB serial   ┌──────────────┐  STEP/DIR ┌────────┐
│ stepper_bridge     │ ─────────────► │ timer 20 kHz │ ────────► │ ไดรเวอร์ │ ─► มอเตอร์
│ rad/m  <-> step    │ ◄───────────── │ planner 1 kHz│ ◄──────── │        │
│ /joint_states      │  status 20 Hz  └──────────────┘  ลิมิตสวิตช์└────────┘
└────────────────────┘
```

ไฟล์:

| ไฟล์ | หน้าที่ |
|---|---|
| `esp32_dm556_stepper.ino` | เฟิร์มแวร์ ESP32 คิดเป็น step ล้วน ๆ |
| `test_stepper_only/` | สเก็ตช์แยก ทดสอบมอเตอร์อย่างเดียว ไม่มี ROS ไม่มีโปรโตคอล ใช้ไล่ปัญหาสายไฟ |
| `ros2/stepper_bridge/` | ROS 2 package (ament_python) แปลงหน่วยและเปิด topic/service |
| `ros2/stepper_hardware/` | ros2_control hardware_interface (C++) สำหรับใช้กับ MoveIt / trajectory controller |
| `tools/bench_test.py` | สคริปต์ทดสอบบนโต๊ะ ไม่ต้องมี ROS |

**เลือกอันไหนดี** — ใช้ตัวใดตัวหนึ่ง ไม่ต้องรันพร้อมกัน (แย่งพอร์ตกัน)

- `stepper_bridge` — สั่งมุมไปแล้วรอให้ถึง แต่ละแกนวิ่งอิสระ ถึงไม่พร้อมกัน ติดตั้งง่าย ดีบั๊กง่าย
- `stepper_hardware` — ต้องการให้ 4 แกนเดินประสานกันตามเวลา หรือจะใช้ MoveIt (ดูหัวข้อ 9)

---

## 1. การต่อสาย

DM556 รับสัญญาณผ่านออปโตคัปเปลอร์ ขาเป็นคู่ `PUL+/PUL-`, `DIR+/DIR-`, `ENA+/ENA-`
ที่ออกแบบมาสำหรับลอจิก 5 V ส่วน ESP32 เป็น 3.3 V จึงต้องต่อแบบ **common-anode**
คือเอาขา `+` ทั้งหมดรวมเข้า 3V3 แล้วให้ GPIO ไปดึงขา `-` ลง GND

```
ESP32 3V3 ──┬── PUL+  (ไดรเวอร์ 1)
            ├── DIR+
            ├── ENA+
            └── ... ต่อแบบเดียวกันทั้ง 4 ตัว

GPIO25 ───────── PUL-  (ไดรเวอร์ 1)      GPIO26 ───── DIR-
GPIO27 ───────── PUL-  (ไดรเวอร์ 2)      GPIO14 ───── DIR-
GPIO32 ───────── PUL-  (ไดรเวอร์ 3)      GPIO33 ───── DIR-
GPIO18 ───────── PUL-  (ไดรเวอร์ 4)      GPIO19 ───── DIR-
GPIO23 ───────── ENA-  (ต่อขนานทั้ง 4 ตัว)

GND ของ ESP32 ต่อกับ GND ฝั่งลอจิกของ DM556 (ไม่ใช่ขั้วลบของไฟ 24-48 V)
```

กระแสที่ไหลผ่านออปโตตอน 3.3 V ≈ (3.3 − 1.2) / 270 Ω ≈ 7.8 mA
พอใช้งานได้ แต่ถ้าสายยาวเกิน ~50 cm หรือเจอสเต็ปหายตอนวิ่งเร็ว
ให้ใส่บัฟเฟอร์ **74HCT245 เลี้ยงไฟ 5 V** คั่นระหว่าง ESP32 กับไดรเวอร์
(74HCT รับอินพุต 3.3 V ได้ตรง ๆ ไม่ต้องมีตัวแบ่งแรงดัน) แล้วเปลี่ยน
`STEP_ACTIVE_LOW` / `ENABLE_ACTIVE_LOW` ให้ตรงกับการต่อใหม่

**ลิมิตสวิตช์** — ต่อระหว่างขา GPIO กับ GND ใช้ pull-up ในตัว ESP32 ไม่ต้องมีตัวต้านทานนอก

| แกน | STEP | DIR | LIMIT |
|---|---|---|---|
| 0 | 25 | 26 | 16 |
| 1 | 27 | 14 | 17 |
| 2 | 32 | 33 | 4 |
| 3 | 18 | 19 | 13 |

แกนไหนไม่มีลิมิตสวิตช์ ให้ตั้ง `PIN_LIMIT` ของแกนนั้นเป็น `-1`

**เรื่องที่พลาดกันบ่อย**

- ห้ามใช้ GPIO 0, 2, 5, 12, 15 กับลิมิตสวิตช์ เพราะเป็น strapping pin ถ้าสวิตช์ทับอยู่ตอนเปิดเครื่อง ESP32 จะบูตไม่ขึ้น
- ไฟ 24-48 V ของ DM556 ต้องแยกแหล่งจ่ายกับ 5 V ของ ESP32 และ **ห้าม** ต่อ GND ของไฟกำลังเข้ากับ GND ลอจิก
- ตอนเสียบ USB จาก Pi เข้า ESP32 บอร์ดจะรีเซ็ต (DTR) มอเตอร์จะปล่อยฟรีชั่วขณะ ถ้าแกนไหนมีน้ำหนักถ่วงต้องมีเบรกกลไก

---

## 2. ตั้ง DIP switch ของ DM556

SW1-SW3 คือ microstep, SW4-SW6 คือกระแส, SW7 คือ half current ตอนหยุดนิ่ง

- **microstep** — เริ่มที่ 1600 pulse/rev (8 microstep) จะได้ความละเอียด 0.225°/step
  ค่าที่ตั้งต้องกรอกลงในไฟล์ `config/steppers.yaml` ให้ตรง มิฉะนั้นองศาที่สั่งจะไม่ตรงของจริง
- **กระแส** — ตั้งให้ตรงกับสเปกมอเตอร์ ตั้งเกินแล้วมอเตอร์ร้อนจัด ตั้งขาดแล้วสเต็ปหลุดตอนเร่ง
- **SW7 (half current)** — เปิดไว้ ช่วยลดความร้อนตอนถือตำแหน่งนิ่ง ๆ

เพดานความเร็วของเฟิร์มแวร์คือ 10,000 step/s ต่อแกน (= `ISR_HZ / 2`)
ที่ 1600 pulse/rev เท่ากับ 375 rpm ถ้าต้องการเร็วกว่านี้ให้ลด microstep ลง
หรือดัน `ISR_HZ` เป็น 40000 (กิน CPU มากขึ้น แต่ ESP32 ยังไหว)

---

## 3. อัปโหลดเฟิร์มแวร์

ไม่ต้องลงไลบรารีเพิ่ม ใช้แค่ arduino-esp32 core (รองรับทั้ง 2.x และ 3.x)

Arduino IDE: เลือกบอร์ด **ESP32 Dev Module** แล้วกด Upload
หรือใช้ arduino-cli:

```bash
arduino-cli compile --fqbn esp32:esp32:esp32 esp32_dm556_stepper
```

```bash
arduino-cli upload -p COM5 --fqbn esp32:esp32:esp32 esp32_dm556_stepper
```

เปิด Serial Monitor ที่ 115200 ควรเห็น `# esp32_dm556_stepper ready axes=4`

---

## 4. ทดสอบบนโต๊ะก่อนเอาเข้า ROS

มีสองระดับ ให้ไล่จากบนลงล่าง อย่าข้ามขั้น

### 4.1 ทดสอบมอเตอร์อย่างเดียว -- `test_stepper_only/`

อัปโหลดสเก็ตช์ในโฟลเดอร์ `test_stepper_only` (คนละโฟลเดอร์กับตัวจริง เพราะ Arduino
ยอมให้มี `setup()`/`loop()` ได้ชุดเดียวต่อโฟลเดอร์) แล้วเปิด Serial Monitor 115200
**ตั้งท้ายบรรทัดเป็น Newline** ไม่งั้นคำสั่งจะไม่ถูกส่ง

ตัวนี้ไม่มี ROS ไม่มีโปรโตคอล ไม่มี timer ISR มีแค่ยิงพัลส์ตรง ๆ ด้วย `delayMicroseconds()`
จุดประสงค์คือแยกให้ชัดว่าปัญหาอยู่ที่สายไฟหรืออยู่ที่ตรรกะในเฟิร์มแวร์ตัวจริง

กด `t` เพื่อรันชุดทดสอบอัตโนมัติ มันจะไล่ให้ 6 ข้อ ตัดไฟ → จ่ายไฟ → หมุนไป 1 รอบ →
หมุนกลับ 1 รอบ → เดินสั้น ๆ 20 step → อ่านลิมิตสวิตช์ พร้อมบอกว่าแต่ละข้อควรเห็นอะไร

คำสั่งที่ใช้บ่อย (กด `?` ดูครบ):

| คำสั่ง | |
|---|---|
| `a 2` | เปลี่ยนไปทดสอบแกน 2 |
| `e` | สลับจ่าย/ตัดไฟเลี้ยงมอเตอร์ |
| `f` / `b` | เดินหน้า / ถอยหลัง ตามจำนวนที่ตั้งไว้ |
| `1` | ยิงพัลส์เดียว ดูว่าขยับ 1 microstep จริงไหม |
| `r` | หมุนต่อเนื่องจนกด Enter |
| `s 2000` | ตั้งความเร็วปลาย 2000 step/s |
| `n 3200` | ตั้งจำนวน step ต่อครั้ง |
| `l` | อ่านลิมิตสวิตช์ทุกแกน |
| `i` | เลิกสนใจลิมิตสวิตช์ (ตอนยังไม่ได้ต่อสวิตช์) |

ตัวนับพัลส์สะสมแสดงเป็นจำนวนรอบให้ด้วย เอาไว้เทียบกับรอบที่นับด้วยตา
ถ้าตัวเลขไม่ตรง แปลว่า `microstep` ที่กรอกไว้ (คำสั่ง `m`) ไม่ตรงกับ DIP จริง

### 4.2 ทดสอบผ่านเฟิร์มแวร์ตัวจริง -- `tools/bench_test.py`

พอ 4.1 ผ่านแล้วค่อยอัปโหลด `esp32_dm556_stepper.ino` แล้วทดสอบทีละแกนโดยยังไม่ต่อกับกลไก

```bash
python3 tools/bench_test.py --port /dev/ttyUSB0 --axis 0 --steps 3200
```

สิ่งที่ต้องเช็ก:

1. สั่ง `E 1` แล้วมอเตอร์ต้อง **ล็อกแน่น** ถ้ายังหมุนด้วยมือได้แสดงว่าขั้ว ENA กลับ ให้แก้ `ENABLE_ACTIVE_LOW` ในไฟล์ `.ino` เป็น `1`
2. สั่งค่าบวกแล้วมอเตอร์ต้องหมุนไปทางที่นับเป็นบวก ถ้ากลับด้านให้แก้ `DIR_INVERT` ของแกนนั้น
3. มอเตอร์สั่นแต่ไม่หมุน / เสียงครวญ = พัลส์ไปไม่ถึงไดรเวอร์ ให้ตรวจ `STEP_ACTIVE_LOW` และการต่อ common-anode
4. กดลิมิตสวิตช์ด้วยมือ แล้วดูบิต `AF_LIMIT` (ค่า 4) ในบรรทัด status ว่าขึ้นจริง

---

## 5. เอา ROS package ขึ้น Pi 5

คัดลอกโฟลเดอร์ `ros2/stepper_bridge` เข้า workspace บน Pi แล้วบิลด์

```bash
sudo apt install python3-serial
```

```bash
cp -r ros2/stepper_bridge ~/ros2_ws/src/ && cd ~/ros2_ws && colcon build --packages-select stepper_bridge --symlink-install
```

Pi ต้องเข้าถึงพอร์ตได้ ถ้ายังไม่เคยตั้ง:

```bash
sudo usermod -aG dialout $USER
```

(ต้อง logout แล้ว login ใหม่ถึงจะมีผล)

ถ้ามีอุปกรณ์ USB serial หลายตัว (เช่นมี YDLidar อยู่ด้วย) `/dev/ttyUSB0` จะสลับกันเองตอนบูต
ให้ผูกชื่อถาวรด้วย udev rule โดยใช้ serial ของชิป CP2102/CH340:

```bash
udevadm info -a -n /dev/ttyUSB0 | grep -m1 -E 'ATTRS\{(serial|idVendor|idProduct)\}'
```

แล้วเขียน `/etc/udev/rules.d/99-esp32-stepper.rules`

```
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", ATTRS{serial}=="<ค่าที่ได้>", SYMLINK+="esp32_stepper"
```

---

## 6. ใช้งานฝั่ง ROS

แก้ `config/steppers.yaml` ให้ตรงกับกลไกจริงก่อน (microstep, อัตราทด, ลิมิต) แล้วสั่ง

```bash
ros2 launch stepper_bridge stepper_bridge.launch.py port:=/dev/esp32_stepper
```

จ่ายไฟมอเตอร์แล้ว home ทุกแกน (home ทีละแกน ไม่ขยับพร้อมกัน กันไฟตก):

```bash
ros2 service call /stepper_bridge/enable std_srvs/srv/SetBool "{data: true}"
```

```bash
ros2 service call /stepper_bridge/home std_srvs/srv/Trigger
```

สั่งตำแหน่ง (หน่วยตาม `units_per_rev` ของแกนนั้น — rad สำหรับแกนหมุน, m สำหรับแกนเลื่อน):

```bash
ros2 topic pub --once /stepper_bridge/command sensor_msgs/msg/JointState "{name: ['joint_base','joint_lift'], position: [0.785, 0.10]}"
```

อ่านตำแหน่งจริงกลับมา:

```bash
ros2 topic echo /joint_states
```

### topic / service ทั้งหมด

| ชื่อ | ชนิด | หมายเหตุ |
|---|---|---|
| `~/command` | sub `sensor_msgs/JointState` | ใช้ `name` + `position`; ถ้าใส่ `velocity` มาด้วยจะใช้เป็นเพดานความเร็วของ move นั้น |
| `/joint_states` | pub `sensor_msgs/JointState` | ตำแหน่ง + ความเร็วจริง ต่อเข้า `robot_state_publisher` / RViz ได้เลย |
| `~/flags` | pub `std_msgs/UInt8MultiArray` | บิตต่อแกน: 1=moving 2=homed 4=limit 8=fault 16=homing |
| `~/enable` | srv `std_srvs/SetBool` | จ่าย/ตัดไฟเลี้ยงมอเตอร์ |
| `~/home` | srv `std_srvs/Trigger` | home ทีละแกนจนครบ |
| `~/stop` | srv `std_srvs/Trigger` | เบรกแบบมี ramp |
| `~/estop` | srv `std_srvs/Trigger` | หยุดทันที + ตัด ENA ต้องเรียก `clear_fault` ก่อนใช้ต่อ |
| `~/clear_fault` | srv `std_srvs/Trigger` | |
| `~/zero` | srv `std_srvs/Trigger` | เซ็ตตำแหน่งปัจจุบันเป็น 0 (ใช้ตอนไม่มีลิมิตสวิตช์) |

### การคิดหน่วย

```
step ต่อ 1 รอบด้านออก = steps_per_rev × microsteps × gear_ratio
steps_per_unit        = step ต่อรอบด้านออก ÷ units_per_rev
ตำแหน่ง step          = (ค่า joint − offset) × steps_per_unit
```

`offset` คือค่า joint ที่ตรงกับจุด home พอดี เช่นถ้า home แล้วแขนอยู่ที่ −90°
ให้ตั้ง `offset: -1.5707963`

---

## 7. โปรโตคอล serial (ถ้าจะต่อจากภาษาอื่น)

ASCII บรรทัดละคำสั่ง จบด้วย `\n`

| คำสั่ง | ความหมาย |
|---|---|
| `E 1` / `E 0` | เปิด/ปิดไฟเลี้ยงมอเตอร์ |
| `M <ax> <step>` | ตำแหน่งสัมบูรณ์ |
| `R <ax> <step>` | ตำแหน่งสัมพัทธ์ |
| `MA <p0> <p1> <p2> <p3>` | สั่ง 4 แกนพร้อมกัน (`*` = แกนนั้นไม่เปลี่ยน) |
| `T <dt_us> <p0>..<p3>` | setpoint โหมด streaming สำหรับ ros2_control ไม่ตอบ `OK` กลับ |
| `V <ax> <vmax> <accel>` | step/s และ step/s² |
| `L <ax> <min> <max>` | soft limit (min == max = ปิด) |
| `Z <ax> [step]` | เซ็ตตำแหน่งปัจจุบัน |
| `H <ax> [dir]` | home |
| `S` / `X` / `C` | เบรก / E-stop / เคลียร์ fault |
| `F <hz>` | ความถี่ status (0 = ปิด) |
| `Q` | ขอ status 1 บรรทัด |
| `P` | heartbeat |

บอร์ดตอบกลับ:

```
S <ms> <sysflag> <pos0> <vel0> <flag0> <pos1> <vel1> <flag1> ... x4
```

`sysflag`: bit0 = ไฟเลี้ยงมอเตอร์เปิด, bit1 = อยู่ในสถานะ fault
บรรทัดที่ขึ้นต้นด้วย `#` คือ log, `OK`/`ERR` คือผลของคำสั่ง

**heartbeat** — ถ้าบอร์ดไม่ได้รับอะไรเลยเกิน 2 วินาที (`HOST_TIMEOUT_MS`)
จะเบรกทุกแกนทิ้ง กันกรณีสาย USB หลุดหรือ node ตายกลางทาง
ฝั่ง ROS ส่ง `P` ให้อยู่แล้วทุก 100 ms

---

## 8. ข้อจำกัดที่ควรรู้

- **ไม่มี encoder** สเต็ปหลุดแล้วระบบไม่รู้ตัว ตำแหน่งจะเพี้ยนไปเรื่อย ๆ จนกว่าจะ home ใหม่ ถ้างานต้องการความมั่นใจ ให้ home ซ้ำเป็นระยะ หรือเติม encoder ที่แกนสำคัญ
- **ramp คิดที่ 1 kHz** ความเร็วจึงเปลี่ยนเป็นขั้นละ 1 ms พอสำหรับ position control แต่ไม่เหมาะกับงานที่ต้องคุมความเร็วต่อเนื่องแบบเนียน ๆ
- **soft limit ถูกล้างระหว่าง home** เพราะยังไม่รู้ระบบพิกัด node จะยัดค่ากลับให้เองหลัง home เสร็จ

---

## 9. ใช้กับ ros2_control (แพ็กเกจ `stepper_hardware`)

ใช้เมื่อต้องการให้ 4 แกนเดินประสานกันตามเวลา หรือจะต่อ MoveIt
ถ้าแค่สั่งมุมทีละจุด `stepper_bridge` ในหัวข้อ 6 พอแล้ว ไม่ต้องมาทางนี้

### hardware_interface ทำอะไร

เป็นปลั๊กอินที่แปลงระหว่างหน่วยความจำของ ros2_control กับพอร์ต serial เท่านั้น
ไม่มีตรรกะควบคุมอยู่ในนั้นเลย ตัวคิด trajectory คือ controller ชั้นบน

| เมธอด | ทำอะไรในโค้ดนี้ |
|---|---|
| `on_init` | อ่าน `<param>` จากแท็ก `<ros2_control>` ใน URDF คิดค่า `steps_per_unit` ของแต่ละ joint |
| `export_state_interfaces` | ส่งคืนพอยเตอร์ของ `double` ตำแหน่ง/ความเร็ว ให้ framework อ่านตรง ๆ |
| `export_command_interfaces` | ส่งคืนพอยเตอร์ของ `double` ค่าสั่งตำแหน่ง |
| `on_configure` | เปิดพอร์ต รอ ESP32 บูต 2.2 วิ แล้วยัด `F` / `V` / `L` ลงบอร์ด |
| `on_activate` | ส่ง `E 1` รอ status ก้อนแรก แล้ว **ก๊อปตำแหน่งจริงใส่ค่าสั่ง** ไม่งั้นแขนกระตุกตั้งแต่รอบแรก |
| `read` | หยิบ snapshot ล่าสุดที่ thread อ่านไว้ แปลง step → rad/m |
| `write` | แปลง rad/m → step แล้วส่ง `T <dt_us> ...` |
| `on_deactivate` | ส่ง `S` แล้ว `E 0` |

### สองเรื่องที่โค้ดนี้จัดการไว้ให้แล้ว

**1. `read()` / `write()` อยู่ในลูป real-time ห้ามบล็อก** — ถ้าเขียนแบบส่งคำสั่งแล้วรอคำตอบ
ลูปจะค้างทันที `SerialLink` จึงแยก thread อ่านพอร์ตต่างหาก แล้วเก็บค่าล่าสุดไว้ใน seqlock
`read()` แค่มาหยิบไป ไม่ต้องรอ lock ไม่จองหน่วยความจำ ส่วน `write()` เขียนแบบ non-blocking
ถ้าบัฟเฟอร์ tty เต็มจะทิ้งบรรทัดนั้นแล้วนับไว้ (ยอมทิ้งดีกว่าให้ลูปค้าง เดี๋ยวคาบหน้าก็ส่งใหม่)

**2. ปัญหาวางแผนซ้อนกัน** — `joint_trajectory_controller` อินเทอร์โพเลตเส้นทางมาให้แล้ว
ถ้าปล่อยให้พลานเนอร์ในบอร์ด ramp อีกชั้นจะกลายเป็น ramp ซ้อน ramp แขนตามหลังตลอดและเลยเป้า
เฟิร์มแวร์จึงมีโหมด streaming (คำสั่ง `T`) ที่ข้ามพลานเนอร์ไปเลย แล้วคิดความเร็วจาก

```
ความเร็วที่สั่ง = feed-forward + (setpoint − ตำแหน่งจริง) / คาบควบคุม
```

feed-forward มาจากส่วนต่างของ setpoint สองครั้งล่าสุด ทำให้ไม่มีค่าคลาดค้างตอนวิ่งความเร็วคงที่
ส่วนพจน์หลังทำให้ค่าคลาดสลายตัวแบบ first-order และไม่มีทางเลยเป้า
แล้วยังจำกัดอัตราการเปลี่ยนความเร็วด้วย `accel` อีกชั้น กัน setpoint กระโดดจนสเต็ปหลุด
ถ้าอยากปิดโหมดนี้ (เช่นตอนดีบั๊ก) ตั้ง `<param name="stream">false</param>` แล้วมันจะกลับไปใช้ `MA`

### บิลด์และรัน

```bash
cp -r ros2/stepper_hardware ~/ros2_ws/src/ && cd ~/ros2_ws && colcon build --packages-select stepper_hardware --symlink-install
```

> Pi 5 แรม 4 GB บิลด์ C++ พร้อมกันหลาย job แล้วโดน OOM ได้ ถ้าเจอให้ใส่
> `--parallel-workers 1` และ `--cmake-args -DCMAKE_CXX_FLAGS_RELEASE=-g0`

แก้ `urdf/stepper_arm.ros2_control.xacro` ให้ตรงกับกลไกจริงก่อน (microstep, อัตราทด, ลิมิต, offset)
ไฟล์นั้นคือแหล่งความจริงแหล่งเดียวของการแปลงหน่วยในเส้นทางนี้ แล้วสั่ง

```bash
ros2 launch stepper_hardware stepper_control.launch.py port:=/dev/esp32_stepper
```

ตรวจว่าปลั๊กอินโหลดขึ้นจริง:

```bash
ros2 control list_hardware_interfaces
```

สั่ง trajectory:

```bash
ros2 topic pub --once /arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "{joint_names: ['joint_base','joint_shoulder','joint_elbow','joint_lift'], points: [{positions: [0.5, 0.8, -0.8, 0.05], time_from_start: {sec: 4}}]}"
```

### แบนด์วิดท์ -- เรื่องที่จะทำให้พังถ้าตั้งผิด

`update_rate` ใน `config/controllers.yaml` ตั้งไว้ **50 Hz** เพราะที่ 115200 baud
คำสั่ง `T` ขาไปกับ status ขากลับกินรวมราว 5.5 kB/s จาก 11.5 kB/s ที่มี

จะดันเป็น 100-200 Hz ต้องทำสองอย่างพร้อมกัน ไม่งั้นคำสั่งจะถูกทิ้ง (ดู log เตือน "ส่งคำสั่งไม่ทัน"):

1. แก้ `SERIAL_BAUD` ในไฟล์ `.ino` เป็น `921600`
2. ตั้ง `baud:=921600` ตอน launch

### ข้อควรรู้

- **ต้อง home ก่อนใช้งานจริงทุกครั้ง** ปลั๊กอินนี้ยังไม่มี service home ในตัว ถ้ายังไม่ได้ home
  มันจะเตือนใน log แล้วถือว่าตำแหน่งตอนเปิดบอร์ดคือ 0 วิธีที่ง่ายที่สุดคือรัน `stepper_bridge`
  เพื่อ home ให้จบก่อน ปิด แล้วค่อย launch ตัวนี้ (อย่ารันพร้อมกัน แย่งพอร์ต)
- ค่าคลาดระหว่างทางจะอยู่ราวครึ่งหนึ่งของระยะที่วิ่งได้ในหนึ่งคาบควบคุม
  ที่ 50 Hz กับ 1 rad/s ราว 0.01 rad ตั้ง `constraints.<joint>.trajectory` ให้เผื่อกว่านั้น
  ไม่งั้น goal จะถูกยกเลิกกลางทางทั้งที่กลไกวิ่งดีอยู่
- ชนลิมิตสวิตช์ระหว่างทาง `read()` จะคืน `ERROR` ทำให้ controller_manager ปลดคอมโพเนนต์
  ต้องเคลียร์ fault แล้ว activate ใหม่
