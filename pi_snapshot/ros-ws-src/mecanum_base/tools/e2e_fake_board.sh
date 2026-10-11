#!/bin/bash
# เทสทั้งระบบบน Pi: mecanum_base_node + fake_board ผ่าน topic/service จริง ไม่ต้องมี ESP32
#   cd ~/ros-ws && colcon build --packages-select mecanum_base && bash src/mecanum_base/tools/e2e_fake_board.sh
# ใช้ ROS_DOMAIN_ID แยก ไม่ชนกับ SLAM ที่รันอยู่  ส่ง rotate_repeat เป็น argument เพื่อวนท่าหมุน/เดินหน้าซ้ำ
HERE="$(cd "$(dirname "$0")" && pwd)"
source /opt/ros/jazzy/setup.bash
source ~/ros-ws/install/setup.bash
export ROS_DOMAIN_ID=77
LIB=~/ros-ws/install/mecanum_base/lib/mecanum_base
CFG=~/ros-ws/install/mecanum_base/share/mecanum_base/config/mecanum_base.yaml
# yaml ปิด vy/wz ไว้ตอนไดรเวอร์ยังไม่ครบ เทสต้องเปิดคืน
# ขนาดหุ่นล็อกไว้ที่ค่าที่ใช้คิดตัวเลขคาดหวังใน e2e_fake_board.py (เช่น scale 0.7664 ของเทส wheel limit)
# ไม่งั้นแก้ขนาดหุ่นจริงใน yaml แล้วเทสจะตกทั้งที่ node ถูก
TEST_PARAMS="-p wheel_radius:=0.05 -p wheel_base:=0.20 -p track_width:=0.24 \
 -p steps_per_rev:=200 -p microsteps:=8 -p gear_ratio:=1.0 -p max_wheel_rate:=3200 \
 -p max_vy:=0.3 -p max_wz:=1.0 -p auto_enable:=true"   # เทสช่วงแรกสั่งวิ่งเลยโดยไม่เรียก ~/enable

python3 -u $LIB/fake_board --link /tmp/fake_mecanum --verbose > /tmp/mb_fake.log 2>&1 &
FAKE=$!
sleep 1
$LIB/mecanum_base_node --ros-args --params-file $CFG -p port:=/tmp/fake_mecanum $TEST_PARAMS > /tmp/mb_node.log 2>&1 &
NODE=$!

python3 -u "$HERE/e2e_fake_board.py" "$@"
RC=$?

kill -INT $NODE; sleep 1; kill -TERM $FAKE; sleep 0.5
kill -9 $NODE $FAKE 2>/dev/null

echo "=== พอร์ตผิด: node ต้องไม่ส่งอะไรออกไปเลย (9 s)"
python3 -u "$HERE/silent_port.py" /tmp/silent_port 9 &
SIL=$!
sleep 0.5
$LIB/mecanum_base_node --ros-args --params-file $CFG -p port:=/tmp/silent_port > /tmp/mb_node_wrong.log 2>&1 &
NODE2=$!
wait $SIL
kill -INT $NODE2; sleep 1; kill -9 $NODE2 2>/dev/null

echo "=== node log";       cat /tmp/mb_node.log
echo "=== fake board log"; cat /tmp/mb_fake.log
echo "=== wrong-port log"; cat /tmp/mb_node_wrong.log
exit $RC
