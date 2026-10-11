#!/bin/bash
# วิ่งทดสอบ 1 m บนพื้นกับระบบจริง (ros2-slam.service ต้องรันอยู่) แล้วปิดไดรเวอร์เสมอ
#   bash ~/ros-ws/src/mecanum_base/tools/floor_ekf_run.sh            # เริ่มล็อกล้อทันที
#   PRE_WAIT=30 bash .../floor_ekf_run.sh                            # ไดรเวอร์ปิดไว้ก่อน 30 s ให้จัดหุ่นเข้าที่
# ถ้าต้องให้คนดูผลด้วยตา ให้รันแบบ background:  setsid nohup bash .../floor_ekf_run.sh > /tmp/floor_ekf.out 2>&1 &
HERE="$(cd "$(dirname "$0")" && pwd)"
source /opt/ros/jazzy/setup.bash && source ~/ros-ws/install/setup.bash
if [ "${PRE_WAIT:-0}" -gt 0 ]; then
  echo "$(date +%T) driver OFF for ${PRE_WAIT} s (reposition robot now)"
  sleep "$PRE_WAIT"
fi
timeout -k 5 -s TERM 200 python3 -u "$HERE/floor_ekf_test.py"
echo "$(date +%T) wrapper: disable driver again: $(timeout 15 ros2 service call /mecanum_base/enable std_srvs/srv/SetBool '{data: false}' 2>&1 | grep -o 'success=[A-Za-z]*')"
echo "$(date +%T) end"
