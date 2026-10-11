#!/bin/bash
# รีเซ็ตตำแหน่งหุ่นเป็น 0 (ใช้หลังย้าย/หมุนหุ่นด้วยมือตอนระบบรัน) -- ไม่ต้อง sudo
#   bash ~/reset_pose.sh       (หรือกดปุ่ม RESET ใน Foxglove)
# ตัวจริงอยู่ใน mecanum_base/reset_pose.py
source /opt/ros/jazzy/setup.bash
source ~/ros-ws/install/setup.bash
ros2 run mecanum_base reset_pose 2>&1 | grep -v RTPS_TRANSPORT
exit ${PIPESTATUS[0]}
