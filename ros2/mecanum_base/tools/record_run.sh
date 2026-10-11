#!/bin/bash
# บันทึกรอบวิ่งเป็น rosbag ไว้ดูย้อนหลัง: ภาพกล้อง (กรอบ YOLO), lidar, TF, odom, คำสั่งล้อ, หมุด, costmap
#   bash record_run.sh [วินาที=720]  -> ~/runs/run_HHMMSS/
# 24 ก.ย. ผู้ใช้ขอ: บันทึกวิดีโอกล้องตอนสำรวจ เพื่อหาว่าหุ่นชนโต๊ะตัวไหนและทำไม
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
source /opt/ros/jazzy/setup.bash
DUR="${1:-720}"
mkdir -p ~/runs
OUT=~/runs/run_$(date +%H%M%S)
echo "$OUT" > ~/runs/latest
exec timeout -s INT "$DUR" ros2 bag record -o "$OUT" \
  /detections/image/compressed /scan_nav /tf /tf_static /odom_rf2o /cmd_vel /objects \
  /local_costmap/costmap /map /waypoint_manager/markers /rosout
