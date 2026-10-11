#!/bin/bash
# โหลดแมพห้องที่เซฟไว้กลับเข้า slam_toolbox แล้วทำแมพต่อจากเดิม   ใช้: bash load_room.sh [ชื่อ=room_main] [x y theta]
# ไม่ใส่ตำแหน่ง = ใช้ตำแหน่งตอนเซฟ (ต้องจอดหุ่นที่เดิม ทิศเดิม) · ย้ายหุ่นแล้วใส่ตำแหน่งใหม่ในแมพเดิมเอง
# หลังโหลด: Nav2 ใช้แมพเดิมได้เลย สำรวจจะไปเฉพาะขอบที่ยังไม่เคยไป เขตห้ามเข้า/หมุดของตรงที่เดิม
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
source /opt/ros/jazzy/setup.bash
NAME="${1:-room_main}"
F=~/maps/$NAME
[ -f "$F.posegraph" ] || { echo "ไม่มี $F.posegraph"; exit 1; }
if [ $# -ge 4 ]; then X=$2; Y=$3; TH=$4; else read X Y TH < "$F.pose"; fi
echo "โหลด $NAME หุ่นอยู่ที่ x=$X y=$Y theta=$TH"
timeout 90 ros2 service call /slam_toolbox/deserialize_map slam_toolbox/srv/DeserializePoseGraph \
  "{filename: '$F', match_type: 2, initial_pose: {x: $X, y: $Y, theta: $TH}}" | tail -1
sleep 3
echo "ตำแหน่งหุ่นในแมพหลังโหลด: $(python3 ~/ros-ws/src/mecanum_base/tools/map_pose.py)"
