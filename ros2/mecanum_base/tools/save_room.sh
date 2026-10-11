#!/bin/bash
# เซฟแมพห้อง + ตำแหน่งหุ่นตอนจอด ไว้โหลดกลับด้วย load_room.sh   ใช้: bash save_room.sh [ชื่อ=room_main]
# จอดหุ่นไว้ที่เดิมแล้วค่อยปิดเครื่อง -- รอบหน้าโหลดแมพ หุ่นจะรู้ว่าอยู่ตรงไหนในแมพเดิมทันที
# เขตห้ามเข้า (keepout_zones.json) กับหมุดของ (objects.json) ผูกกับเฟรมของแมพนี้ ใช้คู่กันได้
# 24 ก.ย. ผู้ใช้: โหลดแมพเดิมแล้วสำรวจต่อ ไม่เดินซ้ำที่เก่า (ข้อ 3)
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
source /opt/ros/jazzy/setup.bash
NAME="${1:-room_main}"
mkdir -p ~/maps
POSE=$(python3 ~/ros-ws/src/mecanum_base/tools/map_pose.py) || exit 1
timeout 60 ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
  "{filename: '$HOME/maps/$NAME'}" | tail -1
echo "$POSE" > ~/maps/$NAME.pose
# ให้ SLAM โหลดแมพนี้ตอนเปิดครั้งหน้า (slam_params.yaml: map_file_name + map_start_pose)
CFG=~/ros-ws/src/my_slam_config/config/slam_params.yaml
read X Y TH <<< "$POSE"
sed -i "s#^    map_file_name: .*#    map_file_name: $HOME/maps/$NAME#; s#^    map_start_pose: .*#    map_start_pose: [$X, $Y, $TH]#" $CFG
grep -E "map_file_name|map_start_pose" $CFG
echo "เซฟ $NAME แล้ว หุ่นอยู่ที่ (x y theta) = $POSE"
ls -la ~/maps/$NAME.*
