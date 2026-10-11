#!/bin/bash
# build mecanum_base แล้วเปิด Nav2 (navigation.launch.py) ใหม่ -- ปิดตัวเก่าก่อนถ้ามี
#   bash ~/ros-ws/src/mecanum_base/tools/restart_nav2.sh
#
# !! ห้ามรันผ่าน ssh ที่บรรทัดคำสั่งมีชื่อ node ด้านล่างปนอยู่ (เช่น grep waypoint_manager ต่อท้าย)
#    pkill -f จะจับบรรทัดคำสั่ง ssh ของตัวเองแล้วฆ่า session ทิ้ง ดู memory pkill-ssh-self-kill !!

# ROS คุยกันใน Pi อย่างเดียว (โน้ตบุ๊กดูผ่าน Foxglove websocket) -- 2026-09-24 คุมผ่าน WiFi:
# ถ้าใช้ SUBNET node จะผูกกับ IP ตอนเปิด เสียบ/ถอดสายแลนแล้วโปรแกรมที่เปิดใหม่หา node เดิมไม่เจอ
# ต้องตรงกับ start-slam.sh และ ~/.bashrc ไม่งั้น Nav2 กับ SLAM มองไม่เห็นกัน
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
source /opt/ros/jazzy/setup.bash
cd ~/ros-ws && colcon build --packages-select mecanum_base 2>&1 | tail -1
source ~/ros-ws/install/setup.bash
python3 -c "import yaml;yaml.safe_load(open('install/mecanum_base/share/mecanum_base/config/nav2_params.yaml'));print('yaml ok')" || exit 1
LAUNCH="navigation.launch.py"
NODES="controller_server|planner_server|behavior_server|bt_navigator|velocity_smoother|collision_monitor|lifecycle_manager|waypoint_manager|reset_pose_server|map_walls|object_locator|tof_node|keepout_zones"
# ใช้ TERM ไม่ใช่ INT: launch ที่เปิดด้วย "... &" ในสคริปต์ (shell ไม่ interactive) ไม่สน SIGINT
# 2026-09-24 launch เก่ารอด แล้ว respawn object_locator/tof_node กลับมาซ้ำ 2 ตัว
pkill -TERM -f "$LAUNCH"
for i in $(seq 1 15); do
  [ "$(pgrep -f "$LAUNCH|$NODES" | wc -l)" -eq 0 ] && break
  sleep 1
done
pkill -KILL -f "$LAUNCH" 2>/dev/null   # ฆ่าตัว launch ก่อน ไม่งั้นมัน respawn node ที่ตั้ง respawn=True
pkill -KILL -f "$NODES" 2>/dev/null
sleep 1
echo "node เก่าเหลือ: $(pgrep -f "$NODES" | wc -l)  launch เก่าเหลือ: $(pgrep -f "$LAUNCH" | wc -l)"
setsid nohup ros2 launch mecanum_base $LAUNCH > /tmp/nav2.log 2>&1 < /dev/null &
for i in $(seq 1 40); do
  if grep -q "Managed nodes are active" /tmp/nav2.log 2>/dev/null; then echo "Nav2 active ใน ${i} s"; break; fi
  if grep -qE "\[FATAL\]|Failed to bring up" /tmp/nav2.log 2>/dev/null; then echo "Nav2 ล้ม ดู /tmp/nav2.log"; break; fi
  sleep 1
done
n=$(pgrep -f "ros2 launch mecanum_base $LAUNCH" | wc -l)
[ "$n" -ne 1 ] && echo "!! Nav2 launch มี $n ชุด (ควรมี 1)"
grep -E "\[ERROR\]|\[FATAL\]|Traceback" /tmp/nav2.log | grep -v RTPS | cut -c1-200 | head -5
