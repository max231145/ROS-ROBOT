#!/bin/bash

# กันรันซ้อน (2026-09-21): เคยรันด้วยมือ "bash ~/start-slam.sh" ตอนที่ systemd กำลังรันตัวเดียวกันอยู่
# สองชุดทับกัน -> yolo / map_image / mecanum_base_node ขึ้นซ้ำ 2 ตัว แย่งพอร์ต ESP32 ต่อติดแล้วหลุดวนไป
# ถ้าจะเปิดใหม่ทั้งระบบให้ใช้  sudo systemctl restart ros2-slam  แทน
# flock -o: ล็อกค้างแค่ตอนสคริปต์นี้รันอยู่ node ลูกที่ nohup ไว้ไม่สืบทอดตัวล็อก (สคริปต์จบ = ปลดล็อก)
# กันรันด้วยมือตอนระบบเปิดอยู่แล้ว (2026-09-23): flock ข้างล่างกันได้แค่ตอนรันพร้อมกัน แต่ service เป็น
# oneshot จบในไม่กี่นาที หลังจากนั้นรันมือได้ -> node ถูกปิดแล้วเปิดใหม่นอก service ทิ้งของค้างซ้ำ (21, 23 ก.ย.)
# รันจาก service = cgroup ros2-slam.service ผ่านได้เสมอ · บังคับรันมือจริง ๆ: FORCE_START=1 bash ~/start-slam.sh
if ! grep -q "ros2-slam.service" /proc/$$/cgroup 2>/dev/null && [ "${FORCE_START:-}" != 1 ] &&    pgrep -f "ydlidar_ros2_driver_node|async_slam_toolbox_node|mecanum_base_node" >/dev/null; then
  echo "ระบบ SLAM เปิดอยู่แล้ว -- ไม่รันซ้ำ (รันด้วยมือตอนระบบเปิดอยู่จะได้ node ซ้ำ)"
  echo "  เปิดใหม่ทั้งระบบ:   sudo systemctl restart ros2-slam"
  echo "  หุ่นแค่ย้ายตำแหน่ง: bash ~/reset_pose.sh  หรือปุ่ม RESET ใน Foxglove"
  echo "  บังคับรันจริง ๆ:    FORCE_START=1 bash ~/start-slam.sh"
  exit 0
fi
if [ "${START_SLAM_LOCKED:-}" != 1 ]; then
  START_SLAM_LOCKED=1 flock -n -o -E 75 /tmp/start-slam.lock "$0" "$@"
  rc=$?
  if [ "$rc" -eq 75 ]; then
    echo "start-slam.sh กำลังรันอยู่แล้ว (อาจเป็น systemd ตอนบูต) -- ไม่รันซ้อน"
    echo "ถ้าจะเปิดใหม่ทั้งระบบ: sudo systemctl restart ros2-slam"
  fi
  exit "$rc"
fi
# เปิดระบบ SLAM + กล้อง ทั้งชุดด้วยคำสั่งเดียว
#   bash ~/start-slam.sh
#
# ลำดับสำคัญ: lidar -> resampler -> rf2o -> slam_toolbox -> camera -> foxglove
# ห้ามสลับ เพราะแต่ละตัวรอ topic จากตัวก่อนหน้า

# ROS คุยกันใน Pi อย่างเดียว (2026-09-24 คุมผ่าน WiFi) -- node ไม่ผูกกับ IP ของสายแลน/WiFi
# เสียบ/ถอดสายแลนแล้วโปรแกรมที่เปิดใหม่ยังเห็น node เดิม ต้องตรงกับ ~/.bashrc และ restart_nav2.sh
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
source /opt/ros/jazzy/setup.bash
source ~/ros-ws/install/setup.bash

# สำคัญ: ต้องใส่ libcamera ที่ build เองไว้หน้าสุด
# เพราะ /opt/ros/jazzy/lib มี libcamera 0.7.1 ต้นทางที่ใช้ชื่อ entity เป็นขีดกลาง
# ซึ่งไม่ตรงกับ kernel 6.8 ของ Ubuntu ที่ใช้ขีดล่าง -> จะได้ "no cameras available"
export LD_LIBRARY_PATH=/usr/local/lib/aarch64-linux-gnu:$LD_LIBRARY_PATH

RED=$'\e[31m'; GRN=$'\e[32m'; YEL=$'\e[33m'; OFF=$'\e[0m'
ok()   { echo "${GRN}  [OK]${OFF} $1"; }
bad()  { echo "${RED}  [ผิดพลาด]${OFF} $1"; }
warn() { echo "${YEL}  [เตือน]${OFF} $1"; }

# รอจน topic โผล่จริง แทนการ sleep ตายตัว
# จำเป็นเพราะเวลาที่แต่ละ node ใช้ไม่คงที่ โดยเฉพาะ ydlidar ที่ต้องรอ
# serial port ถูกปล่อยจาก process เดิมก่อน ซึ่งบางครั้งนานกว่า 14 วินาที
wait_topic() {
  local topic="$1" limit="${2:-45}" i=0
  while [ "$i" -lt "$limit" ]; do
    if timeout 5 ros2 topic list 2>/dev/null | grep -qx "$topic"; then
      echo "$i"; return 0
    fi
    sleep 2; i=$((i + 2))
  done
  return 1
}

echo "=============================================="
echo "  เปิดระบบ SLAM + กล้อง"
echo "=============================================="

# --- ตรวจก่อนเริ่ม ---
echo ""
echo "[0/11] ตรวจความพร้อม"
if [ ! -e /dev/ttyUSB0 ]; then
  bad "ไม่พบ /dev/ttyUSB0 - ยังไม่ได้เสียบ lidar หรือเสียบไม่แน่น"
  exit 1
fi
ok "พบ lidar ที่ /dev/ttyUSB0"

if ! groups | grep -q dialout; then
  bad "user ไม่อยู่ในกลุ่ม dialout - รัน: sudo usermod -aG dialout \$USER แล้ว login ใหม่"
  exit 1
fi
ok "สิทธิ์ dialout ครบ (serial port ของ lidar)"

# กลุ่ม video จำเป็นเมื่อไม่มี GUI เพราะไม่มี ACL จาก logind ให้
if ! groups | grep -q video; then
  warn "user ไม่อยู่ในกลุ่ม video - กล้องจะเปิดไม่ได้ (sudo usermod -aG video \$USER)"
else
  ok "สิทธิ์ video ครบ (media device ของกล้อง)"
fi

HAS_CAM=0
if [ -e /dev/media0 ] && [ -r /dev/media0 ]; then HAS_CAM=1; ok "พบ media device ของกล้อง"; else warn "ไม่พบกล้อง จะข้ามขั้นตอนกล้อง"; fi

IP=$(hostname -I | awk '{print $1}')
ok "IP ของเครื่องนี้: ${IP:-ไม่มี}"

# --- ปิดของเก่าที่อาจค้าง ---
echo ""
echo "[1/11] ปิด node เก่าที่อาจค้างอยู่"
# ros2 daemon ที่เปิดไว้ด้วยค่า ROS_AUTOMATIC_DISCOVERY_RANGE อื่นมองไม่เห็น node ใหม่ -> wait_topic รอ /scan จนหมดเวลา
# (2026-09-24 เปลี่ยนเป็น LOCALHOST แล้ว restart ล้มครั้งแรกเพราะแบบนี้) ปิดทิ้งให้ ros2 topic list เปิดตัวใหม่เอง
ros2 daemon stop >/dev/null 2>&1
pkill -f ydlidar_ros2_driver 2>/dev/null
pkill -f rf2o_laser_odometry 2>/dev/null
pkill -f slam_toolbox 2>/dev/null
pkill -f foxglove_bridge 2>/dev/null
pkill -f scan_resampler 2>/dev/null
pkill -f camera_node 2>/dev/null
pkill -f web_video_server 2>/dev/null
pkill -f yolo_node.py 2>/dev/null
pkill -f map_image_node.py 2>/dev/null
pkill -f "http.server 8090" 2>/dev/null
pkill -f mecanum_base_node 2>/dev/null
pkill -f robot_localization/ekf_node 2>/dev/null
pkill -f odom_covariance_relay 2>/dev/null
pkill -x ros2 2>/dev/null
sleep 4
# camera_node บางครั้งไม่ตายด้วย SIGTERM แต่ยังจับกล้องไว้ ตัวใหม่จะเปิดไม่ได้ ("Pipeline handler in use by another process")
if pgrep -f camera_node >/dev/null; then
  pkill -KILL -f camera_node 2>/dev/null
  sleep 1
  warn "camera_node ตัวเก่าไม่ยอมปิด บังคับปิดด้วย SIGKILL แล้ว"
fi
ok "เคลียร์แล้ว"

echo ""
echo "[2/11] เปิด YDLidar"
nohup ros2 launch ydlidar_ros2_driver ydlidar_launch.py > /tmp/ydlidar.log 2>&1 &
if T=$(wait_topic /scan 60); then
  ok "/scan ทำงาน (ใช้เวลา ${T}s)"
else
  bad "ไม่มี /scan ภายใน 60s - ดู /tmp/ydlidar.log"; exit 1
fi

echo ""
echo "[3/11] เปิด scan_resampler (แก้ปัญหาจำนวนจุดไม่คงที่)"
nohup python3 ~/scan_resampler.py > /tmp/resampler.log 2>&1 &
if T=$(wait_topic /scan_fixed 30); then
  ok "/scan_fixed ทำงาน (ใช้เวลา ${T}s)"
else
  bad "ไม่มี /scan_fixed ภายใน 30s - ดู /tmp/resampler.log"; exit 1
fi

echo ""
echo "[4/11] เปิด rf2o (odometry จาก laser) + EKF + slam_toolbox"
# มี robot_localization: EKF รวม rf2o + wheel_odom แล้วส่ง TF odom -> base_link แทน (rf2o ปิด publish_tf)
#   wheel_odom มาทีหลังในขั้น [11/11] ช่วงแรก EKF ใช้ rf2o อย่างเดียวไปก่อน
# ไม่มี: ใช้ launch เดิมของ rf2o ที่ส่ง TF เอง SLAM ยังทำงานได้
if ros2 pkg prefix robot_localization >/dev/null 2>&1; then
  ODOM_TF="EKF (rf2o + wheel_odom)"
  nohup ros2 launch mecanum_base odometry.launch.py > /tmp/odometry.log 2>&1 &
else
  ODOM_TF="rf2o อย่างเดียว"
  warn "ไม่พบ robot_localization ใช้ TF จาก rf2o อย่างเดียว (sudo apt install ros-jazzy-robot-localization)"
  nohup ros2 launch rf2o_laser_odometry rf2o_laser_odometry.launch.py > /tmp/rf2o.log 2>&1 &
fi
sleep 8
if timeout 6 ros2 run tf2_ros tf2_echo odom base_link 2>&1 | grep -q Translation; then
  ok "TF odom -> base_link จาก ${ODOM_TF}"
else
  warn "ยังไม่มี TF odom -> base_link จาก ${ODOM_TF} - ดู /tmp/odometry.log หรือ /tmp/rf2o.log"
fi
nohup ros2 launch slam_toolbox online_async_launch.py \
  slam_params_file:="$HOME/ros-ws/src/my_slam_config/config/slam_params.yaml" \
  use_sim_time:=false > /tmp/slam.log 2>&1 &
if T=$(wait_topic /map 50); then
  ok "/map ทำงาน (ใช้เวลา ${T}s)"
else
  warn "ยังไม่มี /map ภายใน 50s - ดู /tmp/slam.log"
fi

echo ""
echo "[5/11] เปิดกล้อง Camera Module 3"
if [ "$HAS_CAM" -eq 1 ]; then
  # อย่าใส่ -p AfMode:=2 เด็ดขาด: ทดสอบแล้วกล้องจะ configure สำเร็จแต่ไม่ปล่อยเฟรมเลย
  # (ภาพหยุดนิ่งที่ 0 FPS) ถ้าจะเปิดออโต้โฟกัสต้องหาวิธีอื่น เช่นสั่งตอน runtime
  # orientation:=180 เพราะกล้องติดกลับหัวบนหุ่น (กลับภาพที่เซนเซอร์ ไม่กิน CPU) -- ค่านี้ read-only ต้องตั้งตอนเปิด node
  nohup ros2 run camera_ros camera_node --ros-args \
    -p width:=640 -p height:=480 -p format:=RGB888 -p orientation:=0 \
    -r __node:=camera > /tmp/camera.log 2>&1 &
  if T=$(wait_topic /camera/image_raw 40); then
    ok "/camera/image_raw ทำงาน (ใช้เวลา ${T}s)"
  else
    warn "กล้องเปิดไม่ขึ้นภายใน 40s - ดู /tmp/camera.log (ระบบอื่นยังทำงานปกติ)"
  fi
else
  warn "ข้ามขั้นตอนกล้อง"
fi

echo ""
echo "[6/11] เปิด foxglove_bridge"
# 24 ก.ย. ไม่ส่ง topic หนักให้ Foxglove: เส้นทาง SLAM (graph_visualization โตไม่หยุด) กับภาพกล้องดิบ -- Pi CPU ไม่พอตอนวิ่งตามแมพ
nohup ros2 launch foxglove_bridge foxglove_bridge_launch.xml   topic_whitelist:="['^(?!/slam_toolbox/graph_visualization$|/camera/image_raw$).*']" > /tmp/foxglove.log 2>&1 &
sleep 8
if ss -ltn 2>/dev/null | grep -q ':8765'; then ok "foxglove พอร์ต 8765"; else warn "foxglove ไม่เปิดพอร์ต"; fi

echo ""
echo "[7/11] เปิด web_video_server (ดูภาพสดผ่านเบราว์เซอร์)"
if [ "$HAS_CAM" -eq 1 ]; then
  nohup ros2 run web_video_server web_video_server \
    --ros-args -p port:=8080 -p address:=0.0.0.0 > /tmp/webvideo.log 2>&1 &
  sleep 8
  if ss -ltn 2>/dev/null | grep -q ':8080'; then
    ok "ดูภาพสดได้ที่ http://${IP}:8080"
  else
    warn "web_video_server ไม่เปิดพอร์ต - ดู /tmp/webvideo.log"
  fi
else
  warn "ข้าม web_video_server เพราะไม่มีกล้อง"
fi

echo ""
echo "[8/11] เปิด YOLO object detection"
if [ "$HAS_CAM" -eq 1 ] && [ -f "$HOME/yolo/yolov8n.onnx" ]; then
  nohup python3 "$HOME/yolo_node.py" > /tmp/yolo.log 2>&1 &
  if T=$(wait_topic /detections/image/compressed 45); then
    ok "YOLO ทำงาน (ใช้เวลา ${T}s)"
  else
    warn "YOLO ไม่ขึ้นภายใน 45s - ดู /tmp/yolo.log (ระบบอื่นยังทำงานปกติ)"
  fi
else
  warn "ข้าม YOLO (ไม่มีกล้องหรือไม่พบ ~/yolo/yolov8n.onnx)"
fi

echo ""
echo "[9/11] เปิด map_image_node (แปลงแมพเป็นภาพให้เว็บดูได้ + เขียน status.json)"
nohup python3 "$HOME/map_image_node.py" > /tmp/mapimg.log 2>&1 &
if T=$(wait_topic /map_image/compressed 40); then
  ok "/map_image ทำงาน (ใช้เวลา ${T}s)"
else
  warn "map_image ไม่ขึ้นภายใน 40s - ดู /tmp/mapimg.log"
fi

echo ""
echo "[10/11] เปิดหน้า dashboard"
if [ -f "$HOME/web/index.html" ]; then
  nohup python3 -m http.server 8090 --directory "$HOME/web" --bind 0.0.0.0 \
    > /tmp/webui.log 2>&1 &
  sleep 5
  if ss -ltn 2>/dev/null | grep -q ':8090'; then
    ok "dashboard พร้อมที่ http://${IP}:8090"
  else
    warn "dashboard ไม่เปิดพอร์ต - ดู /tmp/webui.log"
  fi
else
  warn "ไม่พบ ~/web/index.html ข้าม dashboard"
fi

echo ""
echo "[11/11] เปิดฐานขับ mecanum (ESP32 + DM556)"
# ไม่เช็คว่าเสียบ ESP32 ก่อน: node ลองเปิดพอร์ตซ้ำทุก 2 วินาทีเอง เสียบทีหลังก็ต่อติด
# ไดรเวอร์ปิดไว้ตอนบูต (auto_enable: false) มอเตอร์ไม่มีไฟจนกว่าจะเรียก service enable
nohup ros2 launch mecanum_base mecanum_base.launch.py > /tmp/mecanum_base.log 2>&1 &
MB_OK=0
for i in $(seq 1 20); do
  if grep -q "ต่อกับบอร์ด" /tmp/mecanum_base.log 2>/dev/null; then MB_OK=1; break; fi
  sleep 1
done
if [ "$MB_OK" -eq 1 ]; then
  ok "ต่อ ESP32 ฐานขับแล้ว ไดรเวอร์ยังปิดอยู่ (ใช้เวลา ${i}s)"
else
  warn "ยังต่อ ESP32 ฐานขับไม่ได้ภายใน 20s - ดู /tmp/mecanum_base.log (ระบบอื่นยังทำงานปกติ)"
fi

# --- ตรวจผล ---
echo ""
echo "=============================================="
echo "  ตรวจผล"
echo "=============================================="

MAPHZ=$(timeout 14 ros2 topic hz /map 2>&1 | grep -oE 'average rate: [0-9.]+' | tail -1)
if [ -n "$MAPHZ" ]; then ok "/map ออกแล้ว ($MAPHZ)"; else bad "/map ไม่ออก - ดู /tmp/slam.log"; fi

ERR=$(grep -c 'expected' /tmp/slam.log 2>/dev/null || true)
ERR=${ERR:-0}
if [ "$ERR" -eq 0 ]; then
  ok "ไม่มี scan ถูกทิ้ง (error = 0)"
else
  warn "มี scan ถูกทิ้ง $ERR ครั้ง - resampler อาจมีปัญหา"
fi

if [ "$HAS_CAM" -eq 1 ]; then
  CAMHZ=$(timeout 12 ros2 topic hz /camera/image_raw 2>&1 | grep -oE 'average rate: [0-9.]+' | tail -1)
  if [ -n "$CAMHZ" ]; then ok "กล้องทำงาน ($CAMHZ)"; else warn "กล้องไม่มีภาพ"; fi
fi

if timeout 6 ros2 run tf2_ros tf2_echo map base_link 2>&1 | grep -q Translation; then
  ok "TF map -> base_link ครบ"
else
  warn "ยังหา TF map -> base_link ไม่เจอ (ปกติถ้าเพิ่งเปิด รออีกสักครู่)"
fi

echo ""
echo "  RAM:  $(free -h | awk '/^Mem:/{print $3" / "$2}')"
echo "  อุณหภูมิ: $(awk '{printf "%.1f C", $1/1000}' /sys/class/thermal/thermal_zone0/temp)"
echo ""
echo "=============================================="
echo "  Foxglove:   ws://${IP}:8765"
echo "    - แมพ      : /map"
echo "    - ภาพกล้อง : /camera/image_raw/compressed"
echo "    - YOLO     : /detections/image/compressed  +  /detections/summary"
echo "  Dashboard:  http://${IP}:8090   <-- รวมทุกอย่างไว้ที่นี่"
echo "  ภาพสดเว็บ:  http://${IP}:8080"
echo "  ฐานขับ:     ros2 service call /mecanum_base/enable std_srvs/srv/SetBool \"{data: true}\""
echo "              แล้วส่ง /cmd_vel  เลิกใช้แล้วสั่ง {data: false} ตัดไฟมอเตอร์"
echo "  ปิดระบบ:    bash ~/stop-slam.sh"
echo "  เซฟแมพ:     bash ~/save-map.sh ชื่อแมพ"
echo "=============================================="
