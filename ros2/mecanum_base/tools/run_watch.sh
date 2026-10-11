#!/bin/bash
# เฝ้าการวิ่งจริง: รอ START แล้วตามจนจบ บันทึกตำแหน่งหุ่นทุก 1 วิ
source /opt/ros/jazzy/setup.bash
M=$(cat $HOME/nav2_run_mark)
OUT=$HOME/run_watch.txt; : > $OUT
python3 - <<'PY' > $HOME/run_track.txt 2>/dev/null &
import time, rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener
import math
rclpy.init(); n = Node('run_track'); b = Buffer(); TransformListener(b, n)
t0 = time.time(); last = 0
while time.time() - t0 < 900:
    rclpy.spin_once(n, timeout_sec=0.1)
    if time.time() - last >= 1.0:
        last = time.time()
        try:
            tf = b.lookup_transform('map', 'base_link', Time()); t = tf.transform
            q = t.rotation; yaw = math.degrees(math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))
            print(f'{time.strftime("%H:%M:%S")} x={t.translation.x:.3f} y={t.translation.y:.3f} yaw={yaw:.1f}', flush=True)
        except Exception: pass
PY
TRACK=$!
for i in $(seq 1 880); do
  if tail -n +$M /tmp/nav2.log | grep -q "เริ่มวิ่ง"; then break; fi
  sleep 1
done
echo "$(date +%T) เห็น START" >> $OUT
for i in $(seq 1 600); do
  if tail -n +$M /tmp/nav2.log | grep -qE "ครบแล้ว|ยกเลิกแล้ว|ถูกยกเลิก|หยุดการวิ่ง"; then sleep 2; break; fi
  sleep 1
done
kill $TRACK 2>/dev/null
tail -n +$M /tmp/nav2.log | grep -E "waypoint_manager\]|collision_monitor\].*(Robot|stop|Stop)|\[ERROR\]|Aborting|failed|Failed to make progress" | grep -vE "max_points|RTPS" | sed "s/\x1b\[[0-9;]*m//g" | cut -c1-190 >> $OUT
echo "=== ตำแหน่ง (ต้น/ท้าย) ===" >> $OUT
head -2 $HOME/run_track.txt >> $OUT; echo ... >> $OUT; tail -3 $HOME/run_track.txt >> $OUT
