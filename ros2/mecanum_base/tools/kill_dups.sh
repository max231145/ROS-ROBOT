#!/bin/bash
# ปิดตัวซ้ำ: เก็บตัวที่เริ่มก่อน ปิดตัวที่เริ่มทีหลัง (ระบุ PID ไม่ใช้ pkill -f)
ps -eo pid,ppid,etimes,args --no-headers > /tmp/ps_now.txt
for pat in "yolo_node.py" "map_image_node.py" "ros2 launch mecanum_base mecanum_base.launch.py"; do
  grep -F "$pat" /tmp/ps_now.txt | grep -v -e grep -e kill_dups > /tmp/ps_sel.txt
  n=$(wc -l < /tmp/ps_sel.txt)
  if [ "$n" -le 1 ]; then echo "$pat: $n ตัว ไม่ต้องทำอะไร"; continue; fi
  # etimes มากสุด = เริ่มก่อน -> เก็บไว้
  keep=$(sort -k3 -n -r /tmp/ps_sel.txt | head -1 | awk '{print $1}')
  for p in $(awk '{print $1}' /tmp/ps_sel.txt); do
    [ "$p" = "$keep" ] && continue
    kill -INT "$p" && echo "$pat: ปิด PID $p (เก็บ $keep)"
  done
done
sleep 4
ps -eo pid,args --no-headers > /tmp/ps_now.txt
for pat in "yolo_node.py" "map_image_node.py" "mecanum_base_node --ros"; do
  echo "เหลือ $pat: $(grep -F "$pat" /tmp/ps_now.txt | grep -vc -e grep -e kill_dups) ตัว"
done
