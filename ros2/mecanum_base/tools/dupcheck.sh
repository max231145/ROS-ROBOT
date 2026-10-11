#!/bin/bash
# นับ process จริง (ไม่นับ shell ตัวเอง)
ps -eo pid,ppid,lstart,args --no-headers > /tmp/ps_now.txt
for n in ydlidar_ros2_driver_node scan_resampler.py rf2o_laser_odometry_node ekf_node async_slam_toolbox_node foxglove_bridge/foxglove_bridge "camera_ros/camera_node" yolo_node.py map_image_node.py odom_covariance_relay mecanum_base_node; do
  grep -F "$n" /tmp/ps_now.txt | grep -v -e "grep" -e "dupcheck" -e "ros2 launch" -e "ros2 run" > /tmp/ps_sel.txt
  c=$(wc -l < /tmp/ps_sel.txt)
  printf "%-32s %d ตัว  เริ่ม: %s\n" "$n" "$c" "$(awk '{print $6}' /tmp/ps_sel.txt | tr '\n' ' ')"
done
echo "=== start-slam / systemd:"
journalctl -u ros2-slam -b --no-pager 2>/dev/null | grep -E "Starting|Started|Stopping|Stopped|start-slam" | tail -8
echo "(journal จบ)"
last -x reboot 2>/dev/null | head -3
