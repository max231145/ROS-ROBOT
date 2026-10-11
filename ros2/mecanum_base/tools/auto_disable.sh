#!/bin/bash
# ตัดไฟมอเตอร์อัตโนมัติหลัง 15 นาที กันลืมเปิดค้าง
echo $$ > $HOME/auto_disable.pid
sleep 900
source /opt/ros/jazzy/setup.bash
timeout 15 ros2 service call /mecanum_base/enable std_srvs/srv/SetBool "{data: false}" > $HOME/auto_disable.log 2>&1
echo "$(date +%T) auto-disabled" >> $HOME/auto_disable.log
rm -f $HOME/auto_disable.pid
