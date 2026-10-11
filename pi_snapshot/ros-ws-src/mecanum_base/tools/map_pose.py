"""พิมพ์ตำแหน่งหุ่นในแมพตอนนี้ "x y theta" (ใช้กับ save_room.sh / load_room.sh)"""
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener
rclpy.init()
n = Node('map_pose_probe')
buf = Buffer()
TransformListener(buf, n)
t0 = time.time()
while time.time() - t0 < 6:
    rclpy.spin_once(n, timeout_sec=0.2)
    if buf.can_transform('map', 'base_link', Time()):
        tf = buf.lookup_transform('map', 'base_link', Time()).transform
        q = tf.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        print(f'{tf.translation.x:.3f} {tf.translation.y:.3f} {yaw:.4f}')
        break
else:
    raise SystemExit('หา TF map->base_link ไม่ได้')
