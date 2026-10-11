#!/usr/bin/env python3
"""admin_web -- เว็บหลังบ้าน: ภาพกล้องสด + ขับด้วย W A S D + ปุ่มหยุดฉุกเฉิน

    python3 admin_web.py            แล้วเปิด http://udru-desktop.local:8092

- W เดินหน้า  S ถอย  A หมุนซ้าย  D หมุนขวา  Space = หยุดฉุกเฉิน (ปิดไดรเวอร์)
- เว็บส่งคำสั่งทุก 0.1 s ระหว่างกดปุ่ม ถ้าไม่ได้รับคำสั่งเกิน 0.4 s (ปล่อยปุ่ม/ปิดแท็บ/เน็ตหลุด) รถหยุดเอง
- ฐานนี้เลื่อนข้างไม่ได้ (DM556 หนึ่งตัวขับล้อหน้า+หลังฝั่งเดียวกัน)
- อย่าขับพร้อมกับตอนที่ Nav2 กำลังวิ่งเป้า (คำสั่งจะชนกัน)
- 7 ต.ค.: แมพสด + บันทึกเส้นทาง (จุดทุก 0.3 m -> ~/maps/routes/<ชื่อ>.json)
  + ตั้งชื่อจุด/ห้อง (~/maps/places.json) + ปุ่มเซฟแมพ (save_room.sh <ชื่อ>)
"""
import base64
import json
import math
import os
import re
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import CompressedImage, LaserScan
from rclpy.qos import qos_profile_sensor_data
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_srvs.srv import SetBool
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
from tf2_ros import Buffer, TransformException, TransformListener

import heapq

from nav2_msgs.action import FollowPath, NavigateThroughPoses
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient

MAPS = os.path.expanduser('~/maps')
ROUTES = os.path.join(MAPS, 'routes')          # ROUTES/<แมพ>/<เส้นทาง>.json
LEGACY_PLACES = os.path.join(MAPS, 'places.json')
ACTIVE = os.path.join(MAPS, 'active_map.txt')  # ชื่อแมพที่เส้นทาง/จุดผูกอยู่
SAVE_ROOM = os.path.expanduser('~/ros-ws/src/mecanum_base/tools/save_room.sh')
SLAM_CFG = os.path.expanduser('~/ros-ws/src/my_slam_config/config/slam_params.yaml')
WAYPOINT_GAP = 0.5  # m ระยะห่างจุดที่ส่งให้ Nav2 ตอนวิ่งตามเส้นทาง
# กลับรถไม่ได้ -> ถอยย้อนรอยที่วิ่งมาจนถึงที่โล่งในแมพ แล้วค่อยให้ Nav2 กลับรถ (7 ต.ค. ผู้ใช้กำหนด)
TRAIL_STEP = 0.10   # m เก็บรอยทุก 10 cm
BACK_VX = 0.12      # m/s ความเร็วถอย (8 ต.ค. 0.08 -> 0.12)
OPEN_R = 0.50       # m รัศมีที่ต้องโล่งถึงจะหมุนได้ (มุมหุ่นกวาด 0.36 + เผื่อ 0.14)
LEG_RETRY = 2       # ถอยแล้วลองใหม่ได้กี่ครั้งต่อจุด
# กลับรถแบบรถยนต์ (เดินหน้า-ถอยหลังสลับกันพร้อมเลี้ยว) เมื่อหมุนอยู่กับที่ไม่พ้น (7 ต.ค. ผู้ใช้สั่ง)
FOOT = (0.27, -0.24, 0.244)   # ขอบหน้า, ขอบหลัง, ครึ่งกว้าง (ตรง nav2_params footprint)
KT_MARGIN = 0.03    # m ระยะเผื่อรอบตัวตอนจำลองท่า
KT_VX = 0.10        # m/s ความเร็วเดินหน้า/ถอยตอนกลับรถ (8 ต.ค. 0.06 -> 0.10)
KT_WZ = 0.35        # rad/s ความเร็วเลี้ยวตอนกลับรถ
KT_SEG = 0.35       # m เดินหน้าหรือถอยได้ไกลสุดต่อช่วงก่อนสลับทิศ
KT_DONE = 0.30      # rad (~17 องศา) หันตรงพอแล้ว ให้ Nav2 ไปต่อ
CORNER_DEG = 30     # องศา ทางหักเกินนี้ = มุม ให้หยุดหมุนอยู่กับที่ตรงมุม แทนการเลี้ยวตัดโค้ง (8 ต.ค. ผู้ใช้สั่ง)
CORNER_MERGE = 0.50  # m มุมที่ห่างกันน้อยกว่านี้ (ทางตัว S) รวมเป็นหมุนครั้งเดียว (11 ต.ค.)
ROT_TOL_DEG = 3.0   # องศา หมุนนุ่มถือว่าถึงทิศแล้ว
# แบบเลี้ยวที่ปรับได้บนเว็บ (11 ต.ค. ผู้ใช้: ทำหลายแบบแล้วทดลองทีละอัน)
TURN_DEFAULT = {'mode': 'smooth', 'corner_deg': 60.0, 'rot_vmax': 0.5, 'rot_accel': 0.8, 'arc_r': 0.35}
ROT_VMAX_MAX = 1.0  # rad/s สูงสุดของสไลเดอร์ความเร็วหมุน (ฐานขับต้องตั้ง max_wz ให้ถึงด้วย)
CORNER_WIN = 0.40   # m ระยะที่ใช้ดูทิศก่อน/หลังมุม (กันรอยส่ายเล็กๆ นับเป็นมุม)
# เดินตามทางที่เคยขับ (7 ต.ค. ผู้ใช้สั่ง: ทางเดินให้ตามเส้นที่วิ่งมาแล้ว ติดที่จุดจอดให้เลื่อนจุดเอง)
DRIVEN_STEP = 0.20  # m เก็บรอยที่ขับทุก 20 cm (เก็บถาวรต่อแมพ ~/maps/routes/<แมพ>/_driven.json)
DRIVEN_MAX = 6000   # จุด (~1.2 km ล่าสุด)
LINK_R = 0.35       # m รอยสองเส้นที่ห่างกันไม่เกินนี้ (และไม่มีผนังคั่น) ถือว่าต่อกัน
SNAP_R = 1.0        # m หุ่น/จุดหมายต้องอยู่ใกล้รอยไม่เกินนี้ถึงจะเดินตามรอย
GOAL_CLEAR = 0.35   # m จุดหมายต้องห่างสิ่งกีดขวางในแมพเท่านี้ ไม่งั้นเลื่อนจุด
NUDGE_MAX = 1.0     # m เลื่อนจุดหมายได้ไกลสุด
# 8 ต.ค. ผู้ใช้: หาทางเองได้แต่ต้องอยู่ในแถบรอบเส้นที่เคยขับ (นอกแถบอาจมีขาโต๊ะ/คานที่ lidar ไม่เห็น)
BAND_R = 0.40       # m ครึ่งความกว้างแถบรอบรอยที่ขับ
BODY_R = 0.27       # m กลางหุ่นต้องห่างของ (แมพ + lidar ตอนนี้) อย่างน้อยเท่านี้
PREFER_R = 0.55     # m ห่างของน้อยกว่านี้ทางจะแพงขึ้น -> เลือกเดินกลางทาง
CENTER_W = 1.5      # น้ำหนักให้อยู่ใกล้เส้นที่ขับ (ยิ่งมากยิ่งชิดเส้นเดิม)
STALL_S = 15.0      # s Nav2 ไม่ขยับหุ่นเกินนี้ = ยกเลิก (ไม่ปล่อยหมุนวนช่วยเหลือ)
STEP = 0.3         # m ระยะห่างจุดบนเส้นทางที่บันทึก
PORT = 8092
MAX_VX = 0.50      # m/s -- 8 ต.ค. ผู้ใช้: 0.15 -> 0.30 -> 0.50 (ปลดเพื่อทดลองหาความเร็วที่เหมาะ)
MISSION_VMAX = 0.50  # m/s สูงสุดของสไลเดอร์ความเร็วภารกิจ (ต้องไม่เกิน max_vx ใน mecanum_base.yaml)
SETTINGS = os.path.join(MAPS, 'admin_settings.json')
MAX_WZ = 0.5       # rad/s
TIMEOUT = 0.4      # s ไม่มีคำสั่งเกินนี้ = หยุด

PAGE = """<!doctype html><html lang="th"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>หุ่นยนต์ - หลังบ้าน</title>
<style>
body{margin:0;background:#16171b;color:#e8e9ec;font-family:"Segoe UI","Noto Sans Thai",sans-serif}
header{padding:10px 14px;background:#1e2026;display:flex;gap:12px;align-items:center;flex-wrap:wrap}
h1{font-size:16px;margin:0}
#st{font-size:13px;color:#9aa0ab}
main{display:flex;gap:14px;padding:14px;flex-wrap:wrap;align-items:flex-start}
#cam{max-width:100%;width:480px;border-radius:8px;background:#000}
#map{background:#2a2c33;border-radius:8px;max-width:100%;touch-action:none}
.box{display:flex;flex-direction:column;gap:8px;width:300px}
.box input[type=text]{height:36px;border-radius:6px;border:1px solid #2e3138;background:#1e2026;color:#e8e9ec;padding:0 8px;font-size:15px}
.box button{height:42px;font-size:15px}
#rec.on{background:#c62828}
#info{font-size:13px;color:#9aa0ab;white-space:pre-line}
.pad{display:grid;grid-template-columns:repeat(3,72px);gap:8px;align-content:start}
button{height:64px;border:1px solid #2e3138;border-radius:8px;background:#262931;color:#e8e9ec;font-size:18px;cursor:pointer}
button.on{background:#4a9eff;color:#fff}
#stop{grid-column:1/4;background:#c62828;font-size:20px;font-weight:600}
#en{grid-column:1/4;height:44px;font-size:15px}
.speed{grid-column:1/4;font-size:13px;color:#9aa0ab}
.dim{font-size:13px;color:#9aa0ab;font-weight:normal}
.list{display:flex;flex-direction:column;gap:4px;max-height:200px;overflow-y:auto}
.row{display:flex;gap:4px;align-items:center;background:#1e2026;border-radius:6px;padding:4px 6px;font-size:14px}
.row span{flex:1;overflow:hidden;text-overflow:ellipsis;white-space:nowrap}
.row button{height:30px;font-size:13px;padding:0 8px}
.box button.red{background:#c62828}
.box button.green{background:#2e7d32}
.cur{outline:2px solid #3ecf6b}
</style></head><body>
<header><h1>หุ่นยนต์ - หลังบ้าน</h1><span id="st">กำลังเชื่อมต่อ...</span></header>
<main>
<img id="cam" alt="กล้อง">
<div class="pad">
 <span></span><button data-k="w">W<br>หน้า</button><span></span>
 <button data-k="a">A<br>ซ้าย</button><button data-k="s">S<br>ถอย</button><button data-k="d">D<br>ขวา</button>
 <button id="stop">หยุด (Space)</button>
 <button id="en">เปิดไดรเวอร์</button>
 <div class="speed">ความเร็ว <input id="sp" type="range" min="20" max="100" value="60"> <span id="spv">60</span>%</div>
</div>
<div><canvas id="map" width="480" height="480"></canvas><div style="display:flex;gap:6px;margin-top:6px"><button id="mrot" style="height:36px;font-size:13px;flex:1">หมุนตามหุ่น / ทิศเหนือขึ้น</button><button id="mzin" style="height:36px;width:48px">+</button><button id="mzout" style="height:36px;width:48px">−</button></div></div>
<div class="box">
 <b>เส้นทาง</b>
 <input id="rname" type="text" placeholder="ชื่อเส้นทาง เช่น main" value="main">
 <button id="rec">เริ่มบันทึกเส้นทาง</button>
 <b>จุด / ห้อง</b>
 <input id="pname" type="text" placeholder="ชื่อห้อง เช่น ห้อง 1, บ้าน">
 <button id="place">บันทึกจุดนี้</button>
 <b>แมพ</b>
 <input id="mname" type="text" value="room_2">
 <button id="save">เซฟแมพ (ตั้งจุดนี้เป็นจุดจอด)</button>
 <button id="setstart">ตั้งตำแหน่งตอนนี้เป็นจุดเริ่ม (ไม่เซฟแมพ)</button>
 <div id="info"></div>
 <b>ประวัติภารกิจ</b>
 <div id="hist" class="dim" style="white-space:pre-line;font-family:monospace;font-size:12px"></div>
</div>
<div class="box">
 <b>ภารกิจ <span id="mst" class="dim"></span></b>
 <div class="dim">ความเร็วภารกิจ <input id="msp" type="range" min="0.05" max="0.50" step="0.01" value="0.15"> <span id="mspv">0.15</span> m/s</div>
 <div class="dim">แบบเลี้ยว <select id="tmode"><option value="smooth">ก. หมุนนุ่ม (เว็บคุม)</option><option value="nav2">ข. Nav2 หมุนเอง</option><option value="arc">ค. เลี้ยวโค้งไม่หยุด</option></select></div>
 <div class="dim">หยุดหมุนเมื่อทางหักเกิน <input class="tc" data-tk="corner_deg" type="range" min="20" max="120" step="5"> <span data-v="corner_deg"></span>°</div>
 <div class="dim">ความเร็วหมุน <input class="tc" data-tk="rot_vmax" type="range" min="0.2" max="1.0" step="0.05"> <span data-v="rot_vmax"></span> rad/s</div>
 <div class="dim">ความเร่งหมุน (แบบ ก.) <input class="tc" data-tk="rot_accel" type="range" min="0.2" max="3.0" step="0.1"> <span data-v="rot_accel"></span> rad/s²</div>
 <div class="dim">รัศมีโค้ง (แบบ ค.) <input class="tc" data-tk="arc_r" type="range" min="0.15" max="0.6" step="0.05"> <span data-v="arc_r"></span> m</div>
 <button id="cancel" class="red">ยกเลิกภารกิจ</button>
 <b>เส้นทางที่บันทึกไว้</b>
 <div id="routes" class="list"></div>
 <b>จุดที่บันทึกไว้ (กด + เพื่อใส่คิว)</b>
 <div id="places" class="list"></div>
 <b>คิว (เรียงลำดับที่จะไป)</b>
 <div id="queue" class="list"></div>
 <label class="dim"><input id="home" type="checkbox" checked> กลับ "บ้าน" เมื่อเสร็จ</label>
 <button id="go" class="green">เริ่มไปตามคิว</button>
 <b>แมพทั้งหมด <span id="mapcount" class="dim"></span></b>
 <div id="maps" class="list"></div>
 <button id="newmap">เริ่มแมพใหม่</button>
 <b>จุดเริ่มของ SLAM <span class="dim">(ลูกศรเหลืองบนแมพ)</span></b>
 <div id="starts" class="list"></div>
</div>
</main>
<script>
const cam=document.getElementById('cam');
// 11 ต.ค. ผู้ใช้: ใช้ภาพ YOLO (JPEG ~3 ภาพ/วิ จาก /detections/image/compressed) แทนกล้องดิบ ลด CPU -- เว็บส่งต่อเลย ไม่เข้ารหัสใหม่
cam.src='/cam.mjpg';
const keys=new Set(); let enabled=false;
const st=document.getElementById('st'), en=document.getElementById('en'), sp=document.getElementById('sp');
sp.oninput=()=>document.getElementById('spv').textContent=sp.value;
function mark(){document.querySelectorAll('[data-k]').forEach(b=>b.classList.toggle('on',keys.has(b.dataset.k)))}
async function post(path,body){try{const r=await fetch(path,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body||{})});return await r.json()}catch(e){st.textContent='เชื่อมต่อไม่ได้';return null}}
async function setEnable(on){const r=await post('/enable',{on});if(r){enabled=r.enabled;en.textContent=enabled?'ปิดไดรเวอร์':'เปิดไดรเวอร์';en.classList.toggle('on',enabled);st.textContent=r.msg}}
async function stop(){keys.clear();mark();await post('/drive',{vx:0,wz:0});await setEnable(false)}
document.getElementById('stop').onclick=stop; en.onclick=()=>setEnable(!enabled);
// ใช้ e.code (ตำแหน่งปุ่ม) ไม่ใช่ e.key -- แป้นภาษาไทย W/A/S/D จะได้ ไ/ฟ/ห/ก แต่ e.code ยังเป็น KeyW/KeyA/KeyS/KeyD
const CODE={KeyW:'w',KeyA:'a',KeyS:'s',KeyD:'d'};
const typing=e=>e.target&&(e.target.tagName==='INPUT'||e.target.tagName==='TEXTAREA');
document.addEventListener('keydown',e=>{
 if(typing(e)){if(e.key==='Enter'||e.key==='Escape')e.target.blur();return}
 if(e.code==='Space'){e.preventDefault();stop();return}
 const k=CODE[e.code];if(k){e.preventDefault();keys.add(k);mark()}});
document.addEventListener('keyup',e=>{const k=CODE[e.code];if(k){keys.delete(k);mark()}});
// กดปุ่มบนจอเสร็จแล้วคืนโฟกัส ไม่ให้ปุ่มหรือช่องกรอกค้างโฟกัส (Space จะไปกดปุ่มซ้ำ)
document.addEventListener('click',e=>{if(e.target.tagName==='BUTTON')setTimeout(()=>e.target.blur(),0)});
window.addEventListener('blur',()=>{keys.clear();mark()});
document.querySelectorAll('[data-k]').forEach(b=>{const k=b.dataset.k;
 b.addEventListener('pointerdown',e=>{e.preventDefault();keys.add(k);mark()});
 ['pointerup','pointerleave','pointercancel'].forEach(t=>b.addEventListener(t,()=>{keys.delete(k);mark()}))});
setInterval(()=>{if(!keys.size)return;const f=sp.value/100;
 let vx=(keys.has('w')?1:0)-(keys.has('s')?1:0), wz=(keys.has('a')?1:0)-(keys.has('d')?1:0);
 post('/drive',{vx:vx*f,wz:wz*f}).then(r=>{if(r)st.textContent=r.msg})},100);
// ---- แมพ + เส้นทาง + จุด ----
const cv=document.getElementById('map'), cx=cv.getContext('2d'), info=document.getElementById('info');
let M=null, mapImg=null, S=null, recording=false;
// ---- แมพแบบเกม (11 ต.ค. ผู้ใช้): มินิแมพหมุนตามหุ่น + หน้าตาแบบเกม ----
let mapRot=true, mapZoom=45;            // หมุนตามหุ่น / พิกเซลต่อเมตร
try{const sv=JSON.parse(localStorage.getItem('mapview')||'{}');if(sv.rot!==undefined)mapRot=sv.rot;if(sv.zoom)mapZoom=sv.zoom}catch(e){}
function saveView(){try{localStorage.setItem('mapview',JSON.stringify({rot:mapRot,zoom:mapZoom}))}catch(e){}}
async function loadMap(){try{const m=await (await fetch('/map')).json();if(!m.w)return;M=m;
 const raw=atob(m.data), img=cx.createImageData(m.w,m.h);
 for(let j=0;j<m.h;j++)for(let i=0;i<m.w;i++){const v=raw.charCodeAt(j*m.w+i), k=((m.h-1-j)*m.w+i)*4;
  const c=v===2?[90,220,255,255]:v===1?[28,52,66,255]:[0,0,0,0];
  img.data[k]=c[0];img.data[k+1]=c[1];img.data[k+2]=c[2];img.data[k+3]=c[3]}
 mapImg=await createImageBitmap(img)}catch(e){}}
function view(){const W=cv.width,H=cv.height,c={x:W/2,y:H*0.62};
 const r=S&&S.pose?S.pose:[0,0,Math.PI/2];const th=mapRot?Math.PI/2-r[2]:0;
 return{c,th,rx:r[0],ry:r[1],z:mapZoom,cos:Math.cos(th),sin:Math.sin(th)}}
function P(V,x,y){const dx=x-V.rx,dy=y-V.ry;return[V.c.x+V.z*(dx*V.cos-dy*V.sin),V.c.y-V.z*(dx*V.sin+dy*V.cos)]}
function line(V,pts,col,w,dash){if(!pts||pts.length<2)return;cx.strokeStyle=col;cx.lineWidth=w;cx.setLineDash(dash||[]);cx.beginPath();let pen=false;
 pts.forEach(p=>{if(!p){pen=false;return}const q=P(V,p[0],p[1]);pen?cx.lineTo(q[0],q[1]):cx.moveTo(q[0],q[1]);pen=true});cx.stroke();cx.setLineDash([])}
function label(t,x,y,col){cx.font='bold 12px sans-serif';const w=cx.measureText(t).width;cx.fillStyle='rgba(0,0,0,0.6)';
 cx.fillRect(x-w/2-4,y-14,w+8,16);cx.fillStyle=col;cx.textAlign='center';cx.fillText(t,x,y-2);cx.textAlign='left'}
function draw(){const W=cv.width,H=cv.height;
 const g=cx.createRadialGradient(W/2,H/2,20,W/2,H/2,W*0.75);g.addColorStop(0,'#0d1a24');g.addColorStop(1,'#05090d');
 cx.setTransform(1,0,0,1,0,0);cx.fillStyle=g;cx.fillRect(0,0,W,H);if(!M||!mapImg)return;
 const V=view(),k=V.z*M.res,dx0=M.ox-V.rx,dy0=M.oy+M.h*M.res-V.ry;
 // แมพ: setTransform ให้พิกเซลภาพตรงกับพิกัดจอ (หมุนตามหุ่น)
 cx.imageSmoothingEnabled=false;
 cx.setTransform(k*V.cos,-k*V.sin,k*V.sin,k*V.cos,V.c.x+V.z*(dx0*V.cos-dy0*V.sin),V.c.y-V.z*(dx0*V.sin+dy0*V.cos));
 cx.drawImage(mapImg,0,0);cx.setTransform(1,0,0,1,0,0);
 // ตารางทุก 1 m จางๆ
 cx.strokeStyle='rgba(90,220,255,0.06)';cx.lineWidth=1;const R=Math.hypot(W,H)/V.z;
 for(let gx=Math.floor(V.rx-R);gx<=V.rx+R;gx++){const a=P(V,gx,V.ry-R),b=P(V,gx,V.ry+R);cx.beginPath();cx.moveTo(a[0],a[1]);cx.lineTo(b[0],b[1]);cx.stroke()}
 for(let gy=Math.floor(V.ry-R);gy<=V.ry+R;gy++){const a=P(V,V.rx-R,gy),b=P(V,V.rx+R,gy);cx.beginPath();cx.moveTo(a[0],a[1]);cx.lineTo(b[0],b[1]);cx.stroke()}
 if(!S)return;
 if(L&&L.driven)line(V,L.driven,'rgba(120,190,255,0.45)',3);
 if(S.leg&&S.leg.length){line(V,S.leg,'rgba(62,207,107,0.35)',9);line(V,S.leg,'#3ecf6b',3,[10,8])}
 line(V,S.path,'#ff9800',3);
 if(L&&L.start)L.start.starts.forEach(p=>{const q=P(V,p.x,p.y),on=p.name===L.start.active,a=p.yaw+V.th;
  cx.strokeStyle=on?'#ffd54a':'rgba(255,213,74,0.45)';cx.lineWidth=2;cx.beginPath();cx.arc(q[0],q[1],on?8:6,0,7);cx.stroke();
  cx.beginPath();cx.moveTo(q[0],q[1]);cx.lineTo(q[0]+16*Math.cos(a),q[1]-16*Math.sin(a));cx.stroke();if(on)label('เริ่ม',q[0],q[1]+24,'#ffd54a')});
 (S.places||[]).forEach(p=>{const q=P(V,p.x,p.y);  // หมุดแบบเกม
  cx.fillStyle='rgba(74,158,255,0.25)';cx.beginPath();cx.ellipse(q[0],q[1],9,4,0,0,7);cx.fill();
  cx.fillStyle='#4a9eff';cx.beginPath();cx.moveTo(q[0],q[1]);cx.bezierCurveTo(q[0]-11,q[1]-14,q[0]-8,q[1]-26,q[0],q[1]-26);
  cx.bezierCurveTo(q[0]+8,q[1]-26,q[0]+11,q[1]-14,q[0],q[1]);cx.fill();cx.fillStyle='#fff';cx.beginPath();cx.arc(q[0],q[1]-17,4,0,7);cx.fill();
  label(p.name,q[0],q[1]-30,'#cfe6ff')});
 if(S.pose){const q=P(V,S.pose[0],S.pose[1]),a=S.pose[2]+V.th,ca=Math.cos(a),sa=Math.sin(a);
  // วงเรดาร์ + ตัวหุ่นเป็นลูกศรแบบเกม (ขนาดจริง ~0.5 m)
  const t=(Date.now()%2000)/2000;cx.strokeStyle='rgba(62,207,107,'+(0.5*(1-t))+')';cx.lineWidth=2;cx.beginPath();cx.arc(q[0],q[1],10+40*t,0,7);cx.stroke();
  const L1=Math.max(12,0.30*V.z),B1=Math.max(9,0.25*V.z),f=(u,v)=>[q[0]+u*ca+v*sa,q[1]-u*sa+v*ca];
  const p1=f(L1,0),p2=f(-L1*0.7,B1),p3=f(-L1*0.35,0),p4=f(-L1*0.7,-B1);
  cx.fillStyle='#3ecf6b';cx.strokeStyle='#d8ffe6';cx.lineWidth=2;cx.beginPath();cx.moveTo(p1[0],p1[1]);cx.lineTo(p2[0],p2[1]);cx.lineTo(p3[0],p3[1]);cx.lineTo(p4[0],p4[1]);cx.closePath();cx.fill();cx.stroke()}
 // เข็มทิศ: ทิศ +y ของแมพ
 const nx=W-26,ny=26,na=Math.PI/2+V.th;cx.fillStyle='rgba(0,0,0,0.5)';cx.beginPath();cx.arc(nx,ny,16,0,7);cx.fill();
 cx.strokeStyle='#ff5b5b';cx.lineWidth=3;cx.beginPath();cx.moveTo(nx,ny);cx.lineTo(nx+12*Math.cos(na),ny-12*Math.sin(na));cx.stroke();
 cx.fillStyle='#fff';cx.font='bold 10px sans-serif';cx.fillText('N',nx-3,ny+4);
 cx.fillStyle='rgba(207,230,255,0.7)';cx.font='11px sans-serif';cx.fillText((mapRot?'หมุนตามหุ่น':'ทิศเหนือขึ้น')+' · 1 ช่อง = 1 m',8,H-8)}
setInterval(()=>{if(S)draw()},100);   // วาดลื่นๆ 10 ภาพ/วิ (เรดาร์กะพริบ)
cv.addEventListener('wheel',e=>{e.preventDefault();mapZoom=Math.max(15,Math.min(160,mapZoom*(e.deltaY<0?1.15:1/1.15)));saveView();draw()},{passive:false});
async function poll(){try{S=await (await fetch('/status')).json();recording=S.recording;
 const rb=document.getElementById('rec');rb.textContent=recording?'หยุดและบันทึกเส้นทาง':'เริ่มบันทึกเส้นทาง';rb.classList.toggle('on',recording);
 info.textContent=(S.pose?`ตำแหน่ง x=${S.pose[0].toFixed(2)} y=${S.pose[1].toFixed(2)}\n`:'ยังไม่รู้ตำแหน่ง\\n')+(recording?`กำลังบันทึก "${S.route}" ${S.path.length} จุด ${S.length.toFixed(1)} m\n`:'')+`จุดที่ตั้งชื่อ ${S.places.length} จุด`+(S.msg?`\n${S.msg}`:'');document.getElementById('hist').textContent=(S.history||[]).join('\\n');draw()}catch(e){}}
document.getElementById('rec').onclick=async()=>{const r=await post('/record',{on:!recording,name:document.getElementById('rname').value});if(r)st.textContent=r.msg;poll()};
document.getElementById('place').onclick=async()=>{const n=document.getElementById('pname').value.trim();if(!n){alert('พิมพ์ชื่อจุดก่อน');return}const r=await post('/place',{name:n});if(r)st.textContent=r.msg;poll()};
document.getElementById('setstart').onclick=async()=>{if(!S||!S.pose){alert('ยังไม่รู้ตำแหน่งหุ่น');return}
 const n=prompt('ตั้งตำแหน่งที่หุ่นอยู่ตอนนี้ (x='+S.pose[0].toFixed(2)+' y='+S.pose[1].toFixed(2)+' หัน '+Math.round(S.pose[2]*180/Math.PI)+'°) เป็นจุดเริ่มของ SLAM -- ตั้งชื่อจุดเริ่ม (เว้นว่างได้):','');
 if(n===null)return;const r=await act('/setstart',{name:n})};
document.getElementById('save').onclick=async()=>{const n=document.getElementById('mname').value.trim();if(!confirm('เซฟแมพเป็น "'+n+'" และตั้งจุดที่หุ่นอยู่ตอนนี้เป็นจุดจอด?'))return;st.textContent='กำลังเซฟแมพ...';const r=await post('/savemap',{name:n});if(r)st.textContent=r.msg};
// ---- รายการเส้นทาง / จุด / คิว / แมพ ----
let L=null, queue=[];
const el=id=>document.getElementById(id);
const esc=s=>String(s).replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
function row(text,btns,cls){const d=document.createElement('div');d.className='row'+(cls?' '+cls:'');
 d.innerHTML='<span title="'+esc(text)+'">'+esc(text)+'</span>';
 btns.forEach(([t,f])=>{const b=document.createElement('button');b.textContent=t;b.onclick=f;d.appendChild(b)});return d}
async function act(path,body){const r=await post(path,body);if(r)st.textContent=r.msg;poll();loadLists();return r}
function drawQueue(){const q=el('queue');q.innerHTML='';
 if(!queue.length){q.innerHTML='<span class="dim">ยังไม่มีในคิว</span>';return}
 queue.forEach((n,i)=>q.appendChild(row((i+1)+'. '+n,[
  ['▲',()=>{if(i>0){[queue[i-1],queue[i]]=[queue[i],queue[i-1]];drawQueue()}}],
  ['▼',()=>{if(i<queue.length-1){[queue[i+1],queue[i]]=[queue[i],queue[i+1]];drawQueue()}}],
  ['✕',()=>{queue.splice(i,1);drawQueue()}]])))}
async function loadLists(){try{L=await (await fetch('/lists')).json();
 const r=el('routes');r.innerHTML='';
 r.innerHTML='<span class="dim">รอยที่เคยขับ '+L.driven_m+' m (เส้นฟ้าบนแมพ) หุ่นเดินได้เฉพาะบนรอยนี้ · เส้นเขียวประ = ทางที่กำลังไป</span>';
 L.routes.forEach(x=>r.appendChild(row(x.name+' ('+x.length+' m)',[
  ['เดิน',()=>{if(confirm('ให้หุ่นเดินตามเส้นทาง "'+x.name+'" ?'))act('/follow',{name:x.name,reverse:false})}],
  ['ย้อน',()=>{if(confirm('ให้หุ่นเดินย้อนเส้นทาง "'+x.name+'" ?'))act('/follow',{name:x.name,reverse:true})}]])));
 const p=el('places');p.innerHTML='';
 if(!L.places.length)p.innerHTML='<span class="dim">ยังไม่มีจุด</span>';
 L.places.forEach(x=>p.appendChild(row(x.name,[
  ['+',()=>{queue.push(x.name);drawQueue()}],
  ['ลบ',()=>{if(confirm('ลบจุด "'+x.name+'" ?'))act('/delplace',{name:x.name})}]])));
 queue=queue.filter(n=>L.places.some(x=>x.name===n));drawQueue();
 if(!tcInit&&L.turn){showTurn(L.turn);tcInit=true}
 if(!mspInit&&L.mission_speed){msp.max=L.speed_max;msp.value=L.mission_speed;el('mspv').textContent=(+L.mission_speed).toFixed(2);mspInit=true}
 const sl=el('starts');sl.innerHTML='';
 if(!L.start||!L.start.starts.length)sl.innerHTML='<span class="dim">ยังไม่มีจุดเริ่ม (SLAM ต้องโหลดแมพที่เซฟไว้)</span>';
 else L.start.starts.slice().reverse().forEach(x=>sl.appendChild(row(x.name+' ('+x.x.toFixed(2)+', '+x.y.toFixed(2)+') '+Math.round(x.yaw*180/Math.PI)+'°'+(x.name===L.start.active?' · ใช้อยู่':''),[
  ['ใช้',()=>{if(confirm('ใช้ "'+x.name+'" เป็นจุดเริ่ม? ครั้งหน้าวางหุ่นตรงนั้นก่อนเปิด Pi'))act('/usestart',{name:x.name})}],
  ['ลบ',()=>{if(confirm('ลบจุดเริ่ม "'+x.name+'" ?'))act('/delstart',{name:x.name})}]],
  x.name===L.start.active?'cur':'')));
 const m=el('maps');m.innerHTML='';el('mapcount').textContent='('+L.maps.length+' แมพ · ตอนนี้: '+L.map+')';
 if(!L.maps.length)m.innerHTML='<span class="dim">ยังไม่มีแมพที่เซฟ</span>';
 L.maps.forEach(x=>m.appendChild(row(x.name+' · '+x.saved,[
  ['ใช้แมพนี้',()=>{if(confirm('เปลี่ยนไปใช้แมพ "'+x.name+'" ? (ต้อง restart SLAM และ Nav2 เอง)'))act('/usemap',{name:x.name})}]],
  x.name===L.slam_map?'cur':'')))}catch(e){}}
el('go').onclick=()=>{if(!queue.length){alert('ใส่จุดลงคิวก่อน');return}
 if(confirm('เริ่มไป: '+queue.join(' → ')+(el('home').checked?' → บ้าน':'')+' ?'))act('/goplaces',{names:queue,home:el('home').checked})};
el('cancel').onclick=()=>act('/cancel');
el('mrot').onclick=()=>{mapRot=!mapRot;saveView();draw()};el('mzin').onclick=()=>{mapZoom=Math.min(160,mapZoom*1.25);saveView();draw()};el('mzout').onclick=()=>{mapZoom=Math.max(15,mapZoom/1.25);saveView();draw()};
// ความเร็วภารกิจ: ลากแล้วปล่อย = ส่งค่าให้ Nav2 (จำไว้ใช้ครั้งต่อไปด้วย)
let mspInit=false;const msp=el('msp');
msp.oninput=()=>{el('mspv').textContent=(+msp.value).toFixed(2)};
msp.onchange=()=>act('/speed',{v:+msp.value});
// แบบเลี้ยว + ค่าการหมุน (ลากแล้วปล่อย = บันทึก ใช้กับภารกิจถัดไป)
let tcInit=false;
document.querySelectorAll('.tc').forEach(i=>{const k=i.dataset.tk,sp=document.querySelector('[data-v="'+k+'"]');
 i.oninput=()=>{sp.textContent=(+i.value).toFixed(k==='corner_deg'?0:2)};
 i.onchange=()=>{const b={};b[k]=+i.value;act('/turncfg',b)}});
el('tmode').onchange=()=>act('/turncfg',{mode:el('tmode').value});
function showTurn(t){el('tmode').value=t.mode;document.querySelectorAll('.tc').forEach(i=>{const k=i.dataset.tk;i.value=t[k];
 document.querySelector('[data-v="'+k+'"]').textContent=(+t[k]).toFixed(k==='corner_deg'?0:2)})}
el('newmap').onclick=()=>{if(confirm('ตั้งให้ SLAM เริ่มแมพใหม่? (แมพเดิมที่เซฟไว้ยังอยู่)'))act('/usemap',{name:'__new__'})};
setInterval(()=>{if(S)el('mst').textContent=S.mission?'กำลังทำ: '+S.mission:'ว่าง'},500);
loadLists();setInterval(loadLists,5000);
// แมพเวลาจริง: โหลดใหม่ทันทีที่ SLAM ส่งแมพใหม่ (ดูเลขรุ่นแมพจาก /status ทุก 0.5 s)
let mapVer=-1;setInterval(()=>{if(S&&S.map_ver!==undefined&&S.map_ver!==mapVer){mapVer=S.map_ver;loadMap()}},500);
loadMap();setInterval(poll,500);
setInterval(async()=>{try{const r=await (await fetch('/state')).json();enabled=r.enabled;en.textContent=enabled?'ปิดไดรเวอร์':'เปิดไดรเวอร์';en.classList.toggle('on',enabled);if(!keys.size)st.textContent=r.msg}catch(e){st.textContent='เชื่อมต่อไม่ได้'}},1500);
</script></body></html>"""


MISSION_LOG = '/tmp/admin_web_mission.log'


class Admin(Node):
    # ข้อความสถานะ: ทุกครั้งที่เปลี่ยน (ไม่นับตัวเลขที่วิ่ง) เก็บเป็นประวัติ 10 บรรทัด + ลงไฟล์ MISSION_LOG
    @property
    def msg(self):
        return self._msg

    @msg.setter
    def msg(self, v):
        self._msg = v
        key = re.sub(r'[-+]?[\d.]+', '#', v or '')
        if v and key != self._msg_key:
            self._msg_key = key
            self.log(v)

    def log(self, text):
        line = time.strftime('%H:%M:%S ') + text
        self.history.append(line)
        del self.history[:-10]
        try:
            with open(MISSION_LOG, 'a', encoding='utf-8') as fp:
                fp.write(line + '\n')
        except OSError:
            pass

    def __init__(self):
        self._msg, self._msg_key, self.history = '', '', []
        super().__init__('admin_web')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        # ตอนถอยอัตโนมัติ ส่งเข้า collision monitor (input ของมัน) ให้มันเบรกถ้ามีของด้านหลัง
        self.pub_safe = self.create_publisher(Twist, 'cmd_vel_smoothed', 10)
        self.en_cli = self.create_client(SetBool, '/mecanum_base/enable')
        self.lock = threading.Lock()
        self.cmd = (0.0, 0.0)
        self.last = 0.0
        self.moving = False
        self.enabled = False
        self.create_timer(0.05, self.tick)
        # แมพ + ตำแหน่ง + เส้นทาง
        self.tf = Buffer()
        self.tf_listener = TransformListener(self.tf, self)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, qos)
        self.map_json = '{}'
        self.map_ver = 0
        self.jpeg, self.jpeg_n = None, 0          # ภาพ YOLO ล่าสุด (JPEG)
        self.jpeg_cv = threading.Condition()
        self.create_subscription(CompressedImage, '/detections/image/compressed', self.on_jpeg, qos_profile_sensor_data)
        self.pose = None
        self.recording = False
        self.route_name = 'main'
        self.path = []
        self.trail = []          # รอยที่หุ่นวิ่งผ่าน (เก็บตลอด) ใช้ถอยย้อนกลับ
        self.trail_pause = False
        self.grid = None
        self.scan_pts = []       # จุด lidar ล่าสุดในกรอบ base_link
        self.scan_t = 0.0
        self.create_subscription(LaserScan, '/scan_nav', self.on_scan, qos_profile_sensor_data)
        self.msg = ''
        self.map_name = self.read_active()
        os.makedirs(self.route_dir(), exist_ok=True)
        self.places = self.load_places()
        self.create_timer(0.2, self.track)
        # ภารกิจ: วิ่งตามเส้นทาง / ไปจุดตามคิว
        self.nav_tp = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')
        self.follow_cli = ActionClient(self, FollowPath, 'follow_path')
        self.leg_pts = []                    # ทางที่กำลังเดินอยู่ (วาดบนแมพ)
        self.driven = self.load_driven()     # รอยที่ขับ ทั้งขับเองและภารกิจ (None = รอยขาด)
        self.driven_dirty = 0
        self.mission = None
        self.mission_thread = None
        self.cancel_flag = False
        self.goal_handle = None
        # ความเร็วตอนทำภารกิจ ปรับจากเว็บ (8 ต.ค. ผู้ใช้) -- ตั้งค่าให้ Nav2 ตอนรันได้เลยไม่ต้อง restart
        self.param_cli = {n: self.create_client(SetParameters, f'/{n}/set_parameters')
                          for n in ('controller_server', 'velocity_smoother')}
        self.get_cli = self.create_client(GetParameters, '/velocity_smoother/get_parameters')
        self.mission_speed = self.load_settings().get('mission_speed', 0.15)
        self.turn_cfg = dict(TURN_DEFAULT, **self.load_settings().get('turn', {}))

    # ---------------- แมพที่ใช้ ----------------
    def read_active(self):
        try:
            with open(ACTIVE, encoding='utf-8') as fp:
                n = fp.read().strip()
                if n:
                    return n
        except OSError:
            pass
        m = self.slam_map()
        return m or 'room_2'

    def write_active(self, name):
        if self.driven_dirty:
            self.save_driven()
        self.map_name = name
        with open(ACTIVE, 'w', encoding='utf-8') as fp:
            fp.write(name)
        os.makedirs(self.route_dir(), exist_ok=True)
        self.places = self.load_places()
        self.driven = self.load_driven()

    @staticmethod
    def slam_map():
        """ชื่อแมพที่ SLAM โหลดตอนเปิด (None = เริ่มแมพใหม่)"""
        try:
            for line in open(SLAM_CFG, encoding='utf-8'):
                m = re.match(r'\s*map_file_name:\s*\S*/([^/\s]+)\s*$', line)
                if m:
                    return m.group(1)
        except OSError:
            pass
        return None

    def route_dir(self):
        return os.path.join(ROUTES, self.map_name)

    def driven_file(self):
        return os.path.join(self.route_dir(), '_driven.json')

    def load_driven(self):
        try:
            with open(self.driven_file(), encoding='utf-8') as fp:
                return [tuple(p) if p else None for p in json.load(fp).get('points', [])]
        except (OSError, ValueError):
            return []

    def save_driven(self):
        try:
            os.makedirs(self.route_dir(), exist_ok=True)
            tmp = self.driven_file() + '.tmp'
            with open(tmp, 'w', encoding='utf-8') as fp:
                json.dump({'frame': 'map', 'map': self.map_name, 'step_m': DRIVEN_STEP,
                           'points': [list(p) if p else None for p in self.driven]}, fp)
            os.replace(tmp, self.driven_file())
            self.driven_dirty = 0
        except OSError as e:
            self.get_logger().warn(f'เซฟรอยที่ขับไม่ได้: {e}')

    def driven_len(self):
        return sum(math.hypot(b[0] - a[0], b[1] - a[1])
                   for a, b in zip(self.driven, self.driven[1:]) if a and b)

    def places_file(self):
        return os.path.join(MAPS, f'places_{self.map_name}.json')

    def list_maps(self):
        out = []
        for f in sorted(os.listdir(MAPS)):
            if f.endswith('.posegraph'):
                n = f[:-len('.posegraph')]
                pose = None
                try:
                    pose = [float(v) for v in open(os.path.join(MAPS, n + '.pose')).read().split()[:3]]
                except (OSError, ValueError):
                    pass
                out.append({'name': n, 'pose': pose,
                            'saved': time.strftime('%Y-%m-%d %H:%M', time.localtime(os.path.getmtime(os.path.join(MAPS, f))))})
        return out

    def list_routes(self):
        out = []
        for d in (self.route_dir(), ROUTES):
            if not os.path.isdir(d):
                continue
            for f in sorted(os.listdir(d)):
                if f.endswith('.json') and not f.startswith('_'):
                    pts = self.load_route(os.path.join(d, f))
                    if len(pts) >= 2:
                        n = f[:-5] if d != ROUTES else f[:-5] + ' (เก่า)'
                        out.append({'name': n, 'file': os.path.join(d, f), 'points': len(pts),
                                    'length': round(sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pts, pts[1:])), 1)})
        return out

    @staticmethod
    def load_route(f):
        try:
            with open(f, encoding='utf-8') as fp:
                return [tuple(p) for p in json.load(fp).get('points', [])]
        except (OSError, ValueError):
            return []

    # ---------------- จุดเริ่มของ SLAM (map_start_pose) เก็บเป็นรายการต่อแมพ ----------------
    @staticmethod
    def slam_start_pose():
        """map_start_pose ที่ slam_params ใช้อยู่ตอนนี้ (None = ไม่ได้ตั้ง)"""
        try:
            for line in open(SLAM_CFG, encoding='utf-8'):
                m = re.match(r'\s*map_start_pose:\s*\[([^\]]+)\]', line)
                if m:
                    return [float(v) for v in m.group(1).split(',')[:3]]
        except (OSError, ValueError):
            pass
        return None

    def starts_file(self, slam):
        return os.path.join(MAPS, f'starts_{slam}.json')

    def load_starts(self, slam):
        """รายการจุดเริ่มของแมพ -- ครั้งแรกเอาจุดที่ slam_params ใช้อยู่มาเป็น "จุดเดิม" """
        try:
            with open(self.starts_file(slam), encoding='utf-8') as fp:
                return json.load(fp).get('starts', [])
        except (OSError, ValueError):
            pass
        cur = self.slam_start_pose()
        if not cur:
            return []
        return [{'name': 'จุดเดิม', 'x': cur[0], 'y': cur[1], 'yaw': cur[2], 'saved': ''}]

    def save_starts(self, slam, starts):
        with open(self.starts_file(slam), 'w', encoding='utf-8') as fp:
            json.dump({'map': slam, 'starts': starts}, fp, ensure_ascii=False, indent=1)

    def start_info(self):
        slam = self.slam_map()
        if not slam:
            return {'map': None, 'starts': [], 'active': None}
        cur = self.slam_start_pose()
        starts = self.load_starts(slam)
        active = next((s['name'] for s in starts if cur and math.hypot(s['x'] - cur[0], s['y'] - cur[1]) < 0.01
                       and abs(math.atan2(math.sin(s['yaw'] - cur[2]), math.cos(s['yaw'] - cur[2]))) < 0.01), None)
        return {'map': slam, 'starts': starts, 'active': active}

    def apply_start(self, slam, x, y, th):
        """เขียน map_start_pose ลง slam_params (+ ไฟล์ .pose ของแมพ) มีผลตอนเปิด SLAM ครั้งหน้า"""
        try:
            txt = open(SLAM_CFG, encoding='utf-8').read()
        except OSError as e:
            return f'อ่าน slam_params.yaml ไม่ได้: {e}'
        new, n = re.subn(r'(?m)^(\s*)#?\s*map_start_pose:.*$', rf'\1map_start_pose: [{x:.3f}, {y:.3f}, {th:.4f}]', txt)
        if not n:
            return 'หาบรรทัด map_start_pose ใน slam_params.yaml ไม่เจอ -- ไม่ได้แก้'
        bak = SLAM_CFG + '.bak-setstart'
        if not os.path.exists(bak):
            open(bak, 'w', encoding='utf-8').write(txt)
        open(SLAM_CFG, 'w', encoding='utf-8').write(new)
        try:                                   # ให้ปุ่ม "ใช้แมพนี้" ใช้จุดเริ่มเดียวกัน
            with open(os.path.join(MAPS, slam + '.pose'), 'w') as fp:
                fp.write(f'{x:.3f} {y:.3f} {th:.4f}\n')
        except OSError:
            pass
        return 'ok'

    def set_start(self, name=''):
        """เพิ่มตำแหน่งที่หุ่นอยู่ตอนนี้เข้ารายการจุดเริ่ม แล้วใช้เป็นจุดเริ่มของ SLAM (ไม่เซฟแมพใหม่)"""
        if not self.pose:
            return 'ยังไม่รู้ตำแหน่งหุ่น'
        slam = self.slam_map()
        if not slam:
            return 'SLAM ไม่ได้โหลดแมพที่เซฟไว้ (เริ่มแมพใหม่อยู่) -- เซฟแมพก่อน'
        x, y, th = self.pose
        starts = self.load_starts(slam)
        name = (name or '').strip()[:30] or time.strftime('จุดเริ่ม %d/%m %H:%M')
        starts = [s for s in starts if s['name'] != name]
        res = self.apply_start(slam, x, y, th)
        if res != 'ok':
            return res
        starts.append({'name': name, 'x': round(x, 3), 'y': round(y, 3), 'yaw': round(th, 4),
                       'saved': time.strftime('%Y-%m-%d %H:%M')})
        self.save_starts(slam, starts)
        self.log(f'ตั้งจุดเริ่ม "{name}" ของ {slam}: ({x:.2f},{y:.2f}) หัน {math.degrees(th):.0f}°')
        return (f'ตั้งจุดเริ่ม "{name}" แล้ว ({x:.2f}, {y:.2f}) หัน {math.degrees(th):.0f}° -- '
                'ครั้งหน้าวางหุ่นตรงนี้ หันทางนี้ ก่อนเปิด Pi')

    def use_start(self, name):
        slam = self.slam_map()
        s = next((s for s in self.load_starts(slam) if s['name'] == name), None) if slam else None
        if not s:
            return f'ไม่เจอจุดเริ่ม "{name}"'
        res = self.apply_start(slam, s['x'], s['y'], s['yaw'])
        if res != 'ok':
            return res
        self.log(f'ใช้จุดเริ่ม "{name}" ({s["x"]:.2f},{s["y"]:.2f})')
        return f'ใช้จุดเริ่ม "{name}" แล้ว -- ครั้งหน้าวางหุ่นตรงนั้นก่อนเปิด Pi'

    def del_start(self, name):
        info = self.start_info()
        slam, starts = info['map'], info['starts']
        if not slam:
            return 'SLAM ไม่ได้โหลดแมพที่เซฟไว้'
        if not any(s['name'] == name for s in starts):
            return f'ไม่เจอจุดเริ่ม "{name}"'
        rest = [s for s in starts if s['name'] != name]
        if not rest:
            return 'ลบไม่ได้ -- ต้องเหลือจุดเริ่มอย่างน้อย 1 จุด (ตั้งจุดใหม่ก่อนแล้วค่อยลบ)'
        msg = f'ลบจุดเริ่ม "{name}" แล้ว'
        if info['active'] == name:            # ลบตัวที่ใช้อยู่ -> ใช้ตัวล่าสุดที่เหลือ
            s = rest[-1]
            res = self.apply_start(slam, s['x'], s['y'], s['yaw'])
            if res != 'ok':
                return res
            msg += f' -- เปลี่ยนไปใช้ "{s["name"]}"'
        self.save_starts(slam, rest)
        self.log(msg)
        return msg

    def use_map(self, name):
        if self.mission:
            return 'กำลังทำภารกิจอยู่ -- ยกเลิกก่อน'
        try:
            txt = open(SLAM_CFG, encoding='utf-8').read()
        except OSError as e:
            return f'อ่าน slam_params.yaml ไม่ได้: {e}'
        bak = SLAM_CFG + '.bak-adminweb'
        if not os.path.exists(bak):
            open(bak, 'w', encoding='utf-8').write(txt)
        if name in ('', '__new__'):
            txt = re.sub(r'(?m)^(\s*)#?\s*(map_file_name:.*)$', r'\1# \2', txt)
            txt = re.sub(r'(?m)^(\s*)#?\s*(map_start_pose:.*)$', r'\1# \2', txt)
            open(SLAM_CFG, 'w', encoding='utf-8').write(txt)
            self.write_active(time.strftime('new_%m%d_%H%M'))
            return ('ตั้งให้ SLAM เริ่มแมพใหม่แล้ว -- รัน "sudo systemctl restart ros2-slam" บน Pi '
                    'แล้วรัน restart_nav2.sh (อย่าลืมเซฟแมพเมื่อสำรวจเสร็จ)')
        name = self.safe_name(name, '')
        try:
            x, y, th = open(os.path.join(MAPS, name + '.pose')).read().split()[:3]
        except (OSError, ValueError):
            return f'ไม่เจอไฟล์ {name}.pose -- ใช้แมพนี้ไม่ได้'
        txt, n1 = re.subn(r'(?m)^(\s*)#?\s*map_file_name:.*$', rf'\1map_file_name: {MAPS}/{name}', txt)
        txt, n2 = re.subn(r'(?m)^(\s*)#?\s*map_start_pose:.*$', rf'\1map_start_pose: [{x}, {y}, {th}]', txt)
        if not (n1 and n2):
            return 'หาบรรทัด map_file_name / map_start_pose ใน slam_params.yaml ไม่เจอ -- ไม่ได้แก้'
        open(SLAM_CFG, 'w', encoding='utf-8').write(txt)
        self.write_active(name)
        return (f'ตั้งให้ใช้แมพ "{name}" แล้ว -- วางหุ่นที่จุดจอดของแมพนี้ (x={float(x):.2f} y={float(y):.2f}) '
                'แล้วรัน "sudo systemctl restart ros2-slam" ตามด้วย restart_nav2.sh')

    # ---------------- ภารกิจ ----------------
    def line_free(self, a, b):
        """เส้นตรง a->b ไม่ผ่านช่องที่แมพบอกว่ามีสิ่งกีดขวาง"""
        if self.grid is None:
            return True
        v, w, h, res, ox, oy = self.grid
        n = max(1, int(math.hypot(b[0] - a[0], b[1] - a[1]) / (res * 0.7)))
        for k in range(n + 1):
            x = a[0] + (b[0] - a[0]) * k / n
            y = a[1] + (b[1] - a[1]) * k / n
            i, j = int((x - ox) / res), int((y - oy) / res)
            if 0 <= i < w and 0 <= j < h and v[j * w + i] == 2:
                return False
        return True

    def path_on_driven(self, sx, sy, tx, ty):
        """ทางสั้นสุดจาก (sx,sy) ไป (tx,ty) ที่วิ่งบนรอยที่เคยขับ + เส้นทางที่บันทึกไว้ (None = ไม่มีทางบนรอย)"""
        lines, cur = [], []
        for q in self.driven:
            if q is None:
                if len(cur) > 1:
                    lines.append(cur)
                cur = []
            else:
                cur.append(q)
        if len(cur) > 1:
            lines.append(cur)
        for r in self.list_routes():
            pts = self.load_route(r['file'])
            if len(pts) > 1:
                lines.append(pts)
        nodes, adj = [], []
        for ln in lines:
            for k, q in enumerate(ln):
                nodes.append((q[0], q[1]))
                adj.append([])
                if k:
                    a, b = len(nodes) - 2, len(nodes) - 1
                    d = math.hypot(nodes[a][0] - q[0], nodes[a][1] - q[1])
                    if d < 0.8:
                        adj[a].append((b, d))
                        adj[b].append((a, d))
        if not nodes:
            return None
        # เชื่อมรอยที่ซ้อน/ตัดกัน (ใกล้กันไม่เกิน LINK_R และไม่มีผนังคั่น)
        cell, buckets = LINK_R, {}
        for i, (x, y) in enumerate(nodes):
            buckets.setdefault((int(x // cell), int(y // cell)), []).append(i)
        for (cx, cy), ids in buckets.items():
            near = [j for dx in (-1, 0, 1) for dy in (-1, 0, 1) for j in buckets.get((cx + dx, cy + dy), ())]
            for i in ids:
                for j in near:
                    if j > i + 1:
                        d = math.hypot(nodes[i][0] - nodes[j][0], nodes[i][1] - nodes[j][1])
                        if d <= LINK_R and self.line_free(nodes[i], nodes[j]):
                            adj[i].append((j, d))
                            adj[j].append((i, d))

        def snap(x, y):
            best = None
            for i, (nx, ny) in enumerate(nodes):
                d = math.hypot(nx - x, ny - y)
                if d <= SNAP_R and (best is None or d < best[0]) and self.line_free((x, y), (nx, ny)):
                    best = (d, i)
            return best
        a, b = snap(sx, sy), snap(tx, ty)
        if a is None or b is None:
            return None
        src, dst = a[1], b[1]
        dist, prev, pq = {src: 0.0}, {}, [(0.0, src)]
        while pq:
            d, i = heapq.heappop(pq)
            if i == dst:
                break
            if d > dist.get(i, 1e18):
                continue
            for j, w in adj[i]:
                nd = d + w
                if nd < dist.get(j, 1e18):
                    dist[j], prev[j] = nd, i
                    heapq.heappush(pq, (nd, j))
        if dst not in dist:
            return None
        out, i = [], dst
        while True:
            out.append(nodes[i])
            if i == src:
                break
            i = prev[i]
        out.reverse()
        return [(sx, sy)] + out + [(tx, ty)]

    def nudge_goal(self, tx, ty):
        """จุดหมายชิดสิ่งกีดขวางเกิน -> เลื่อนไปจุดที่ใกล้สุดที่ห่างของ GOAL_CLEAR (ไม่เกิน NUDGE_MAX)"""
        if self.grid is None or self.is_open(tx, ty, GOAL_CLEAR):
            return tx, ty, 0.0
        res = self.grid[3]
        step = max(res * 2, 0.05)
        for ring in range(1, int(NUDGE_MAX / step) + 1):
            r = ring * step
            n = max(8, int(2 * math.pi * r / step))
            cands = []
            for k in range(n):
                x, y = tx + r * math.cos(2 * math.pi * k / n), ty + r * math.sin(2 * math.pi * k / n)
                if self.is_open(x, y, GOAL_CLEAR) and self.line_free((tx, ty), (x, y)):
                    cands.append((math.hypot(x - self.pose[0], y - self.pose[1]), x, y))
            if cands:
                _, x, y = min(cands)               # ระยะเลื่อนเท่ากัน เลือกฝั่งที่ใกล้หุ่น
                return x, y, r
        return tx, ty, -1.0

    # ---------------- ความเร็วภารกิจ ----------------
    @staticmethod
    def load_settings():
        try:
            with open(SETTINGS, encoding='utf-8') as fp:
                return json.load(fp)
        except (OSError, ValueError):
            return {}

    def save_settings(self):
        s = self.load_settings()
        s['mission_speed'] = self.mission_speed
        s['turn'] = self.turn_cfg
        with open(SETTINGS, 'w', encoding='utf-8') as fp:
            json.dump(s, fp, indent=1)

    def call_srv(self, cli, req, timeout=3.0):
        if not cli.wait_for_service(timeout_sec=2.0):
            return None
        box, ev = {}, threading.Event()
        cli.call_async(req).add_done_callback(lambda f: (box.__setitem__('r', f.result()), ev.set()))
        ev.wait(timeout)
        return box.get('r')

    def apply_speed(self, v):
        """ตั้งความเร็ววิ่งของ Nav2 ตอนรัน: RPP desired_linear_vel + velocity_smoother max/min_velocity (แกน x)"""
        v = max(0.05, min(MISSION_VMAX, float(v)))

        def dparam(name, value):
            pv = ParameterValue()
            if isinstance(value, list):
                pv.type, pv.double_array_value = ParameterType.PARAMETER_DOUBLE_ARRAY, [float(x) for x in value]
            else:
                pv.type, pv.double_value = ParameterType.PARAMETER_DOUBLE, float(value)
            return Parameter(name=name, value=pv)
        wz = float(self.turn_cfg['rot_vmax'])
        r = self.call_srv(self.param_cli['controller_server'],
                          SetParameters.Request(parameters=[dparam('FollowPath.desired_linear_vel', v)]))
        if r is None:
            return 'ไม่เจอ Nav2 (controller_server) -- เปิด restart_nav2.sh ก่อน'
        if not all(x.successful for x in r.results):
            return 'controller_server ไม่รับค่า: ' + '; '.join(x.reason for x in r.results)
        r2 = self.call_srv(self.param_cli['controller_server'],   # ความเร็วหมุนของ Nav2 (แบบ "Nav2 หมุนเอง") แยกเรียก ไม่ให้พังทั้งชุด
                           SetParameters.Request(parameters=[dparam('FollowPath.rotate_to_heading_angular_vel', wz)]))
        if r2 is None or not all(x.successful for x in r2.results):
            self.log('  Nav2 ไม่รับความเร็วหมุน (rotate_to_heading_angular_vel) -- แบบ "Nav2 หมุนเอง" ใช้ค่าเดิม')
        g = self.call_srv(self.get_cli, GetParameters.Request(names=['max_velocity', 'min_velocity']))
        if g is not None and len(g.values) == 2:
            mx, mn = list(g.values[0].double_array_value), list(g.values[1].double_array_value)
            if len(mx) == 3 and len(mn) == 3:
                mx[0], mn[0] = v, -v
                mx[2], mn[2] = wz, -wz
                r = self.call_srv(self.param_cli['velocity_smoother'],
                                  SetParameters.Request(parameters=[dparam('max_velocity', mx), dparam('min_velocity', mn)]))
                if r is None or not all(x.successful for x in r.results):
                    return 'velocity_smoother ไม่รับค่า' + ('' if r is None else ': ' + '; '.join(x.reason for x in r.results))
        return 'ok'

    def set_speed(self, v):
        self.mission_speed = round(max(0.05, min(MISSION_VMAX, float(v))), 2)
        self.save_settings()
        res = self.apply_speed(self.mission_speed)
        self.log(f'ความเร็วภารกิจ {self.mission_speed:.2f} m/s' + ('' if res == 'ok' else f' (ยังไม่ถึง Nav2: {res})'))
        return (f'ตั้งความเร็วภารกิจ {self.mission_speed:.2f} m/s แล้ว' if res == 'ok'
                else f'จำความเร็ว {self.mission_speed:.2f} m/s ไว้แล้ว จะตั้งให้ตอนเริ่มภารกิจ ({res})')

    def set_turn(self, cfg):
        t = dict(self.turn_cfg)
        if cfg.get('mode') in ('nav2', 'smooth', 'arc'):
            t['mode'] = cfg['mode']
        lims = {'corner_deg': (20, 120), 'rot_vmax': (0.2, ROT_VMAX_MAX), 'rot_accel': (0.2, 3.0), 'arc_r': (0.15, 0.6)}
        for k, (lo, hi) in lims.items():
            if k in cfg:
                try:
                    t[k] = round(max(lo, min(hi, float(cfg[k]))), 2)
                except (TypeError, ValueError):
                    pass
        self.turn_cfg = t
        self.save_settings()
        res = self.apply_speed(self.mission_speed)
        names = {'nav2': 'Nav2 หมุนเอง', 'smooth': 'หมุนนุ่มโดยเว็บ', 'arc': 'เลี้ยวโค้งไม่หยุด'}
        self.log(f'แบบเลี้ยว: {names[t["mode"]]} · มุม>{t["corner_deg"]:.0f}° · หมุน {t["rot_vmax"]:.2f} rad/s'
                 f' เร่ง {t["rot_accel"]:.2f} · โค้ง r={t["arc_r"]:.2f}')
        return f'ตั้งแบบเลี้ยว "{names[t["mode"]]}" แล้ว' + ('' if res == 'ok' else f' (Nav2: {res})')

    def plan_in_band(self, trail):
        """หาทางเอง (A*) แต่ให้อยู่ในแถบ ±BAND_R รอบรอยที่เคยขับ หลบของในแมพ + ที่ lidar เห็นตอนนี้ เลือกเดินกลางทาง
        คืน None ถ้าหาไม่ได้ (ทางในแถบตัน)"""
        if self.grid is None or not self.pose or len(trail) < 2:
            return None
        v, w, h, res, ox, oy = self.grid
        cell = lambda x, y: (int((x - ox) / res), int((y - oy) / res))
        world = lambda c: (ox + (c[0] + 0.5) * res, oy + (c[1] + 0.5) * res)
        # 1) แถบรอบรอย: ช่อง -> ระยะถึงรอย
        dense = [trail[0]]
        for a, b in zip(trail, trail[1:]):
            n = max(1, int(math.hypot(b[0] - a[0], b[1] - a[1]) / res))
            dense += [(a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n) for k in range(1, n + 1)]
        nb = int(math.ceil(BAND_R / res))
        disk = [(di, dj, math.hypot(di, dj) * res) for di in range(-nb, nb + 1) for dj in range(-nb, nb + 1)
                if math.hypot(di, dj) * res <= BAND_R]
        band = {}
        for p in dense[::2]:
            ci, cj = cell(*p)
            for di, dj, d in disk:
                c = (ci + di, cj + dj)
                if d < band.get(c, 9.0):
                    band[c] = d
        # 2) ระยะถึงของที่ใกล้สุด (แมพ + lidar ตอนนี้) เฉพาะในแถบ
        obs = set()
        np_ = int(math.ceil(PREFER_R / res))
        lo_i = min(c[0] for c in band) - np_
        hi_i = max(c[0] for c in band) + np_
        lo_j = min(c[1] for c in band) - np_
        hi_j = max(c[1] for c in band) + np_
        for j in range(max(0, lo_j), min(h, hi_j + 1)):
            row = j * w
            for i in range(max(0, lo_i), min(w, hi_i + 1)):
                if v[row + i] == 2:
                    obs.add((i, j))
        if time.monotonic() - self.scan_t < 1.0:
            rx, ry, ryaw = self.pose
            c_, s_ = math.cos(ryaw), math.sin(ryaw)
            for bx, by in self.scan_pts:
                obs.add(cell(rx + bx * c_ - by * s_, ry + bx * s_ + by * c_))
        pdisk = [(di, dj, math.hypot(di, dj) * res) for di in range(-np_, np_ + 1) for dj in range(-np_, np_ + 1)
                 if math.hypot(di, dj) * res <= PREFER_R]
        dobs = {}
        for oi, oj in obs:
            for di, dj, d in pdisk:
                c = (oi + di, oj + dj)
                if c in band and d < dobs.get(c, 9.0):
                    dobs[c] = d

        def cost(c):
            d = dobs.get(c, 9.0)
            if d < BODY_R:
                return None
            near = max(0.0, (PREFER_R - d) / (PREFER_R - BODY_R))
            return 1.0 + CENTER_W * (band[c] / BAND_R) ** 2 + 3.0 * near * near
        start, goal_xy = cell(*self.pose[:2]), trail[-1]
        goal = cell(*goal_xy)
        if goal not in band or cost(goal) is None:          # จุดหมายชิดของ -> เลื่อนไปช่องในแถบที่ใกล้สุดที่ผ่านได้
            ok = [c for c in band if cost(c) is not None]
            if not ok:
                return None
            goal = min(ok, key=lambda c: (c[0] - goal[0]) ** 2 + (c[1] - goal[1]) ** 2)
            gx, gy = world(goal)
            self.log(f'  จุดหมายชิดของ เลื่อนไป {math.hypot(gx - goal_xy[0], gy - goal_xy[1]) * 100:.0f} cm')
            goal_xy = (gx, gy)
        # 3) A* 8 ทิศ (ช่องเริ่มยอมให้ชิดของได้ เพราะหุ่นอยู่ตรงนั้นแล้ว)
        steps = [(1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
                 (1, 1, 1.414), (1, -1, 1.414), (-1, 1, 1.414), (-1, -1, 1.414)]
        g, prev, pq = {start: 0.0}, {}, [(0.0, start)]
        hcost = lambda c: math.hypot(c[0] - goal[0], c[1] - goal[1])
        n_exp = 0
        while pq:
            f, c = heapq.heappop(pq)
            if c == goal:
                break
            n_exp += 1
            if n_exp > 200000:
                return None
            gc = g[c]
            for di, dj, sl in steps:
                n = (c[0] + di, c[1] + dj)
                if n not in band:
                    continue
                k = cost(n)
                if k is None:
                    continue
                ng = gc + sl * k
                if ng < g.get(n, 1e18):
                    g[n], prev[n] = ng, c
                    heapq.heappush(pq, (ng + hcost(n), n))
        if goal not in g:
            return None
        cells, c = [], goal
        while c != start:
            cells.append(c)
            c = prev[c]
        cells.reverse()
        # 4) เกลี่ยขั้นบันไดของกริด: เอาทุก ~0.2 m แล้วเฉลี่ย 3 จุด
        raw = [world(c) for c in cells]
        k = max(1, int(0.2 / res))
        pts = [self.pose[:2]] + raw[k - 1::k] + [goal_xy]
        sm = [pts[0]] + [((pts[i - 1][0] + pts[i][0] + pts[i + 1][0]) / 3, (pts[i - 1][1] + pts[i][1] + pts[i + 1][1]) / 3)
                         for i in range(1, len(pts) - 1)] + [pts[-1]]
        return [(round(x, 3), round(y, 3)) for x, y in sm]

    def leg_to(self, tx, ty):
        """เส้นทางจากหุ่นไปจุด (tx,ty): ตามรอยที่เคยขับ/เส้นทางที่บันทึก ถ้าไม่มีให้ Nav2 หาทาง (เลื่อนจุดถ้าชิดของ)
        คืน (จุด, คำอธิบาย, โหมด) โหมด 'follow' = เดินตามเส้นเป๊ะ, 'nav' = Nav2 วางทางเอง"""
        rx, ry = self.pose[:2]
        pts = self.path_on_driven(rx, ry, tx, ty)
        if pts:
            band = self.plan_in_band(pts)
            how = 'หาทางเองในแถบรอยที่เคยขับ'
            if band is None:
                band, how = pts, 'ตามรอยที่เคยขับ (หาทางในแถบไม่ได้)'
            ln = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(band, band[1:]))
            return band, f'{how} ({ln:.1f} m)', 'follow'
        # 7 ต.ค. ผู้ใช้: ไม่มีรอยก็ไม่ไป -- ปล่อย Nav2 วางทางเองจะชนขาโต๊ะที่ lidar มองไม่เห็น
        return None, 'ยังไม่มีรอยที่ขับไปจุดนี้ -- ขับไปจุดนี้เองก่อน 1 รอบ (หุ่นหรือจุดอยู่ห่างรอยเกิน 1 m)', 'none'

    @staticmethod
    def thin(pts):
        out = [pts[0]]
        for p in pts[1:]:
            if math.hypot(p[0] - out[-1][0], p[1] - out[-1][1]) >= WAYPOINT_GAP:
                out.append(p)
        if out[-1] != pts[-1]:
            out.append(pts[-1])
        return out

    def poses(self, pts, final_yaw=None):
        out = []
        for k, (x, y) in enumerate(pts):
            if k + 1 < len(pts):
                yaw = math.atan2(pts[k + 1][1] - y, pts[k + 1][0] - x)
            elif final_yaw is not None:
                yaw = final_yaw
            elif len(pts) > 1:
                yaw = math.atan2(y - pts[k - 1][1], x - pts[k - 1][0])
            else:
                yaw = math.atan2(y - self.pose[1], x - self.pose[0])
            p = PoseStamped()
            p.header.frame_id = 'map'
            p.header.stamp = self.get_clock().now().to_msg()
            p.pose.position.x, p.pose.position.y = float(x), float(y)
            p.pose.orientation.z, p.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            out.append(p)
        return out

    def run_goal(self, client, goal, name):
        """ส่งเป้าให้ Nav2 แล้วรอ -- ถ้าหุ่นไม่ขยับเลยเกิน STALL_S (Nav2 หมุนวน/รอ) ยกเลิกเอง"""
        if not client.wait_for_server(timeout_sec=5.0):
            return f'ไม่เจอ Nav2 ({name}) -- เปิด restart_nav2.sh ก่อน'
        box, ev = {}, threading.Event()
        client.send_goal_async(goal).add_done_callback(lambda f: (box.__setitem__('h', f.result()), ev.set()))
        if not ev.wait(10) or not box.get('h') or not box['h'].accepted:
            return 'Nav2 ไม่รับเป้า'
        h = self.goal_handle = box['h']
        ev2 = threading.Event()
        h.get_result_async().add_done_callback(lambda f: (box.__setitem__('r', f.result()), ev2.set()))
        ref, ref_t, stalled = self.pose, time.monotonic(), False
        while not ev2.wait(0.2):
            if self.cancel_flag:
                h.cancel_goal_async()
                ev2.wait(5)
                break
            x, y, yaw = self.pose
            if math.hypot(x - ref[0], y - ref[1]) > 0.05:      # หมุนอยู่กับที่ไม่นับ (คือที่เห็นวน)
                ref, ref_t = self.pose, time.monotonic()
            elif time.monotonic() - ref_t > STALL_S:
                stalled = True
                h.cancel_goal_async()
                ev2.wait(5)
                break
        self.goal_handle = None
        r = box.get('r')
        if self.cancel_flag:
            return 'ยกเลิก'
        if stalled:
            return f'ไม่เคลื่อนที่ {STALL_S:.0f} s (Nav2 วน/ติด) -- ยกเลิกคำสั่ง'
        return 'ok' if r is not None and r.status == 4 else f'ไปไม่ถึง (status {r.status if r else "?"})'

    def nav_through(self, pts, final_yaw=None):
        goal = NavigateThroughPoses.Goal()
        goal.poses = self.poses(self.thin(pts), final_yaw)
        return self.run_goal(self.nav_tp, goal, 'navigate_through_poses')

    def follow_line(self, pts, final_yaw=None):
        """เดินตามเส้นเป๊ะด้วยตัวคุมของ Nav2 (FollowPath) ไม่ให้ planner วางทางใหม่"""
        dense = [pts[0]]
        for a, b in zip(pts, pts[1:]):
            n = max(1, int(math.hypot(b[0] - a[0], b[1] - a[1]) / 0.05))
            dense += [(a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n) for k in range(1, n + 1)]
        goal = FollowPath.Goal()
        goal.path = Path()
        goal.path.header.frame_id = 'map'
        goal.path.header.stamp = self.get_clock().now().to_msg()
        goal.path.poses = self.poses(dense, final_yaw)
        return self.run_goal(self.follow_cli, goal, 'follow_path')

    def go(self, pts, final_yaw, mode):
        if mode == 'none':
            return 'ไม่มีรอยให้เดิน'
        self.leg_pts = [(round(x, 2), round(y, 2)) for x, y in pts[::max(1, len(pts) // 200)]] + [pts[-1]]
        if mode != 'follow':
            return self.nav_through(pts, final_yaw)
        tc = self.turn_cfg
        mode_t = tc['mode']
        # 11 ต.ค. ผู้ใช้: "ลองทำหลายๆ แบบ แล้วทดลองทีละอัน" -- เลือกแบบเลี้ยวได้บนเว็บ
        if mode_t == 'arc':                       # เลี้ยวโค้งต่อเนื่อง ไม่หยุด: ลบมุมเป็นโค้งแล้ววิ่งรวดเดียว
            cuts = self.corner_cuts(pts, tc['corner_deg'])
            if cuts:
                pts = self.fillet(pts, cuts, tc['arc_r'])
                self.log(f'  เลี้ยวโค้ง {len(cuts)} มุม (รัศมี {tc["arc_r"]:.2f} m)')
            return self.follow_line(pts, final_yaw)
        # 8 ต.ค. ผู้ใช้: เลี้ยวแล้วตัดโค้งไปชนของ -> แบ่งเส้นที่มุม วิ่งให้ถึงมุม หยุดหมุนอยู่กับที่ (เหมือนที่คนขับกด A/D) แล้วค่อยไปต่อ
        segs = self.split_corners(pts, tc['corner_deg'])
        if len(segs) > 1:
            self.log(f'  แบ่งทางเป็น {len(segs)} ช่วง หยุดหมุนที่มุม {len(segs) - 1} มุม'
                     f' ({"หมุนนุ่มโดยเว็บ" if mode_t == "smooth" else "Nav2 หมุน"})')
        for k, seg in enumerate(segs):
            if k + 1 < len(segs):
                cx_, cy_ = seg[-1]                # ทิศของช่วงถัดไป: จากมุมไปจุดที่ห่าง ~CORNER_WIN
                tx_, ty_ = next((q for q in segs[k + 1] if math.hypot(q[0] - cx_, q[1] - cy_) >= CORNER_WIN),
                                segs[k + 1][-1])
                yaw = math.atan2(ty_ - cy_, tx_ - cx_)
            else:
                yaw = final_yaw
            if mode_t == 'smooth':
                # หันเข้าทางของช่วงนี้ก่อน (นุ่ม) แล้วให้ Nav2 วิ่งตรงอย่างเดียว ไม่ต้องหมุนเอง
                h0 = self.seg_heading(seg)
                if h0 is not None:
                    r = self.smooth_rotate(h0)
                    if r != 'ok':
                        return r
                res = self.follow_line(seg, None)
                if res == 'ok' and k + 1 == len(segs) and final_yaw is not None:
                    res = self.smooth_rotate(final_yaw)
            else:
                res = self.follow_line(seg, yaw)
            if res != 'ok':
                return res
        return 'ok'

    @staticmethod
    def seg_heading(seg):
        """ทิศเริ่มของช่วง: จากจุดแรกไปจุดที่ห่าง ~CORNER_WIN"""
        if len(seg) < 2:
            return None
        x0, y0 = seg[0]
        tx, ty = next((q for q in seg if math.hypot(q[0] - x0, q[1] - y0) >= CORNER_WIN), seg[-1])
        if math.hypot(tx - x0, ty - y0) < 0.05:
            return None
        return math.atan2(ty - y0, tx - x0)

    def fresh_pose(self):
        """อ่าน map->base_link ตอนนี้เลย (ไม่รอ timer 5 Hz) ใช้ตอนหมุนนุ่ม"""
        try:
            t = self.tf.lookup_transform('map', 'base_link', Time()).transform
        except TransformException:
            return self.pose
        q = t.rotation
        return (t.translation.x, t.translation.y,
                math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))

    def smooth_rotate(self, tyaw):
        """หมุนอยู่กับที่แบบเร่ง-ผ่อน: ความเร็วตามมุมที่เหลือ w = min(vmax, sqrt(2*a*|err|)) ไม่หมุนเกินแล้วส่ายกลับ
        ส่งผ่าน collision monitor + เช็กชนด้วย lidar ก่อน ถ้าหมุนไม่ได้ -> กลับรถแบบรถยนต์"""
        tc = self.turn_cfg
        vmax, acc, tol = tc['rot_vmax'], tc['rot_accel'], math.radians(ROT_TOL_DEG)
        err0 = math.atan2(math.sin(tyaw - self.fresh_pose()[2]), math.cos(tyaw - self.fresh_pose()[2]))
        if abs(err0) < math.radians(8):
            return 'ok'
        w, t0, last_t = 0.0, time.monotonic(), time.monotonic()
        yaw_ref, moved_at = self.fresh_pose()[2], time.monotonic()
        settle = 0
        try:
            while True:
                if self.cancel_flag:
                    return 'ยกเลิก'
                now = time.monotonic()
                dt, last_t = max(0.01, now - last_t), now
                yaw = self.fresh_pose()[2]
                err = math.atan2(math.sin(tyaw - yaw), math.cos(tyaw - yaw))
                if abs(err) < tol:
                    settle += 1
                    if settle >= 3 or abs(w) < 0.05:
                        self.log(f'  หมุนนุ่มเสร็จ คลาด {math.degrees(err):+.1f}° ใช้ {now - t0:.1f} s')
                        return 'ok'
                else:
                    settle = 0
                target = math.copysign(min(vmax, math.sqrt(2 * acc * max(0.0, abs(err) - tol * 0.5))), err)
                if abs(err) < tol:
                    target = 0.0
                step = acc * dt                    # เร่ง/ผ่อนไม่เกิน acc
                w = min(w + step, target) if target > w else max(w - step, target)
                if abs(w) > 0.02 and self.blocked(0.0, w):
                    self.log('  หมุนนุ่ม: lidar บอกว่าหมุนต่อจะชน -> ใช้กลับรถแบบรถยนต์')
                    return self.kturn(tyaw)
                if abs(math.atan2(math.sin(yaw - yaw_ref), math.cos(yaw - yaw_ref))) > 0.03:
                    yaw_ref, moved_at = yaw, now
                elif abs(w) > 0.1 and now - moved_at > 2.5:
                    self.log('  หมุนนุ่ม: สั่งหมุนแต่ไม่หมุน (collision monitor หยุด?) -> ใช้กลับรถแบบรถยนต์')
                    return self.kturn(tyaw)
                if now - t0 > 25:
                    return f'หมุนไม่เสร็จใน 25 s (เหลือ {math.degrees(err):.0f}°)'
                t = Twist()
                t.angular.z = w
                self.pub_safe.publish(t)
                time.sleep(0.05)
        finally:
            for _ in range(3):
                self.pub_safe.publish(Twist())

    @staticmethod
    def fillet(pts, cuts, r):
        """ลบมุมเป็นโค้ง (Bezier ผ่านมุม) ตัดจุดก่อน/หลังมุมระยะ ~r ออก แล้วใส่โค้งแทน -- โค้งอยู่ในสามเหลี่ยมมุม จึงไม่ออกนอกแถบ"""
        def walk(i, step, d):
            j, acc = i, 0.0
            while 0 <= j + step < len(pts) and acc < d:
                acc += math.hypot(pts[j + step][0] - pts[j][0], pts[j + step][1] - pts[j][1])
                j += step
            return j
        out, last = [], 0
        for c in cuts:
            i0, i1 = walk(c, -1, r), walk(c, 1, r)
            if i0 < last:
                i0 = last
            if i1 <= c or i0 >= c:
                continue
            out += pts[last:i0 + 1]
            p0, v, p1 = pts[i0], pts[c], pts[i1]
            for k in range(1, 10):
                s = k / 10
                out.append(((1 - s) ** 2 * p0[0] + 2 * (1 - s) * s * v[0] + s * s * p1[0],
                            (1 - s) ** 2 * p0[1] + 2 * (1 - s) * s * v[1] + s * s * p1[1]))
            last = i1
        out += pts[last:]
        return out

    @staticmethod
    def corner_cuts(pts, deg=None):
        """ดัชนีจุดมุมที่ทางหักเกิน deg (ดูทิศย้อนหลัง/ล่วงหน้า ~CORNER_WIN m) กลุ่มที่หักต่อเนื่องเลือกจุดที่หักมากสุด"""
        if len(pts) < 3:
            return []

        def at_dist(i, step, d):
            j, acc = i, 0.0
            while 0 <= j + step < len(pts) and acc < d:
                acc += math.hypot(pts[j + step][0] - pts[j][0], pts[j + step][1] - pts[j][1])
                j += step
            return j
        turn = []
        for i in range(1, len(pts) - 1):
            a, b = at_dist(i, -1, CORNER_WIN), at_dist(i, 1, CORNER_WIN)
            if a == i or b == i:
                turn.append(0.0)
                continue
            h1 = math.atan2(pts[i][1] - pts[a][1], pts[i][0] - pts[a][0])
            h2 = math.atan2(pts[b][1] - pts[i][1], pts[b][0] - pts[i][0])
            turn.append(abs(math.atan2(math.sin(h2 - h1), math.cos(h2 - h1))))
        cuts, i = [], 0
        lim = math.radians(CORNER_DEG if deg is None else deg)
        while i < len(turn):
            if turn[i] > lim:
                j = i
                while j + 1 < len(turn) and turn[j + 1] > lim:
                    j += 1
                cuts.append(max(range(i, j + 1), key=lambda k: turn[k]) + 1)
                i = j + 1
            else:
                i += 1
        return cuts

    @staticmethod
    def split_corners(pts, deg=None):
        """แบ่งเส้นตรงมุมที่หักเกิน deg -- ช่วงสั้นกว่า CORNER_MERGE รวมกับช่วงก่อน (ทางตัว S = หมุนครั้งเดียว)"""
        if len(pts) < 3:
            return [pts]
        cuts = Admin.corner_cuts(pts, deg)
        segs, start = [], 0
        for c in cuts:
            seg = pts[start:c + 1]
            if sum(math.hypot(q[0] - p[0], q[1] - p[1]) for p, q in zip(seg, seg[1:])) >= CORNER_MERGE:
                segs.append(seg)
                start = c
        last = pts[start:]
        if len(last) >= 2:
            segs.append(last)
        elif segs:
            segs[-1] = segs[-1] + last
        return segs or [pts]

    def start_mission(self, label, legs):
        if self.mission:
            return 'กำลังทำภารกิจอยู่ -- ยกเลิกก่อน'
        if not self.pose:
            return 'ยังไม่รู้ตำแหน่งหุ่น'
        self.cancel_flag = False
        self.mission = label
        self.mission_thread = threading.Thread(target=self.run_mission, args=(label, legs), daemon=True)
        self.mission_thread.start()
        return f'เริ่ม: {label}'

    def run_mission(self, label, legs):
        try:
            sp = self.apply_speed(self.mission_speed)      # restart_nav2 คืนค่าใน yaml -- ตั้งใหม่ทุกภารกิจ
            if sp != 'ok':
                self.log(f'ตั้งความเร็วภารกิจไม่ได้: {sp} (ใช้ค่าเดิมของ Nav2)')
            self.msg = self.set_enable(True)
            if not self.enabled:
                self.msg = 'เปิดไดรเวอร์ไม่ได้ -- ยกเลิกภารกิจ'
                return
            for k, (name, make) in enumerate(legs, 1):
                if self.cancel_flag:
                    break
                pts, how, final_yaw, mode = make()
                if mode == 'none':
                    self.msg = f'[{k}/{len(legs)}] "{name}": {how} -- หยุดภารกิจ'
                    break
                res = self.turn_if_needed(pts)
                if res == 'ok':
                    self.msg = f'[{k}/{len(legs)}] กำลังไป "{name}" {how}'
                    res = self.go(pts, final_yaw, mode)
                tries = 0
                while res not in ('ok', 'ยกเลิก') and tries < LEG_RETRY and not res.startswith('ไม่เจอ Nav2'):
                    tries += 1
                    self.msg = f'[{k}/{len(legs)}] "{name}": {res} -- กลับรถแบบรถยนต์ (ครั้งที่ {tries})'
                    pts, how, final_yaw, mode = make()
                    if mode == 'none':
                        res = how
                        break
                    turn = self.kturn(self.heading_to(pts))
                    if turn != 'ok':
                        # กลับรถตรงนี้ไม่ได้ -> ถอยย้อนรอยหาที่โล่ง แล้วลองกลับรถอีกที
                        self.msg = f'[{k}/{len(legs)}] {turn} -- ถอยหาที่โล่ง'
                        back = self.back_to_open()
                        if back != 'ok':
                            res = back
                            break
                        pts, how, final_yaw, mode = make()
                        if mode == 'none':
                            res = how
                            break
                        turn = self.turn_if_needed(pts)
                        if turn != 'ok':
                            res = turn
                            break
                    pts, how, final_yaw, mode = make()
                    if mode == 'none':
                        res = how
                        break
                    self.msg = f'[{k}/{len(legs)}] กลับรถแล้ว ไป "{name}" {how}'
                    res = self.go(pts, final_yaw, mode)
                if res != 'ok':
                    self.msg = f'[{k}/{len(legs)}] "{name}": {res} -- หยุดภารกิจ'
                    break
                self.msg = f'[{k}/{len(legs)}] ถึง "{name}" แล้ว'
            else:
                self.msg = f'{label} เสร็จแล้ว'
        finally:
            for _ in range(3):
                self.pub.publish(Twist())
            self.set_enable(False)
            self.mission = None
            self.leg_pts = []

    def is_open(self, x, y, r=OPEN_R):
        """รอบจุด (x,y) รัศมี r ไม่มีช่องที่แมพบอกว่ามีสิ่งกีดขวาง"""
        if self.grid is None:
            return False
        v, w, h, res, ox, oy = self.grid
        ci, cj, n = int((x - ox) / res), int((y - oy) / res), int(math.ceil(r / res))
        for j in range(cj - n, cj + n + 1):
            for i in range(ci - n, ci + n + 1):
                if (i - ci) ** 2 + (j - cj) ** 2 > n * n:
                    continue
                if not (0 <= i < w and 0 <= j < h) or v[j * w + i] == 2:
                    return False
        return True

    def back_to_open(self):
        """ถอยหลังย้อนรอยที่วิ่งมา จนกว่ารอบตัวจะโล่ง OPEN_R ถ้ารอยหมดแล้วยังไม่โล่ง = จบภารกิจ"""
        if self.is_open(*self.pose[:2]):
            return 'ok'
        self.trail_pause = True
        trail = list(self.trail)
        try:
            # เริ่มจากจุดบนรอยที่ใกล้หุ่นสุด (มองย้อนจากท้าย)
            rx, ry = self.pose[:2]
            idx = min(range(len(trail)), key=lambda k: math.hypot(trail[k][0] - rx, trail[k][1] - ry)) if trail else 0
            moved_at, last_xy, dist = time.monotonic(), (rx, ry), 0.0
            while True:
                if self.cancel_flag:
                    return 'ยกเลิก'
                rx, ry, yaw = self.pose
                d = math.hypot(rx - last_xy[0], ry - last_xy[1])
                if d > 0.02:
                    dist += d; last_xy = (rx, ry); moved_at = time.monotonic()
                if self.is_open(rx, ry):
                    self.trail = trail[:idx + 1]
                    return 'ok'
                # เลื่อนเป้าไปตามรอยย้อนหลัง ให้อยู่ห่างหุ่น ~25 cm
                while idx > 0 and math.hypot(trail[idx][0] - rx, trail[idx][1] - ry) < 0.25:
                    idx -= 1
                if not trail or (idx == 0 and math.hypot(trail[0][0] - rx, trail[0][1] - ry) < 0.1):
                    return f'ถอยจนสุดรอยแล้ว ({dist:.1f} m) ยังไม่เจอที่โล่งพอกลับรถ'
                if time.monotonic() - moved_at > 6.0:
                    return f'ถอยไม่ได้ (มีของด้านหลังหรือติด) หลังถอยไป {dist:.1f} m'
                tx, ty = trail[idx]
                # ขับถอย: ท้ายหุ่นหันเข้าหาเป้า
                err = math.atan2(ty - ry, tx - rx) - (yaw + math.pi)
                err = math.atan2(math.sin(err), math.cos(err))
                t = Twist()
                t.linear.x = -BACK_VX * max(0.3, math.cos(err)) if abs(err) < 1.2 else 0.0
                t.angular.z = max(-0.4, min(0.4, 1.5 * err))
                self.pub_safe.publish(t)
                self.msg = f'กำลังถอยย้อนรอย {dist:.1f} m หาที่โล่ง'
                time.sleep(0.1)
        finally:
            for _ in range(3):
                self.pub_safe.publish(Twist())
            self.trail_pause = False

    def blocked(self, vx, wz):
        """จำลองว่าถ้าขับ (vx, wz) ต่อไปอีก 0.3/0.6/0.9 s ตัวหุ่น (+เผื่อ KT_MARGIN) จะทับจุด lidar ไหม"""
        if time.monotonic() - self.scan_t > 1.0:
            return True                   # ไม่มี scan สดๆ = ถือว่าติดไว้ก่อน
        f, b, w = FOOT[0] + KT_MARGIN, FOOT[1] - KT_MARGIN, FOOT[2] + KT_MARGIN
        pts = self.scan_pts
        for dt in (0.3, 0.6, 0.9):
            th = wz * dt
            # ตำแหน่งหุ่นหลังขับ (โค้งวงกลม) ในกรอบปัจจุบัน
            if abs(wz) > 1e-6:
                px, py = vx / wz * math.sin(th), vx / wz * (1 - math.cos(th))
            else:
                px, py = vx * dt, 0.0
            c, sn = math.cos(-th), math.sin(-th)
            hit = 0
            for x, y in pts:
                dx, dy = x - px, y - py
                qx, qy = dx * c - dy * sn, dx * sn + dy * c
                if b <= qx <= f and -w <= qy <= w:
                    hit += 1
                    if hit >= 2:
                        return True
        return False

    def kturn(self, tyaw):
        """กลับรถแบบรถยนต์: เลี้ยวทางเดียว สลับเดินหน้า/ถอยหลังเมื่อติดหรือครบช่วง จนหันตรง tyaw"""
        t_start, switches, dirn = time.monotonic(), 0, 1
        e0 = math.atan2(math.sin(tyaw - self.pose[2]), math.cos(tyaw - self.pose[2]))
        sign = 1.0 if e0 > 0 else -1.0
        seg, moved_at, last, prev, dbg_t = 0.0, time.monotonic(), self.pose, self.pose, 0.0
        self.log(f'เริ่มกลับรถ: หุ่น ({self.pose[0]:.2f},{self.pose[1]:.2f}) หัน {math.degrees(self.pose[2]):.0f}° '
                 f'ต้องหัน {math.degrees(tyaw):.0f}° เลี้ยว{"ซ้าย" if sign > 0 else "ขวา"} · scan {len(self.scan_pts)} จุด '
                 f'อายุ {time.monotonic() - self.scan_t:.1f} s')
        try:
            while True:
                if self.cancel_flag:
                    return 'ยกเลิก'
                x, y, yaw = self.pose
                e = math.atan2(math.sin(tyaw - yaw), math.cos(tyaw - yaw))
                if time.monotonic() - dbg_t > 2.0:
                    dbg_t = time.monotonic()
                    self.log(f'  กลับรถ: ({x:.2f},{y:.2f}) หัน {math.degrees(yaw):.0f}° เหลือ {math.degrees(e):.0f}° '
                             f'ทิศ {"หน้า" if dirn > 0 else "หลัง"} สลับ {switches}')
                if abs(e) < KT_DONE:
                    self.log(f'กลับรถเสร็จ หัน {math.degrees(yaw):.0f}° (สลับทิศ {switches} ครั้ง)')
                    return 'ok'
                if time.monotonic() - t_start > 120 or switches > 20:
                    return f'กลับรถไม่สำเร็จ (สลับทิศ {switches} ครั้ง ยังหันไม่ตรง)'
                wz = sign * KT_WZ
                seg += math.hypot(x - prev[0], y - prev[1])     # ระยะที่วิ่งจริงในช่วงนี้
                prev = (x, y, yaw)
                if math.hypot(x - last[0], y - last[1]) > 0.01 or abs(yaw - last[2]) > 0.02:
                    last, moved_at = (x, y, yaw), time.monotonic()
                vx = dirn * KT_VX
                stuck = time.monotonic() - moved_at > 2.5
                blk = self.blocked(vx, wz)
                if seg > KT_SEG or stuck or blk:
                    why = 'ครบช่วง' if seg > KT_SEG else ('ไม่ขยับ' if stuck else 'ข้างหน้าจะชน' if dirn > 0 else 'ข้างหลังจะชน')
                    if not self.blocked(-vx, wz):
                        dirn, switches = -dirn, switches + 1
                        seg, moved_at = 0.0, time.monotonic()
                        vx = dirn * KT_VX
                        self.log(f'  สลับเป็น{"เดินหน้า" if dirn > 0 else "ถอย"} ({why})')
                    elif not self.blocked(0.0, wz):
                        vx = 0.0                     # หมุนอยู่กับที่ได้ก็หมุน
                    elif not self.blocked(vx, -wz) or not self.blocked(-vx, -wz):
                        sign = -sign                 # ทางนี้ตันหมด ลองเลี้ยวอีกทาง
                        self.log(f'  เลี้ยวทางนี้ตัน เปลี่ยนเป็นเลี้ยว{"ซ้าย" if sign > 0 else "ขวา"}')
                        continue
                    else:
                        return 'กลับรถไม่ได้ ติดทั้งหน้า หลัง และหมุน'
                t = Twist()
                t.linear.x, t.angular.z = vx, wz
                self.pub_safe.publish(t)
                act = 'เดินหน้า' if vx > 0 else ('ถอย' if vx < 0 else 'หมุน')
                self.msg = f'กำลังกลับรถแบบรถยนต์ ({act}) เหลือ {math.degrees(abs(e)):.0f} องศา สลับทิศแล้ว {switches} ครั้ง'
                time.sleep(0.1)
        finally:
            for _ in range(3):
                self.pub_safe.publish(Twist())

    def heading_to(self, pts):
        """ทิศจากหุ่นไปจุดแรกบนเส้นทางที่ห่างเกิน 0.6 m"""
        rx, ry = self.pose[:2]
        for px, py in pts:
            if math.hypot(px - rx, py - ry) > 0.6:
                return math.atan2(py - ry, px - rx)
        px, py = pts[-1]
        return math.atan2(py - ry, px - rx)

    def turn_if_needed(self, pts):
        """ทางไปอยู่ด้านหลัง (> 100 องศา) และรอบตัวไม่โล่ง -> กลับรถแบบรถยนต์ก่อนส่งให้ Nav2"""
        tyaw = self.heading_to(pts)
        e = math.atan2(math.sin(tyaw - self.pose[2]), math.cos(tyaw - self.pose[2]))
        if abs(e) > math.radians(100) and not self.is_open(*self.pose[:2]):
            return self.kturn(tyaw)
        return 'ok'

    def follow_route(self, name, reverse):
        r = next((r for r in self.list_routes() if r['name'] == name), None)
        if not r:
            return f'ไม่เจอเส้นทาง "{name}"'
        pts = self.load_route(r['file'])
        if reverse:
            pts = pts[::-1]

        def make():
            rx, ry = self.pose[:2]
            d = [math.hypot(px - rx, py - ry) for px, py in pts]
            i0 = d.index(min(d))
            return [(rx, ry)] + pts[i0:], f'(เริ่มจากจุดที่ใกล้สุด ห่าง {d[i0]:.1f} m)', None, 'follow'
        return self.start_mission(f'เดินตามเส้นทาง "{name}"' + (' (ย้อนกลับ)' if reverse else ''), [(name, make)])

    def go_places(self, names, home):
        by = {p['name']: p for p in self.places}
        seq = [n for n in names if n in by]
        if home and 'บ้าน' in by and (not seq or seq[-1] != 'บ้าน'):
            seq.append('บ้าน')
        if not seq:
            return 'ยังไม่ได้เลือกจุด'

        def maker(p):
            def make():
                pts, how, mode = self.leg_to(p['x'], p['y'])
                return pts, how, p.get('yaw'), mode
            return make
        return self.start_mission('ไปตามคิว: ' + ' → '.join(seq), [(n, maker(by[n])) for n in seq])

    def cancel_mission(self):
        if not self.mission:
            return 'ไม่มีภารกิจที่กำลังทำ'
        self.cancel_flag = True
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
        return 'กำลังยกเลิกภารกิจ...'

    def del_place(self, name):
        self.places = [p for p in self.places if p['name'] != name]
        self.save_places()
        return f'ลบจุด "{name}" แล้ว'

    def save_places(self):
        with open(self.places_file(), 'w', encoding='utf-8') as fp:
            json.dump({'frame': 'map', 'map': self.map_name, 'places': self.places}, fp, ensure_ascii=False, indent=1)

    # ---------------- แมพ / ตำแหน่ง / เส้นทาง ----------------
    def on_jpeg(self, m):
        with self.jpeg_cv:
            self.jpeg, self.jpeg_n = bytes(m.data), self.jpeg_n + 1
            self.jpeg_cv.notify_all()

    def on_map(self, m):
        v = bytes(0 if c < 0 else (2 if c > 50 else 1) for c in m.data)
        i = m.info
        self.grid = (v, i.width, i.height, i.resolution, i.origin.position.x, i.origin.position.y)
        self.map_ver += 1
        self.map_json = json.dumps({'w': i.width, 'h': i.height, 'res': i.resolution,
                                    'ox': i.origin.position.x, 'oy': i.origin.position.y,
                                    'data': base64.b64encode(v).decode()})

    def on_scan(self, s):
        try:
            t = self.tf.lookup_transform('base_link', s.header.frame_id, Time()).transform
        except TransformException:
            return
        q = t.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        c, sn, tx, ty = math.cos(yaw), math.sin(yaw), t.translation.x, t.translation.y
        pts = []
        for k, r in enumerate(s.ranges):
            if not (s.range_min <= r <= min(s.range_max, 1.5)):
                continue          # สนใจแค่ของใกล้ตัว
            a = s.angle_min + k * s.angle_increment
            lx, ly = r * math.cos(a), r * math.sin(a)
            pts.append((tx + lx * c - ly * sn, ty + lx * sn + ly * c))
        self.scan_pts, self.scan_t = pts, time.monotonic()

    def track(self):
        try:
            t = self.tf.lookup_transform('map', 'base_link', Time()).transform
        except TransformException:
            return
        q = t.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.pose = (t.translation.x, t.translation.y, yaw)
        if not self.trail_pause:
            x, y = self.pose[:2]
            if not self.trail or math.hypot(x - self.trail[-1][0], y - self.trail[-1][1]) >= TRAIL_STEP:
                self.trail.append((x, y))
                del self.trail[:-1000]   # เก็บไว้ ~100 m ล่าสุด
        x, y = self.pose[:2]
        last = next((q for q in reversed(self.driven[-1:]) if q), None)
        if last is None or math.hypot(x - last[0], y - last[1]) >= DRIVEN_STEP:
            if last is not None and math.hypot(x - last[0], y - last[1]) > 0.8:
                self.driven.append(None)          # ตำแหน่งกระโดด (SLAM แก้ตำแหน่ง) = รอยขาด ไม่ต่อกัน
            self.driven.append((round(x, 3), round(y, 3)))
            del self.driven[:-DRIVEN_MAX]
            self.driven_dirty += 1
            if self.driven_dirty >= 25:           # เซฟทุก ~5 m
                self.save_driven()
        if self.recording:
            x, y = self.pose[:2]
            if not self.path or math.hypot(x - self.path[-1][0], y - self.path[-1][1]) >= STEP:
                self.path.append((round(x, 3), round(y, 3)))

    def path_len(self):
        return sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(self.path, self.path[1:]))

    @staticmethod
    def safe_name(name, default):
        name = re.sub(r'[^\w\-]', '_', str(name).strip())[:40]
        return name or default

    def set_record(self, on, name):
        if on:
            self.route_name = self.safe_name(name, 'main')
            self.path = [tuple(round(v, 3) for v in self.pose[:2])] if self.pose else []
            self.recording = True
            return f'เริ่มบันทึกเส้นทาง "{self.route_name}" -- ขับไปตามทางได้เลย'
        self.recording = False
        if len(self.path) < 2:
            return 'เส้นทางสั้นเกินไป ไม่ได้บันทึก'
        os.makedirs(self.route_dir(), exist_ok=True)
        f = os.path.join(self.route_dir(), self.route_name + '.json')
        with open(f, 'w', encoding='utf-8') as fp:
            json.dump({'name': self.route_name, 'frame': 'map', 'step_m': STEP,
                       'saved': time.strftime('%Y-%m-%d %H:%M:%S'), 'points': self.path}, fp, ensure_ascii=False, indent=1)
        return f'บันทึกเส้นทาง "{self.route_name}" แล้ว {len(self.path)} จุด ยาว {self.path_len():.1f} m -> {f}'

    def load_places(self):
        for f in (self.places_file(), LEGACY_PLACES):
            try:
                with open(f, encoding='utf-8') as fp:
                    return json.load(fp).get('places', [])
            except (OSError, ValueError):
                continue
        return []

    def add_place(self, name):
        if not self.pose:
            return 'ยังไม่รู้ตำแหน่งหุ่น -- บันทึกจุดไม่ได้'
        name = str(name).strip()[:40]
        x, y, yaw = self.pose
        self.places = [p for p in self.places if p['name'] != name]
        self.places.append({'name': name, 'x': round(x, 3), 'y': round(y, 3), 'yaw': round(yaw, 3)})
        self.save_places()
        return f'บันทึกจุด "{name}" ที่ x={x:.2f} y={y:.2f} แล้ว'

    def save_map(self, name):
        name = self.safe_name(name, 'room_2')
        try:
            out = subprocess.run(['bash', SAVE_ROOM, name], capture_output=True, text=True, timeout=120)
        except Exception as e:  # noqa: BLE001
            return f'เซฟแมพไม่สำเร็จ: {e}'
        last = (out.stdout.strip().splitlines() or [''])[-1]
        if out.returncode != 0:
            return f'เซฟแมพไม่สำเร็จ: {out.stderr.strip()[-200:]}'
        if self.driven_dirty:
            self.save_driven()
        if name != self.map_name:      # ย้ายเส้นทาง/จุดที่ทำบนแมพนี้ไปอยู่ใต้ชื่อใหม่
            old_dir, old_places = self.route_dir(), self.places_file()
            new_dir = os.path.join(ROUTES, name)
            os.makedirs(new_dir, exist_ok=True)
            if os.path.isdir(old_dir):
                for f in os.listdir(old_dir):
                    os.replace(os.path.join(old_dir, f), os.path.join(new_dir, f))
            places = self.places
            self.map_name = name
            self.places = places
            self.save_places()
            if os.path.exists(old_places):
                os.remove(old_places)
            with open(ACTIVE, 'w', encoding='utf-8') as fp:
                fp.write(name)
        return f'เซฟแมพ "{name}" แล้ว {last}'

    def drive(self, vx, wz):
        vx = max(-1.0, min(1.0, float(vx))) * MAX_VX
        wz = max(-1.0, min(1.0, float(wz))) * MAX_WZ
        if self.mission and (vx or wz):
            self.cancel_mission()
            return 'กดขับเอง -- ยกเลิกภารกิจแล้ว (กดอีกครั้งเพื่อขับ)'
        with self.lock:
            self.cmd, self.last = (vx, wz), time.monotonic()
        if not self.enabled and (vx or wz):
            return 'ไดรเวอร์ปิดอยู่ -- กด "เปิดไดรเวอร์" ก่อน'
        return f'กำลังขับ vx={vx:+.2f} m/s  wz={wz:+.2f} rad/s'

    def tick(self):
        with self.lock:
            vx, wz = self.cmd
            fresh = time.monotonic() - self.last < TIMEOUT
        if fresh and (vx or wz):
            t = Twist(); t.linear.x, t.angular.z = vx, wz
            self.pub.publish(t); self.moving = True
        elif self.moving:
            self.pub.publish(Twist()); self.moving = False

    def set_enable(self, on):
        if not self.en_cli.wait_for_service(timeout_sec=2.0):
            return 'ไม่เจอ /mecanum_base/enable'
        if not on:
            self.pub.publish(Twist())
            if self.mission and threading.current_thread() is not self.mission_thread:
                self.cancel_mission()
        done = threading.Event(); res = {}
        f = self.en_cli.call_async(SetBool.Request(data=bool(on)))
        f.add_done_callback(lambda fut: (res.setdefault('r', fut.result()), done.set()))
        done.wait(3.0)
        r = res.get('r')
        if r is not None and r.success:
            self.enabled = bool(on)
        return ('เปิดไดรเวอร์แล้ว' if on else 'ปิดไดรเวอร์แล้ว หุ่นหยุด') if r and r.success else 'สั่งไดรเวอร์ไม่สำเร็จ'


node = None


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *a):
        pass

    def reply(self, code, body, ctype='application/json; charset=utf-8'):
        data = body.encode('utf-8') if isinstance(body, str) else body
        self.send_response(code); self.send_header('Content-Type', ctype)
        self.send_header('Content-Length', str(len(data))); self.end_headers(); self.wfile.write(data)

    def stream_cam(self):
        """MJPEG จากภาพ YOLO: ส่งต่อ JPEG ที่ได้มาตรงๆ ทุกครั้งที่มีภาพใหม่"""
        self.send_response(200)
        self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
        self.send_header('Cache-Control', 'no-cache')
        self.end_headers()
        last = -1
        try:
            while True:
                with node.jpeg_cv:
                    node.jpeg_cv.wait_for(lambda: node.jpeg_n != last, timeout=5.0)
                    if node.jpeg is None or node.jpeg_n == last:
                        continue
                    data, last = node.jpeg, node.jpeg_n
                self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\nContent-Length: ' + str(len(data)).encode()
                                 + b'\r\n\r\n' + data + b'\r\n')
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass

    def do_GET(self):
        if self.path in ('/', '/index.html'):
            self.reply(200, PAGE, 'text/html; charset=utf-8')
        elif self.path == '/map':
            self.reply(200, node.map_json)
        elif self.path.startswith('/cam.mjpg'):
            self.stream_cam()
        elif self.path == '/status':
            self.reply(200, json.dumps({'pose': node.pose, 'recording': node.recording, 'route': node.route_name,
                                        'path': node.path, 'length': node.path_len(), 'places': node.places,
                                        'msg': node.msg, 'mission': node.mission, 'map_ver': node.map_ver,
                                        'history': node.history, 'leg': node.leg_pts}, ensure_ascii=False))
        elif self.path == '/lists':
            self.reply(200, json.dumps({'map': node.map_name, 'slam_map': node.slam_map(), 'maps': node.list_maps(),
                                        'driven_m': round(node.driven_len(), 1),
                                        'start': node.start_info(),
                                        'mission_speed': node.mission_speed, 'speed_max': MISSION_VMAX,
                                        'turn': node.turn_cfg, 'rot_max': ROT_VMAX_MAX,
                                        'driven': [(round(q[0], 2), round(q[1], 2)) if q else None
                                                   for q in node.driven[::2]],
                                        'routes': [{k: r[k] for k in ('name', 'points', 'length')} for r in node.list_routes()],
                                        'places': node.places}, ensure_ascii=False))
        elif self.path == '/state':
            self.reply(200, json.dumps({'enabled': node.enabled, 'msg': 'ไดรเวอร์เปิด พร้อมขับ' if node.enabled else 'ไดรเวอร์ปิด'}, ensure_ascii=False))
        else:
            self.reply(404, '{}')

    def do_POST(self):
        try:
            body = json.loads(self.rfile.read(int(self.headers.get('Content-Length', 0)) or 0) or b'{}')
        except ValueError:
            body = {}
        if self.path == '/drive':
            msg = node.drive(body.get('vx', 0), body.get('wz', 0))
        elif self.path == '/enable':
            msg = node.set_enable(body.get('on', False))
        elif self.path == '/record':
            msg = node.msg = node.set_record(bool(body.get('on')), body.get('name', 'main'))
        elif self.path == '/place':
            msg = node.msg = node.add_place(body.get('name', ''))
        elif self.path == '/setstart':
            msg = node.msg = node.set_start(body.get('name', ''))
        elif self.path == '/usestart':
            msg = node.msg = node.use_start(body.get('name', ''))
        elif self.path == '/delstart':
            msg = node.msg = node.del_start(body.get('name', ''))
        elif self.path == '/savemap':
            msg = node.msg = node.save_map(body.get('name', 'room_2'))
        elif self.path == '/follow':
            msg = node.msg = node.follow_route(body.get('name', ''), bool(body.get('reverse')))
        elif self.path == '/goplaces':
            msg = node.msg = node.go_places(list(body.get('names', [])), bool(body.get('home')))
        elif self.path == '/turncfg':
            msg = node.msg = node.set_turn(body)
        elif self.path == '/speed':
            msg = node.msg = node.set_speed(body.get('v', 0.15))
        elif self.path == '/cancel':
            msg = node.msg = node.cancel_mission()
        elif self.path == '/delplace':
            msg = node.msg = node.del_place(body.get('name', ''))
        elif self.path == '/usemap':
            msg = node.msg = node.use_map(body.get('name', ''))
        else:
            self.reply(404, '{}'); return
        self.reply(200, json.dumps({'enabled': node.enabled, 'msg': msg}, ensure_ascii=False))


def main():
    global node
    rclpy.init()
    node = Admin()
    srv = ThreadingHTTPServer(('0.0.0.0', PORT), Handler)
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    node.get_logger().info(f'เว็บหลังบ้าน: http://udru-desktop.local:{PORT}')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub.publish(Twist())
        if node.enabled:
            node.set_enable(False)
        srv.shutdown(); node.destroy_node(); rclpy.try_shutdown()


if __name__ == '__main__':
    main()
