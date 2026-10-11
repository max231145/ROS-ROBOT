#!/usr/bin/env python3
"""
sim_two_drivers -- จำลองการต่อ DM556 สองตัว (ซ้าย 1 ตัว ขวา 1 ตัว) ก่อนต่อจริง ไม่ต้องใช้ ROS/บอร์ด

ใช้โค้ดจริงของ node ทั้งเส้นทาง: kinematics.inverse -> invert_wheels -> limit_wheel_rates
-> เฟิร์มแวร์ปั่นพัลส์ 4 ช่อง -> (จำลอง) สายไฟ -> ไดรเวอร์ -> มอเตอร์ -> หุ่นเคลื่อนจริง
แล้วเทียบกับ odom ที่ node คิดจากตัวนับ step (node ไม่รู้ว่าสายต่อผิด)

    python tools/sim_two_drivers.py            # สรุปในเทอร์มินัล
    python tools/sim_two_drivers.py --html     # + ไฟล์ภาพเคลื่อนไหว tools/sim_two_drivers.html

ช่องเฟิร์มแวร์: 0 = หน้าซ้าย (STEP 25 / DIR 26)  1 = หน้าขวา (STEP 27 / DIR 14)
                2 = หลังซ้าย (STEP 32 / DIR 33)  3 = หลังขวา (STEP 18 / DIR 19)   ENA 23 ร่วม
แผนต่อ: ไดรเวอร์ซ้าย <- ช่อง 0  ไดรเวอร์ขวา <- ช่อง 1  ช่อง 2, 3 ปล่อยว่าง
ตอน vy = 0 สมการให้ FL = RL และ FR = RR พอดี ช่อง 0/1 จึงแทนทั้งข้างได้
ช่อง 2/3 เฟิร์มแวร์ยังนับ step อยู่ odom จึงยังคิดครบ 4 ล้อ
"""

import itertools
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'mecanum_base'))
from kinematics import MecanumKinematics, integrate_pose, limit_wheel_rates  # noqa: E402

# ค่าจาก config/mecanum_base.yaml
R, L, W = 0.0661, 0.42, 0.54
STEPS_PER_RAD = 200 * 8 / (2 * math.pi)
MAX_WHEEL_RATE = 3200

DT, T = 0.02, 3.0
COMMANDS = [  # (ชื่อ, vx, wz, ผลที่ต้องได้)
    ('เดินหน้า', 0.10, 0.0, 'fwd'),
    ('ถอยหลัง', -0.10, 0.0, 'back'),
    ('หมุนซ้าย (ทวนเข็ม)', 0.0, 0.5, 'left'),
    ('หมุนขวา (ตามเข็ม)', 0.0, -0.5, 'right'),
]
LABEL = {'fwd': 'เดินหน้า', 'back': 'ถอยหลัง', 'left': 'หมุนซ้าย', 'right': 'หมุนขวา',
         'stop': 'อยู่กับที่', 'arc': 'เลี้ยวโค้ง', 'fight': 'มอเตอร์สู้กัน/ลากไถล', 'other': 'เพี้ยน'}


class Wiring:
    """
    ch_left/ch_right : ช่องเฟิร์มแวร์ที่ต่อเข้าไดรเวอร์ซ้าย/ขวา (None = ไดรเวอร์ไม่ทำงาน)
    flip_left/right  : ทิศทั้งข้างกลับ (สลับ DIR+/DIR- หรือสลับคอยล์ A+/A- ที่ขั้วไดรเวอร์)
    flip_motor       : มอเตอร์ตัวเดียวกลับ เทียบกับอีกตัวในข้างเดียวกัน {'FL','RL','FR','RR'}
    scale_right      : ไดรเวอร์ขวาหมุนกี่เท่าของที่สั่ง (DIP microstep 3200 แทน 1600 = 0.5)
    """

    def __init__(self, key, name, why, ch_left=0, ch_right=1, flip_left=False, flip_right=False,
                 flip_motor=(), scale_right=1.0):
        self.key, self.name, self.why = key, name, why
        self.ch_left, self.ch_right = ch_left, ch_right
        self.flip_left, self.flip_right = flip_left, flip_right
        self.flip_motor = set(flip_motor)
        self.scale_right = scale_right


CASES = [
    Wiring('ok', 'ต่อถูกต้อง', 'ซ้าย->ช่อง 0, ขวา->ช่อง 1, สายมอเตอร์เหมือนตอนต่อ DM556 ตัวเดียว'),
    Wiring('left_rev', 'ข้างซ้ายทิศกลับ', 'สลับ DIR+/DIR- หรือ A+/A- ที่ไดรเวอร์ซ้าย', flip_left=True),
    Wiring('right_rev', 'ข้างขวาทิศกลับ', 'ขวาติดตั้งกลับด้าน (mirror) ถ้าแยกสายมอเตอร์ใหม่มักเจอข้อนี้',
           flip_right=True),
    Wiring('both_rev', 'ทิศกลับทั้งสองข้าง', 'DIR_FORWARD_LEVEL ตรงข้ามกับสายชุดใหม่',
           flip_left=True, flip_right=True),
    Wiring('swap', 'สลับข้าง', 'ไดรเวอร์ซ้ายเสียบช่อง 1 (27/14) ขวาเสียบช่อง 0 (25/26)', ch_left=1, ch_right=0),
    Wiring('swap_both_rev', 'สลับข้าง + ทิศกลับทั้งสองข้าง', 'ผิดสองจุดพร้อมกัน',
           ch_left=1, ch_right=0, flip_left=True, flip_right=True),
    Wiring('swap_left_rev', 'สลับข้าง + ซ้ายทิศกลับ', 'ผิดสองจุดพร้อมกัน', ch_left=1, ch_right=0, flip_left=True),
    Wiring('rl_rev', 'มอเตอร์หลังซ้ายกลับตัวเดียว', 'สายมอเตอร์หลังซ้ายสลับคอยล์ ต่อขนานกับหน้าซ้าย',
           flip_motor=['RL']),
    Wiring('fr_rev', 'มอเตอร์หน้าขวากลับตัวเดียว', 'สายมอเตอร์หน้าขวาสลับคอยล์', flip_motor=['FR']),
    Wiring('right_microstep', 'DIP ไดรเวอร์ขวาตั้ง 3200', 'microstep ไม่ตรงกับไดรเวอร์ซ้าย (1600) ข้างขวาหมุนครึ่งเดียว',
           scale_right=0.5),
    Wiring('right_dead', 'ไดรเวอร์ขวาไม่หมุน', 'ENA/ไฟ 24V/STEP ข้างขวาไม่ได้ต่อ หรือ ALM ติด', ch_right=None),
]


def node_rates(vx, wz, invert):
    """ตรงกับ MecanumBaseNode.on_timer: step/s 4 ช่องที่ส่งเป็นคำสั่ง V"""
    kin = MecanumKinematics(R, L, W)
    sign = [-1 if inv else 1 for inv in invert]
    rates = [sign[i] * w * STEPS_PER_RAD for i, w in enumerate(kin.inverse(vx, 0.0, wz))]
    return [int(round(r)) for r in limit_wheel_rates(rates, MAX_WHEEL_RATE)]


def actual_wheels(ch_rates, wiring):
    """step/s ของช่องเฟิร์มแวร์ -> ความเร็วล้อจริง (rad/s, + = พาหุ่นไปหน้า) ลำดับ FL FR RL RR"""
    def side(ch, flip):
        if ch is None:
            return 0.0
        return (-1 if flip else 1) * ch_rates[ch] / STEPS_PER_RAD
    left = side(wiring.ch_left, wiring.flip_left)
    right = side(wiring.ch_right, wiring.flip_right) * wiring.scale_right
    w = {'FL': left, 'RL': left, 'FR': right, 'RR': right}
    for m in wiring.flip_motor:
        w[m] = -w[m]
    return [w['FL'], w['FR'], w['RL'], w['RR']]


def classify(dx, dy, dth, fight):
    if fight:
        return 'fight'
    # สั่งเดินตรง 0.3 m ต้องเฉไม่เกิน 3 องศา  สั่งหมุน 1.5 rad ต้องเลื่อนไม่เกิน 3 cm
    moved, turned = math.hypot(dx, dy) > 0.03, abs(dth) > 0.05
    if not moved and not turned:
        return 'stop'
    if moved and turned:
        return 'arc'
    if moved:
        if abs(dy) > 0.3 * abs(dx):
            return 'other'
        return 'fwd' if dx > 0 else 'back'
    return 'left' if dth > 0 else 'right'


def run(vx, wz, wiring, invert):
    kin = MecanumKinematics(R, L, W)
    rates = node_rates(vx, wz, invert)
    sign = [-1 if inv else 1 for inv in invert]
    wa = actual_wheels(rates, wiring)
    # odom ของ node: ตัวนับ step ทั้ง 4 ช่อง x sign (ไม่รู้เรื่องสายไฟ)
    wo = [sign[i] * rates[i] / STEPS_PER_RAD for i in range(4)]
    fight = (abs(wa[0] - wa[2]) > 1e-6 and wa[0] * wa[2] < 0) or (wa[1] * wa[3] < 0)
    pa, po, path_a, path_o = (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), [], []
    for _ in range(int(T / DT)):
        # ล้อ mecanum ที่หมุนไม่สอดคล้องกัน ใช้ least-squares (forward) เป็นค่าประมาณ พื้นจริงจะไถล
        pa = integrate_pose(*pa, *[v * DT for v in kin.forward(wa)])
        po = integrate_pose(*po, *[v * DT for v in kin.forward(wo)])
        path_a.append([round(c, 4) for c in pa])
        path_o.append([round(c, 4) for c in po])
    return {
        'rates': rates, 'wheels': [round(v * STEPS_PER_RAD) for v in wa],
        'result': classify(*pa, fight), 'odom_result': classify(*po, False),
        'end': [round(c, 3) for c in pa], 'odom_end': [round(c, 3) for c in po],
        'path': path_a[::5], 'odom_path': path_o[::5],
    }


def evaluate(wiring, invert):
    runs = [run(vx, wz, wiring, invert) for _, vx, wz, _ in COMMANDS]
    ok = all(r['result'] == c[3] for r, c in zip(runs, COMMANDS))
    return runs, ok


def find_fix(wiring):
    """ลอง invert_wheels ทุกแบบ (ช่อง 2/3 ตั้งตามช่อง 0/1 ข้างเดียวกัน) หาแบบที่ผ่านครบ 4 คำสั่ง"""
    for inv_l, inv_r in itertools.product([False, True], repeat=2):
        invert = [inv_l, inv_r, inv_l, inv_r]
        if evaluate(wiring, invert)[1]:
            return invert
    return None


HW_FIX = {
    'swap': 'สลับขั้วต่อ STEP/DIR ระหว่างสองไดรเวอร์ (ซ้ายต้องอยู่ 25/26 ขวา 27/14) -- invert_wheels แก้ไม่ได้',
    'swap_both_rev': 'สลับขั้วต่อ STEP/DIR สองไดรเวอร์ แล้วตั้ง invert_wheels: [true, true, true, true]',
    'swap_left_rev': 'สลับขั้วต่อ STEP/DIR สองไดรเวอร์ แล้วตั้ง invert_wheels: [true, false, true, false]',
    'rl_rev': 'สลับคอยล์ (A+ <-> A-) ของมอเตอร์หลังซ้ายตัวเดียว -- config แก้ไม่ได้ ห้ามปล่อยไว้ มอเตอร์ร้อน',
    'fr_rev': 'สลับคอยล์ (A+ <-> A-) ของมอเตอร์หน้าขวาตัวเดียว -- config แก้ไม่ได้',
    'right_microstep': 'ตั้ง DIP microstep ของไดรเวอร์ขวาให้ตรงกับซ้าย (1600) ดูตารางบนตัวไดรเวอร์ -- config แก้ไม่ได้ (steps ตั้งได้ค่าเดียวทั้งหุ่น)',
    'right_dead': 'เช็คไฟ 24V และไฟ ALM ที่ไดรเวอร์ขวา, สาย PUL/DIR จาก GPIO 27/14, ENA ต่อขนานกับไดรเวอร์ซ้าย (GPIO 23)',
}


def main():
    default = [False] * 4
    report = []
    print(f'หุ่น r={R} L={L} W={W}  คำสั่ง V ต่อคำสั่ง:')
    for name, vx, wz, _ in COMMANDS:
        print(f'  {name:<20} vx={vx:+.2f} wz={wz:+.2f} -> V {" ".join(map(str, node_rates(vx, wz, default)))}')
    print()
    for wc in CASES:
        runs, ok = evaluate(wc, default)
        fix = None if ok else find_fix(wc)
        if ok:
            fix_text = 'ไม่ต้องแก้'
        elif fix:
            fix_text = f'invert_wheels: [{", ".join(str(v).lower() for v in fix)}]'
        else:
            fix_text = HW_FIX.get(wc.key, 'แก้ที่สายไฟ')
        fixed_runs = evaluate(wc, fix)[0] if fix else None
        print(f'== {wc.name}  ({wc.why})')
        for (cname, *_), r in zip(COMMANDS, runs):
            mark = 'OK ' if r['result'] == _[-1] else 'ผิด'
            odom = '' if r['odom_result'] == r['result'] else f'  <- odom บอกว่า{LABEL[r["odom_result"]]}!'
            print(f'   [{mark}] สั่ง{cname:<18} หุ่นจริง: {LABEL[r["result"]]:<12}{odom}')
        print(f'   แก้: {fix_text}\n')
        report.append({'key': wc.key, 'name': wc.name, 'why': wc.why, 'ok': ok, 'fix': fix_text,
                       'fix_by_config': bool(fix), 'runs': runs, 'fixed_runs': fixed_runs})

    # ทำไม max_vy ต้องคงเป็น 0: ช่องหลัง (2, 3) ไม่ได้ต่อ ล้อหลังเลยหมุนตามช่องหน้า
    kin = MecanumKinematics(R, L, W)
    vy_rates = [int(round(w * STEPS_PER_RAD)) for w in kin.inverse(0.0, 0.1, 0.0)]
    wa = actual_wheels(vy_rates, CASES[0])
    _, vy, wz = kin.forward(wa)
    print(f'ถ้าเปิด max_vy แล้วสั่งเดินข้างซ้าย vy=0.10: V {" ".join(map(str, vy_rates))} '
          f'-> ล้อจริง {[round(v * STEPS_PER_RAD) for v in wa]} -> หุ่นหมุนตัว wz={wz:+.2f} rad/s '
          f'(ไม่ได้เดินข้าง vy={vy:+.2f})  => ต้องคง max_vy: 0.0\n')

    # เช็คตัวเอง: กรณีต่อถูกต้องต้องผ่าน และ odom ต้องตรงกับหุ่นจริง
    assert report[0]['ok'], 'สมการ/เครื่องจำลองผิด: ต่อถูกแล้วยังไม่ผ่าน'
    for key, invert in (('swap_both_rev', [True] * 4), ('swap_left_rev', [True, False, True, False])):
        wc = next(c for c in CASES if c.key == key)
        wc.ch_left, wc.ch_right = wc.ch_right, wc.ch_left      # สลับขั้วต่อกลับให้ถูก
        assert evaluate(wc, invert)[1], f'วิธีแก้ข้อ {key} ที่เขียนไว้ใช้ไม่ได้'
        wc.ch_left, wc.ch_right = wc.ch_right, wc.ch_left
    assert all(r['odom_result'] == r['result'] for r in report[0]['runs'])

    if '--html' in sys.argv:
        tpl = open(os.path.join(HERE, 'sim_two_drivers.template.html'), encoding='utf-8').read()
        data = {'cases': report, 'commands': [[c[0], c[1], c[2], c[3]] for c in COMMANDS],
                'labels': LABEL, 'dt': DT * 5, 'geom': [L, W]}
        out = os.path.join(HERE, 'sim_two_drivers.html')
        with open(out, 'w', encoding='utf-8') as f:
            f.write(tpl.replace('/*DATA*/null', json.dumps(data, ensure_ascii=False)))
        print('เขียน', out)


if __name__ == '__main__':
    main()
