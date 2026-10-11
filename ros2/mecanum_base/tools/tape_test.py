"""Step-loss check with tape on the wheels: repeated moves of exactly 1600 steps (1 wheel rev at microstep 8).
Firmware is velocity-only, so the host closes the loop on the board's pulse counter (wheel 0) and brakes into
ABSOLUTE targets start + k*1600, so small overshoots never accumulate.
Timeline: E 1, hold INITIAL_HOLD s (user notes tape position), then the PLAN groups, holding HOLD s after each move.
Ends (or on SIGTERM/SIGINT) with V 0 + E 0.  Log: /tmp/tape_test.log"""
import math
import signal
import time

import serial

PORT = '/dev/serial/by-id/usb-1a86_USB_Single_Serial_58EA045139-if00'
REV = 1600
ACCEL = 3000.0          # step/s^2, sent to the board with A
LATENCY = 0.06          # s, status age + serial delay budget for braking
CREEP = 40.0            # step/s final approach
INITIAL_HOLD = 40.0
HOLD = 8.0
PLAN = [('slow forward', +1, 800.0, 5), ('fast forward', +1, 1600.0, 5), ('fast backward', -1, 1600.0, 5)]

log = open('/tmp/tape_test.log', 'w', buffering=1)
stop = False


def on_signal(*_):
    global stop
    stop = True


signal.signal(signal.SIGTERM, on_signal)
signal.signal(signal.SIGINT, on_signal)

ser = serial.Serial()
ser.port, ser.baudrate, ser.timeout = PORT, 115200, 0.005
ser.dtr = False
ser.rts = False
ser.open()

T0 = time.monotonic()
buf = b''
pos = [0, 0, 0, 0]
vel = [0, 0, 0, 0]
sysf = 0
ready = False


def say(msg):
    log.write(f'{time.strftime("%H:%M:%S")} {time.monotonic() - T0:7.2f}  {msg}\n')


def pump():
    global buf, pos, vel, sysf, ready
    buf += ser.read(4096)
    while b'\n' in buf:
        raw, buf = buf.split(b'\n', 1)
        line = raw.decode('ascii', 'replace').strip()
        if line.startswith('W '):
            f = line.split()
            if len(f) == 11:
                sysf = int(f[2])
                pos = [int(x) for x in f[3:7]]
                vel = [int(x) for x in f[7:11]]
        elif line:
            if 'ready' in line:
                ready = True
            if not line.startswith(('ets ', 'rst:', 'configsip', 'clk_drv', 'mode:', 'load:', 'entry')):
                say(f'board: {line}')


def send(line):
    ser.write((line + '\n').encode('ascii'))


def wait(sec, keepalive=True):
    end = time.monotonic() + sec
    while time.monotonic() < end and not stop:
        if keepalive:
            send('V 0 0 0 0')
        pump()
        time.sleep(0.02)


def move_to(target, direction, vmax):
    last_send = 0.0
    t_start = time.monotonic()
    while not stop:
        pump()
        rem = (target - pos[0]) * direction
        if rem <= 0:
            break
        a_l = ACCEL * LATENCY
        v = min(vmax, -a_l + math.sqrt(a_l * a_l + 2.0 * ACCEL * rem))
        v = max(v, CREEP)
        now = time.monotonic()
        if now - last_send >= 0.02:
            r = int(round(v)) * direction
            send(f'V {r} {r} {r} {r}')
            last_send = now
        if now - t_start > 15.0:
            say('move timeout')
            break
        time.sleep(0.004)
    send('V 0 0 0 0')
    t = time.monotonic()
    while time.monotonic() - t < 1.0:          # wait until the board reports standstill
        pump()
        if not any(vel):
            break
        send('V 0 0 0 0')
        time.sleep(0.02)


try:
    while not ready and time.monotonic() - T0 < 4.0:
        pump()
        time.sleep(0.02)
    send('F 50')
    send(f'A 3200 {int(ACCEL)}')
    send('E 1')
    wait(0.5)
    say(f'driver enabled sys={sysf} pos={pos}  -> holding {INITIAL_HOLD:.0f} s for the tape reference')
    wait(INITIAL_HOLD)

    origin = pos[0]
    k = 0
    for label, direction, vmax, count in PLAN:
        for i in range(count):
            if stop:
                break
            k += direction
            target = origin + k * REV
            t = time.monotonic()
            move_to(target, direction, vmax)
            say(f'{label} {i + 1}/{count} ({vmax:.0f} step/s) target {target - origin:+6d}  '
                f'got {[p - origin for p in pos]}  err {pos[0] - target:+d}  {time.monotonic() - t:.2f} s')
            wait(HOLD)
        if stop:
            break
    say('plan done' if not stop else 'stopped by signal')
finally:
    send('V 0 0 0 0')
    wait(0.8)
    send('E 0')
    time.sleep(0.3)
    pump()
    say(f'END driver off sys={sysf} pos={pos}')
    ser.close()
