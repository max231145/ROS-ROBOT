"""Diagnostic spin that mimics test_one_driver_4motors (1600 step/s, back and forth) using the new firmware.
Runs DUR seconds, then V 0 + E 0. SIGTERM/SIGINT also end it cleanly with the driver off.
Log: /tmp/diag_spin.log (non-W board lines + last W line after every phase)."""
import signal
import sys
import time

import serial

PORT = '/dev/serial/by-id/usb-1a86_USB_Single_Serial_58EA045139-if00'
DUR = float(sys.argv[1]) if len(sys.argv) > 1 else 240.0
RATE = 1600
PHASES = [(+RATE, 1.5), (0, 1.0), (-RATE, 1.5), (0, 1.0)]

log = open('/tmp/diag_spin.log', 'w', buffering=1)
stop = False


def on_signal(*_):
    global stop
    stop = True


signal.signal(signal.SIGTERM, on_signal)
signal.signal(signal.SIGINT, on_signal)

ser = serial.Serial()
ser.port, ser.baudrate, ser.timeout = PORT, 115200, 0.02
ser.dtr = False
ser.rts = False
ser.open()

T0 = time.monotonic()
buf = b''
last_w = ''
ready = False


def stamp():
    return f'{time.monotonic() - T0:7.2f}'


def pump():
    global buf, last_w, ready
    buf += ser.read(4096)
    while b'\n' in buf:
        raw, buf = buf.split(b'\n', 1)
        line = raw.decode('ascii', 'replace').strip()
        if not line:
            continue
        if line.startswith('W '):
            last_w = line
        else:
            if 'ready' in line:
                ready = True
            log.write(f'{stamp()} {line}\n')


def send(line):
    ser.write((line + '\n').encode('ascii'))


while not ready and time.monotonic() - T0 < 4.0:
    pump()
    time.sleep(0.02)
log.write(f'{stamp()} boot ready={ready}\n')

send('F 10')
send('A 3200 3000')
send('E 1')
time.sleep(0.3)
pump()
log.write(f'{stamp()} after E 1: {last_w}\n')

cycles = 0
try:
    while not stop and time.monotonic() - T0 < DUR:
        for v, d in PHASES:
            end = time.monotonic() + d
            while time.monotonic() < end and not stop:
                send(f'V {v} {v} {v} {v}')
                pump()
                time.sleep(0.05)
            log.write(f'{stamp()} phase V {v:+5d} done: {last_w}\n')
            if stop:
                break
        cycles += 1
finally:
    send('V 0 0 0 0')
    t = time.monotonic()
    while time.monotonic() - t < 1.0:
        pump()
        time.sleep(0.05)
    send('E 0')
    time.sleep(0.3)
    pump()
    log.write(f'{stamp()} END cycles={cycles} driver off: {last_w}\n')
    ser.close()
