"""ESP32-S3 USB serial -> Mecanum chassis bridge (MicroPython)."""

import sys
import time
import uselect

from esp32 import CAN
from chassis_control import MecanumChassis
from motor_lib import MotorBus
from robot_config import CAN_BAUDRATE, CAN_BUS_ID, CAN_RX, CAN_TX

COMMAND_WATCHDOG_MS = 300

can = CAN(CAN_BUS_ID, mode=CAN.NORMAL, baudrate=CAN_BAUDRATE, tx=CAN_TX, rx=CAN_RX)
can.clear_rx_queue()
chassis = MecanumChassis(MotorBus(can))
poller = uselect.poll()
poller.register(sys.stdin, uselect.POLLIN)
enabled = False
last_drive_ms = time.ticks_ms()
watchdog_reported = False


def reply(text):
    print(text)


def handle(line):
    global enabled, last_drive_ms, watchdog_reported
    parts = line.strip().split()
    if not parts:
        return
    command = parts[0].upper()
    if command == "PING":
        reply("OK:PING:ready")
    elif command == "ENABLE":
        if not enabled:
            chassis.prepare()
            enabled = True
        chassis.stop()
        last_drive_ms = time.ticks_ms()
        watchdog_reported = False
        reply("OK:ENABLE")
    elif command == "DRIVE":
        if not enabled:
            reply("ERR:DISABLED")
            return
        if len(parts) != 4:
            reply("ERR:DRIVE requires vx vy omega")
            return
        vx, vy, omega = (float(value) for value in parts[1:])
        chassis.drive(vx, vy, omega)
        last_drive_ms = time.ticks_ms()
        watchdog_reported = False
    elif command == "STOP":
        chassis.stop()
        reply("OK:STOP")
    elif command == "DISABLE":
        chassis.stop()
        chassis.disable()
        enabled = False
        reply("OK:DISABLE")
    elif command == "STATUS":
        age = time.ticks_diff(time.ticks_ms(), last_drive_ms)
        reply("OK:STATUS:enabled=%d,drive_age_ms=%d" % (1 if enabled else 0, age))
    else:
        reply("ERR:UNKNOWN_COMMAND")


reply("READY:TennisBot serial chassis bridge")
try:
    while True:
        if poller.poll(10):
            try:
                handle(sys.stdin.readline())
            except Exception as exc:
                reply("ERR:%s" % exc)
        if enabled and time.ticks_diff(time.ticks_ms(), last_drive_ms) > COMMAND_WATCHDOG_MS:
            chassis.stop()
            if not watchdog_reported:
                reply("WARN:WATCHDOG:stopped")
                watchdog_reported = True
except Exception as exc:
    reply("FATAL:%s" % exc)
finally:
    try:
        chassis.stop()
        chassis.disable()
    except Exception:
        pass
