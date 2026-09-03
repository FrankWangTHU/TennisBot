"""ESP32-S3 Wi-Fi UDP / USB serial -> Mecanum chassis bridge."""

import sys
import time
import uselect
import network
import socket

from esp32 import CAN
from chassis_control import MecanumChassis
from motor_lib import MotorBus
from robot_config import CAN_BAUDRATE, CAN_BUS_ID, CAN_RX, CAN_TX
from wifi_config import AP_PASSWORD, AP_SSID, CONTROL_TOKEN, UDP_PORT
from control_config import CONTROL_MODE

COMMAND_WATCHDOG_MS = 300


def start_access_point():
    interface_id = network.WLAN.IF_AP if hasattr(network.WLAN, "IF_AP") else network.AP_IF
    wlan = network.WLAN(interface_id)
    wlan.active(True)
    wlan.config(ssid=AP_SSID, password=AP_PASSWORD, max_clients=2)
    while not wlan.active():
        time.sleep_ms(50)
    return wlan


can = CAN(CAN_BUS_ID, mode=CAN.NORMAL, baudrate=CAN_BAUDRATE, tx=CAN_TX, rx=CAN_RX)
can.clear_rx_queue()
chassis = MecanumChassis(MotorBus(can))


def run_ps2_mode():
    from ps2_control import ps2_loop
    from ps2_lib import PS2Controller, PS2Receiver
    from robot_config import PS2_CLK, PS2_CS, PS2_DI, PS2_DO

    chassis.prepare()
    controller = PS2Controller(di=PS2_DI, do=PS2_DO, cs=PS2_CS, clk=PS2_CLK)
    controller.init_vibration()
    receiver = PS2Receiver(controller, 30, True)
    receiver.start()
    try:
        # PS2 原示例只有 data 不为 None 时才访问 serial；此模式不接收相机串口。
        ps2_loop(chassis, None, receiver, {"value": None}, None)
    finally:
        receiver.stop()
        chassis.stop()
        chassis.disable()


if CONTROL_MODE == "ps2":
    print("READY:TennisBot PS2 mode")
    run_ps2_mode()
    raise SystemExit
if CONTROL_MODE not in ("serial", "udp"):
    raise ValueError("CONTROL_MODE must be serial, udp, or ps2")

ap = None
udp = None
if CONTROL_MODE == "udp":
    ap = start_access_point()
    udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp.bind(("0.0.0.0", UDP_PORT))
    udp.setblocking(False)
poller = uselect.poll()
poller.register(sys.stdin, uselect.POLLIN)

enabled = False
active_peer = None
last_udp_seq = -1
last_drive_ms = time.ticks_ms()
watchdog_reported = False


def serial_reply(text):
    print(text)


def udp_reply(text, peer):
    if udp is None:
        return
    udp.sendto(text.encode("ascii"), peer)


def enable_chassis():
    global enabled, last_drive_ms, watchdog_reported
    if not enabled:
        chassis.prepare()
        enabled = True
    chassis.stop()
    last_drive_ms = time.ticks_ms()
    watchdog_reported = False


def handle_serial(line):
    """Legacy USB protocol kept for setup and recovery."""
    global enabled, active_peer, last_drive_ms, watchdog_reported
    parts = line.strip().split()
    if not parts:
        return
    command = parts[0].upper()
    if command == "PING":
        serial_reply("OK:PING:ready")
    elif command == "ENABLE":
        active_peer = "serial"
        enable_chassis()
        serial_reply("OK:ENABLE")
    elif command == "DRIVE":
        if not enabled or active_peer != "serial":
            serial_reply("ERR:DISABLED_OR_BUSY")
            return
        if len(parts) != 4:
            serial_reply("ERR:DRIVE requires vx vy omega")
            return
        vx, vy, omega = (float(value) for value in parts[1:])
        chassis.drive(vx, vy, omega)
        last_drive_ms = time.ticks_ms()
        watchdog_reported = False
    elif command == "STOP":
        chassis.stop()
        serial_reply("OK:STOP")
    elif command == "DISABLE":
        chassis.stop()
        chassis.disable()
        enabled = False
        active_peer = None
        serial_reply("OK:DISABLE")
    elif command == "STATUS":
        age = time.ticks_diff(time.ticks_ms(), last_drive_ms)
        serial_reply("OK:STATUS:enabled=%d,drive_age_ms=%d" % (enabled, age))
    else:
        serial_reply("ERR:UNKNOWN_COMMAND")


def handle_udp(payload, peer):
    """Authenticated, sequenced protocol: COMMAND token sequence [values]."""
    global enabled, active_peer, last_udp_seq, last_drive_ms, watchdog_reported
    try:
        parts = payload.decode("ascii").strip().split()
        if len(parts) < 3:
            raise ValueError("short packet")
        command, token, sequence_text = parts[:3]
        command = command.upper()
        if token != CONTROL_TOKEN:
            udp_reply("ERR:AUTH", peer)
            return
        sequence = int(sequence_text)
        if command == "PING":
            udp_reply("OK:PING %d" % sequence, peer)
            return
        if command == "ENABLE":
            if active_peer not in (None, peer):
                udp_reply("ERR:BUSY", peer)
                return
            active_peer = peer
            last_udp_seq = sequence
            enable_chassis()
            udp_reply("OK:ENABLE %d" % sequence, peer)
            return
        if active_peer != peer or not enabled:
            udp_reply("ERR:DISABLED_OR_BUSY", peer)
            return
        if sequence <= last_udp_seq:
            udp_reply("ERR:STALE", peer)
            return
        last_udp_seq = sequence
        if command == "DRIVE":
            if len(parts) != 6:
                raise ValueError("DRIVE requires vx vy omega")
            vx, vy, omega = (float(value) for value in parts[3:])
            chassis.drive(vx, vy, omega)
            last_drive_ms = time.ticks_ms()
            watchdog_reported = False
        elif command == "STOP":
            chassis.stop()
            udp_reply("OK:STOP %d" % sequence, peer)
        elif command == "DISABLE":
            chassis.stop()
            chassis.disable()
            enabled = False
            active_peer = None
            udp_reply("OK:DISABLE %d" % sequence, peer)
        elif command == "STATUS":
            age = time.ticks_diff(time.ticks_ms(), last_drive_ms)
            udp_reply("OK:STATUS %d enabled=%d drive_age_ms=%d" % (sequence, enabled, age), peer)
        else:
            udp_reply("ERR:UNKNOWN_COMMAND", peer)
    except Exception as exc:
        udp_reply("ERR:%s" % exc, peer)


serial_reply("READY:TennisBot %s chassis bridge" % CONTROL_MODE)
if ap is not None:
    ip = ap.ifconfig()[0]
    serial_reply("WIFI:ssid=%s ip=%s udp=%d" % (AP_SSID, ip, UDP_PORT))
try:
    while True:
        if poller.poll(0):
            try:
                handle_serial(sys.stdin.readline())
            except Exception as exc:
                serial_reply("ERR:%s" % exc)
        if udp is not None:
            try:
                data, sender = udp.recvfrom(160)
                handle_udp(data, sender)
            except OSError:
                pass
        if enabled and time.ticks_diff(time.ticks_ms(), last_drive_ms) > COMMAND_WATCHDOG_MS:
            chassis.stop()
            chassis.disable()
            enabled = False
            active_peer = None
            if not watchdog_reported:
                serial_reply("WARN:WATCHDOG:stopped_and_disabled")
                watchdog_reported = True
        time.sleep_ms(5)
except Exception as exc:
    serial_reply("FATAL:%s" % exc)
finally:
    try:
        chassis.stop()
        chassis.disable()
    except Exception:
        pass
