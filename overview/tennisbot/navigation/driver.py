from __future__ import annotations

import time
import socket
from abc import ABC, abstractmethod

from tennisbot.navigation.models import VelocityCommand


class ChassisDriver(ABC):
    @abstractmethod
    def connect(self) -> None: ...
    @abstractmethod
    def enable(self) -> None: ...
    @abstractmethod
    def send(self, command: VelocityCommand) -> None: ...
    @abstractmethod
    def stop(self) -> None: ...
    @abstractmethod
    def close(self) -> None: ...


class DryRunDriver(ChassisDriver):
    def __init__(self) -> None:
        self.enabled = False
        self.last_command = VelocityCommand()

    def connect(self) -> None:
        print("[dry-run] chassis output is disabled")

    def enable(self) -> None:
        self.enabled = True
        print("[dry-run] ENABLE")

    def send(self, command: VelocityCommand) -> None:
        self.last_command = command if self.enabled else VelocityCommand()

    def stop(self) -> None:
        self.last_command = VelocityCommand()

    def close(self) -> None:
        self.stop()
        self.enabled = False


class SerialChassisDriver(ChassisDriver):
    def __init__(self, port: str, baudrate: int = 115200, timeout_s: float = 0.25, enable_timeout_s: float = 2.0) -> None:
        self.port = port
        self.baudrate = int(baudrate)
        self.timeout_s = float(timeout_s)
        self.enable_timeout_s = float(enable_timeout_s)
        self.serial = None
        self.enabled = False

    def _write(self, line: str) -> None:
        if self.serial is None:
            raise RuntimeError("Serial chassis is not connected")
        self.serial.write((line.rstrip() + "\n").encode("ascii"))

    def _request(self, line: str, expected_prefix: str = "OK:", timeout_s: float | None = None) -> str:
        self._write(line)
        deadline = time.monotonic() + (self.timeout_s if timeout_s is None else timeout_s)
        while time.monotonic() < deadline:
            raw = self.serial.readline()
            if not raw:
                continue
            reply = raw.decode("utf-8", "replace").strip()
            if reply.startswith(expected_prefix):
                return reply
            if reply.startswith("ERR:"):
                raise RuntimeError(f"ESP32 rejected {line!r}: {reply}")
        raise TimeoutError(f"ESP32 did not answer {line!r} on {self.port}")

    def connect(self) -> None:
        try:
            import serial
        except ImportError as exc:
            raise RuntimeError("请先安装 pyserial: pip install pyserial") from exc
        self.serial = serial.Serial(self.port, self.baudrate, timeout=0.03, write_timeout=0.2)
        time.sleep(0.8)
        self.serial.reset_input_buffer()
        self._request("PING", "OK:PING")

    def enable(self) -> None:
        self._request("ENABLE", "OK:ENABLE", self.enable_timeout_s)
        self.enabled = True

    def send(self, command: VelocityCommand) -> None:
        if self.enabled:
            self._write(f"DRIVE {command.vx:.4f} {command.vy:.4f} {command.omega:.4f}")

    def stop(self) -> None:
        if self.serial is not None:
            self._write("STOP")

    def close(self) -> None:
        if self.serial is None:
            return
        try:
            self.stop()
            self._request("DISABLE", "OK:DISABLE")
        except Exception:
            pass
        finally:
            self.enabled = False
            self.serial.close()
            self.serial = None


class UdpChassisDriver(ChassisDriver):
    def __init__(self, host: str, port: int = 5005, token: str = "change-me-tennisbot", timeout_s: float = 0.3, enable_timeout_s: float = 2.0) -> None:
        self.address = (host, int(port))
        self.token = token
        self.timeout_s = float(timeout_s)
        self.enable_timeout_s = float(enable_timeout_s)
        self.socket: socket.socket | None = None
        self.sequence = 0
        self.enabled = False

    def _packet(self, command: str, *values: object) -> bytes:
        self.sequence += 1
        fields = [command, self.token, str(self.sequence), *(str(value) for value in values)]
        return (" ".join(fields) + "\n").encode("ascii")

    def _request(self, command: str, expected: str, timeout_s: float | None = None) -> str:
        if self.socket is None:
            raise RuntimeError("UDP chassis is not connected")
        packet = self._packet(command)
        deadline = time.monotonic() + (self.timeout_s if timeout_s is None else timeout_s)
        enable_started = False
        self.socket.sendto(packet, self.address)
        while time.monotonic() < deadline:
            try:
                data, sender = self.socket.recvfrom(256)
            except socket.timeout:
                # ENABLE blocks the ESP32 loop while four motors initialize.
                # Once its progress reply arrives, do not queue duplicate ENABLE packets.
                if not enable_started:
                    self.socket.sendto(packet, self.address)
                continue
            reply = data.decode("ascii", "replace").strip()
            if sender[0] == self.address[0] and reply.startswith("INFO:ENABLE:"):
                enable_started = True
                continue
            if sender[0] == self.address[0] and reply.startswith(expected):
                return reply
            if sender[0] == self.address[0] and reply.startswith("ERR:"):
                raise RuntimeError(f"ESP32 rejected {command!r}: {reply}")
        raise TimeoutError(f"ESP32 did not answer {command!r} at {self.address}")

    def connect(self) -> None:
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.socket.settimeout(min(0.08, self.timeout_s))
        self._request("PING", "OK:PING")

    def enable(self) -> None:
        self._request("ENABLE", "OK:ENABLE", self.enable_timeout_s)
        self.enabled = True

    def send(self, command: VelocityCommand) -> None:
        if self.socket is not None and self.enabled:
            packet = self._packet("DRIVE", f"{command.vx:.4f}", f"{command.vy:.4f}", f"{command.omega:.4f}")
            self.socket.sendto(packet, self.address)

    def stop(self) -> None:
        if self.socket is not None and self.enabled:
            try:
                self._request("STOP", "OK:STOP")
            except Exception:
                pass

    def close(self) -> None:
        if self.socket is None:
            return
        try:
            self.stop()
            if self.enabled:
                self._request("DISABLE", "OK:DISABLE")
        except Exception:
            pass
        finally:
            self.enabled = False
            self.socket.close()
            self.socket = None


def create_driver(config: dict, allow_motion: bool) -> ChassisDriver:
    driver_cfg = config.get("driver", {})
    kind = str(driver_cfg.get("type", "dry_run")).lower()
    if not allow_motion or kind == "dry_run":
        return DryRunDriver()
    if kind == "serial":
        return SerialChassisDriver(str(driver_cfg.get("port", "COM12")), int(driver_cfg.get("baudrate", 115200)), float(driver_cfg.get("command_timeout_s", 0.25)), float(driver_cfg.get("enable_timeout_s", 2.0)))
    if kind == "udp":
        return UdpChassisDriver(str(driver_cfg.get("host", "192.168.4.1")), int(driver_cfg.get("udp_port", 5005)), str(driver_cfg.get("token", "change-me-tennisbot")), float(driver_cfg.get("command_timeout_s", 0.3)), float(driver_cfg.get("enable_timeout_s", 2.0)))
    raise ValueError(f"Unsupported chassis driver type: {kind}")
