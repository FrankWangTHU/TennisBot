from __future__ import annotations

import time
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
    def __init__(self, port: str, baudrate: int = 115200, timeout_s: float = 0.25) -> None:
        self.port = port
        self.baudrate = int(baudrate)
        self.timeout_s = float(timeout_s)
        self.serial = None
        self.enabled = False

    def _write(self, line: str) -> None:
        if self.serial is None:
            raise RuntimeError("Serial chassis is not connected")
        self.serial.write((line.rstrip() + "\n").encode("ascii"))

    def _request(self, line: str, expected_prefix: str = "OK:") -> str:
        self._write(line)
        deadline = time.monotonic() + self.timeout_s
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
        self._request("ENABLE", "OK:ENABLE")
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


def create_driver(config: dict, allow_motion: bool) -> ChassisDriver:
    driver_cfg = config.get("driver", {})
    if str(driver_cfg.get("type", "dry_run")).lower() != "serial" or not allow_motion:
        return DryRunDriver()
    return SerialChassisDriver(str(driver_cfg.get("port", "COM12")), int(driver_cfg.get("baudrate", 115200)), float(driver_cfg.get("command_timeout_s", 0.25)))
