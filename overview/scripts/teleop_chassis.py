"""Low-speed keyboard teleoperation for validating PC-to-chassis control on Windows."""

from __future__ import annotations

import argparse
import time

import _bootstrap  # noqa: F401

from tennisbot.navigation.driver import SerialChassisDriver, UdpChassisDriver
from tennisbot.navigation.models import VelocityCommand


def main() -> None:
    import msvcrt

    parser = argparse.ArgumentParser(description="Safe keyboard chassis control")
    parser.add_argument("--transport", choices=("serial", "udp"), default="serial")
    parser.add_argument("--port", default="COM12", help="serial COM port")
    parser.add_argument("--host", default="192.168.4.1")
    parser.add_argument("--udp-port", type=int, default=5005)
    parser.add_argument("--token", default="change-me-tennisbot")
    parser.add_argument("--speed", type=float, default=0.05)
    parser.add_argument("--omega", type=float, default=0.15)
    args = parser.parse_args()
    if not 0 < args.speed <= 0.10 or not 0 < args.omega <= 0.25:
        raise SystemExit("安全限幅：speed 必须 <=0.10 m/s，omega 必须 <=0.25 rad/s")

    if args.transport == "serial":
        driver = SerialChassisDriver(args.port)
    else:
        driver = UdpChassisDriver(args.host, args.udp_port, args.token)

    commands = {
        "w": VelocityCommand(vx=args.speed),
        "s": VelocityCommand(vx=-args.speed),
        "a": VelocityCommand(vy=args.speed),
        "d": VelocityCommand(vy=-args.speed),
        "z": VelocityCommand(omega=args.omega),
        "c": VelocityCommand(omega=-args.omega),
    }
    armed = False
    active = VelocityCommand()
    active_until = 0.0
    try:
        driver.connect()
        print("连接成功。G=使能，W/S=前后，A/D=左右，Z/C=旋转，X=停车，ESC=退出")
        print("每次按键只保持 0.20 秒；持续运动需要按住按键。")
        while True:
            while msvcrt.kbhit():
                key = msvcrt.getwch().lower()
                if key == "\x1b":
                    return
                if key == "g" and not armed:
                    try:
                        driver.enable()
                        armed = True
                        print("ARMED")
                    except (RuntimeError, TimeoutError) as exc:
                        print(f"使能失败：{exc}。确认网络后再按 G。")
                elif key == "x":
                    active = VelocityCommand()
                    active_until = 0.0
                    driver.stop()
                    armed = False
                    print("STOPPED / DISARMED")
                elif key in commands and armed:
                    active = commands[key]
                    active_until = time.monotonic() + 0.20
            command = active if armed and time.monotonic() < active_until else VelocityCommand()
            try:
                driver.send(command)
            except (RuntimeError, OSError) as exc:
                armed = False
                active = VelocityCommand()
                print(f"控制链路已进入安全停车：{exc}。重新按 G 恢复。")
            time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    finally:
        driver.close()
        print("底盘已停车并失能。")


if __name__ == "__main__":
    main()
