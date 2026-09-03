from __future__ import annotations

import argparse
import time

import _bootstrap  # noqa: F401

from tennisbot.navigation.driver import UdpChassisDriver
from tennisbot.navigation.models import VelocityCommand


def main() -> None:
    parser = argparse.ArgumentParser(description="Safe ESP32 Wi-Fi UDP chassis test")
    parser.add_argument("--host", default="192.168.4.1")
    parser.add_argument("--port", type=int, default=5005)
    parser.add_argument("--token", default="change-me-tennisbot")
    parser.add_argument("--axis", choices=("vx", "vy", "omega"))
    parser.add_argument("--value", type=float, default=0.05)
    parser.add_argument("--duration", type=float, default=0.5)
    parser.add_argument("--enable-motion", action="store_true")
    args = parser.parse_args()
    if abs(args.value) > (0.10 if args.axis != "omega" else 0.25):
        raise SystemExit("首次测试限幅：线速度 <=0.10 m/s，角速度 <=0.25 rad/s")
    if not 0.05 <= args.duration <= 2.0:
        raise SystemExit("duration 必须在 0.05..2.0 秒")

    driver = UdpChassisDriver(args.host, args.port, args.token)
    try:
        driver.connect()
        print(f"UDP PING 成功：ESP32 {args.host}:{args.port}")
        if not args.enable_motion:
            print("未带 --enable-motion：只测试无线通信，不使能电机。")
            return
        if args.axis is None:
            raise SystemExit("运动测试还需要 --axis vx|vy|omega")
        values = {"vx": 0.0, "vy": 0.0, "omega": 0.0}
        values[args.axis] = args.value
        print("确认车轮已架空；3 秒后执行。Ctrl+C 可取消。")
        for remaining in (3, 2, 1):
            print(remaining)
            time.sleep(1)
        driver.enable()
        deadline = time.monotonic() + args.duration
        command = VelocityCommand(**values)
        while time.monotonic() < deadline:
            driver.send(command)
            time.sleep(0.05)
        print("动作完成并停车。")
    finally:
        driver.close()


if __name__ == "__main__":
    main()
