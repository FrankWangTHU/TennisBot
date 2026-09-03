from __future__ import annotations

import argparse
import time

import _bootstrap  # noqa: F401

from tennisbot.navigation.driver import UdpChassisDriver


def main() -> None:
    parser = argparse.ArgumentParser(description="Measure ESP32 UDP RTT without enabling motors")
    parser.add_argument("--host", default="192.168.4.1")
    parser.add_argument("--port", type=int, default=5005)
    parser.add_argument("--token", default="change-me-tennisbot")
    parser.add_argument("--count", type=int, default=200)
    args = parser.parse_args()
    driver = UdpChassisDriver(args.host, args.port, args.token, timeout_s=0.3)
    samples = []
    failures = 0
    try:
        driver.connect()
        for index in range(args.count):
            try:
                rtt_ms = driver.ping() * 1000.0
                samples.append(rtt_ms)
                if (index + 1) % 20 == 0:
                    print(f"{index + 1}/{args.count}: latest={rtt_ms:.1f} ms")
            except (RuntimeError, TimeoutError, OSError) as exc:
                failures += 1
                print(f"{index + 1}/{args.count}: failed: {exc}")
            time.sleep(0.05)
    finally:
        driver.close()
    if not samples:
        raise SystemExit("没有收到任何 UDP 回复")
    ordered = sorted(samples)
    p95 = ordered[int(0.95 * (len(ordered) - 1))]
    print(
        f"result: received={len(samples)}/{args.count}, lost={failures}, "
        f"avg={sum(samples) / len(samples):.1f} ms, p95={p95:.1f} ms, max={max(samples):.1f} ms"
    )


if __name__ == "__main__":
    main()
