"""Change both values before using the robot outside the lab."""

AP_SSID = "TennisBot"
AP_PASSWORD = "TennisBot2026"  # WPA password: at least 8 characters
CONTROL_TOKEN = "change-me-tennisbot"
UDP_PORT = 5005
# Windows scheduling and 2.4 GHz jitter can occasionally exceed 300 ms.
UDP_COMMAND_WATCHDOG_MS = 600
