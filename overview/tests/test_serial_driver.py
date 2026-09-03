from tennisbot.navigation.driver import SerialChassisDriver
from tennisbot.navigation.models import VelocityCommand


class FakeSerial:
    def __init__(self) -> None:
        self.writes = []

    def write(self, payload: bytes) -> None:
        self.writes.append(payload)


def test_serial_driver_formats_original_chassis_velocity_units() -> None:
    driver = SerialChassisDriver("COM_TEST")
    driver.serial = FakeSerial()
    driver.enabled = True
    driver.send(VelocityCommand(0.05, -0.02, 0.1))
    assert driver.serial.writes == [b"DRIVE 0.0500 -0.0200 0.1000\n"]


def test_enable_uses_long_motor_initialization_timeout() -> None:
    class RecordingDriver(SerialChassisDriver):
        def _request(self, line, expected_prefix="OK:", timeout_s=None):
            self.recorded = (line, expected_prefix, timeout_s)
            return "OK:ENABLE"

    driver = RecordingDriver("COM_TEST", enable_timeout_s=2.0)
    driver.enable()
    assert driver.recorded == ("ENABLE", "OK:ENABLE", 2.0)
    assert driver.enabled
