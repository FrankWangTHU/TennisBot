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
