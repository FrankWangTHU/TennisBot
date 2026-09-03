from tennisbot.navigation.driver import DryRunDriver, UdpChassisDriver, create_driver
from tennisbot.navigation.models import VelocityCommand


class FakeSocket:
    def __init__(self) -> None:
        self.sent = []
        self.timeout = 0.08

    def sendto(self, packet, address):
        self.sent.append((packet, address))

    def gettimeout(self):
        return self.timeout

    def setblocking(self, _enabled):
        pass

    def settimeout(self, timeout):
        self.timeout = timeout

    def recvfrom(self, _size):
        raise BlockingIOError


def test_udp_drive_packet_contains_token_sequence_and_body() -> None:
    driver = UdpChassisDriver("192.168.4.1", 5005, "secret")
    driver.socket = FakeSocket()
    driver.enabled = True
    driver.send(VelocityCommand(0.05, -0.02, 0.1))
    packet, address = driver.socket.sent[0]
    assert address == ("192.168.4.1", 5005)
    assert packet == b"DRIVE secret 1 0.0500 -0.0200 0.1000\n"


def test_driver_factory_requires_explicit_motion_permission() -> None:
    config = {"driver": {"type": "udp", "host": "192.168.4.1"}}
    assert isinstance(create_driver(config, allow_motion=False), DryRunDriver)
    assert isinstance(create_driver(config, allow_motion=True), UdpChassisDriver)
