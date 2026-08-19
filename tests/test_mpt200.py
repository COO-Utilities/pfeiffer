"""Unit tests for src/mpt200.py"""
from unittest.mock import MagicMock

import pytest

from src.mpt200 import MPT200PressureSensor


def make_sensor(address=1):
    """Create an MPT200PressureSensor with a mocked serial port."""
    sensor = MPT200PressureSensor(log=False, address=address)
    sensor.serial = MagicMock()
    sensor.serial.write.return_value = 1
    return sensor


def build_frame(address, param_num, data):
    """Build a valid gauge reply frame (with checksum) for the given data."""
    body = f"{address:03d}" + "00" + f"{param_num:03d}" + "00" + data
    checksum = sum(ord(c) for c in body) % 256
    return body + f"{checksum:03d}\r"


def test_send_command_formats_status_request():
    """_send_command should write a correctly formatted status request with checksum."""
    sensor = make_sensor()

    assert sensor._send_command("pressure_value") is True  # pylint: disable=protected-access

    expected_cmd = "{:03d}00{:03d}02=?".format(1, 740)
    expected_cmd += "{:03d}\r".format(sum(ord(x) for x in expected_cmd) % 256)

    sensor.serial.write.assert_called_once_with(expected_cmd.encode())
    assert sensor.last_command_num == 740


def test_read_pressure_parses_valid_reply():
    """read_pressure should convert a valid gauge reply into the correct pressure value."""
    sensor = make_sensor()
    frame = build_frame(address=1, param_num=740, data="123423")
    sensor.serial.read.side_effect = [c.encode() for c in frame]

    value = sensor.read_pressure()

    assert value == pytest.approx(1234.0)


def test_read_pressure_returns_none_on_short_reply():
    """read_pressure should return None when the gauge reply is too short to be valid."""
    sensor = make_sensor()
    # too short to be a valid reply
    sensor.serial.read.side_effect = [b"1", b"2", b"3", b""]

    assert sensor.read_pressure() is None
