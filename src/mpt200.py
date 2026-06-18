""" Class for Pfeiffer MPT200 pressure sensor """
from typing import Union
from enum import Enum

import serial

try:
    from hardware_device_base import HardwareSensorBase
except ModuleNotFoundError:
    from hardware_device_base.hardware_sensor_base import HardwareSensorBase  # type: ignore

# Error states for vacuum gauges
class ErrorCode(Enum):
    """Class for Pfeiffer vacuum protocol error codes"""
    NO_ERROR = 0
    DEFECTIVE_SENSOR = 1
    DEFECTIVE_MEMORY = 2


class MPT200PressureSensor(HardwareSensorBase):  # pylint: disable=too-many-instance-attributes
    """ Class for Pfeiffer MPT200 pressure sensor

    This class provides methods to read the MPT200 pressure sensor

    """
    # commands that initial control functions
    control_commands = {
        'on_off': 41,
        'switching_ranges': 49,
        'set_pressure_switch_point1': 730,
        'set_pressure_switch_point2': 732,
        'set_pressure_value': 740,  # only used for calibration
        'set_pressure_adjustment_point': 741,
        'set_correction_factor_pirani': 742,
        'set_correction_factor_cold_cathode': 743
    }
    # commands that request status or data
    status_requests = {
        'current_error': 303,
        'firmware_version': 312,
        'device_name': 349,
        'hardware_version': 354,
        'serial_number': 355,
        'order_number': 388,
        'pressure_switch_point1': 730,
        'pressure_switch_point2': 732,
        'pressure_value': 740,
        'pressure_adjustment_point': 741,
        'correction_factor_pirani': 742,
        'correction_factor_cold_cathode': 743
    }

    error_codes = {
        "000000": ErrorCode.NO_ERROR,
        "Err001": ErrorCode.DEFECTIVE_SENSOR,
        "Err002": ErrorCode.DEFECTIVE_MEMORY
    }

    def __init__(self, log=True, logfile: str = __name__.rsplit(",", 1)[-1],
                 read_timeout: float = 1.0, address: int = 1) -> None:
        """ Constructor """
        super().__init__(log, logfile)
        self.read_timeout = read_timeout
        self.device_name = ""
        self.software_version = ""
        self.hardware_version = ""
        self.serial_number = ""
        self.pressure_switch_point1 = 0.0
        self.pressure_switch_point2 = 0.0
        self.pressure_adjustment_point = 0.0
        self.correction_factor_priani = 0.0
        self.correction_factor_cc = 0.0
        self.last_error_code = 0
        self.gauge_type = ""
        self.port = "/dev/ttyS0"
        self.baud = 9600
        self.address = address
        self.serial = None
        self.last_command_num = 0

    def connect(self, port: str ="/dev/ttyS0", baud: int =9600, con_type: str ="serial") -> None:  # pylint: disable=W0221
        """ Connect to Pfeiffer MPT200 pressure sensor """
        if self.validate_connection_params((port, baud)):
            try:
                if con_type == "serial":
                    self.port = port
                    self.baud = baud
                    self.serial = serial.Serial(port=port, baudrate=baud,
                                                bytesize=serial.EIGHTBITS,
                                                timeout=self.read_timeout,
                                                parity=serial.PARITY_NONE,
                                                stopbits=serial.STOPBITS_ONE)
                    self._set_connected(True)
                    self.report_info(f"Serial connection opened: {self.serial.is_open}")
                else:
                    self._set_connected(False)
                    self.report_error("Only serial connection is supported")
            except serial.SerialException as ex:
                self._set_connected(False)
                self.report_error(f"Could not connect to Pfeiffer MPT200 sensor: {ex}")
        else:
            self._set_connected(False)
            self.report_error(f"Invalid connection parameters {port} and {baud}")

    def disconnect(self) -> None:
        """ Disconnect from Pfeiffer MPT200 pressure sensor """
        if not self.is_connected():
            self.report_warning("Already disconnected from device")
            return
        try:
            self.serial.close()
            self._set_connected(False)
            self.report_info("Serial connection closed")
        except serial.SerialException as ex:
            self.report_error(f"Could not disconnect from Pfeiffer MPT200 sensor: {ex}")

    def _send_command(self, command: str, data_str:str ="") -> bool:  # pylint: disable=W0221
        """ Send a command to the Pfeiffer MPT200 pressure sensor """
        if command in self.status_requests:
            self.last_command_num = self.status_requests[command]
            cmd = "{:03d}00{:03d}02=?".format(self.address, self.status_requests[command])
            cmd += "{:03d}\r".format(sum([ord(x) for x in cmd]) % 256)
            self.report_debug(f"Sending status request command: {cmd}")
            n_chars = self.serial.write(cmd.encode())
        elif command in self.control_commands:
            if data_str is not None:
                self.last_command_num = self.control_commands[command]
                cmd = "{:03d}10{:03d}{:02d}{:s}".format(self.address,
                                                        self.control_commands[command],
                                                        len(data_str), data_str)
                cmd += "{:03d}\r".format(sum([ord(x) for x in cmd]) % 256)
                self.report_debug(f"Sending control command: {cmd}")
                n_chars = self.serial.write(cmd.encode())
            else:
                self.report_error("Control commands require data to be sent")
                n_chars = 0
        else:
            self.report_error(f"Invalid command {command}")
            n_chars = 0
        self.report_debug(f"Chars sent: {n_chars}")
        return n_chars > 0

    def _read_reply(self) -> Union[str, None]: # pylint: disable=too-many-branches
        """ read the gauge response """

        # Read until newline or we stop getting a response
        reply = ""
        for _ in range(64):
            char = self.serial.read(1)

            if char == b"":
                break

            try:
                reply += char.decode("ascii")
            except UnicodeDecodeError:
                self.report_warning(f"Invalid character {char}")
                continue

            if char == b"\r":
                break

        self.report_debug(f"Reply: {reply}")

        # Check the length
        if len(reply) < 14:
            self.report_error(f"gauge response too short to be valid: {reply}")
            data = None

        # Check it is terminated correctly
        elif reply[-1] != "\r":
            self.report_error("gauge response incorrectly terminated")
            data = None

        # Evaluate the checksum
        elif int(reply[-4:-1]) != (sum([ord(x) for x in reply[:-4]]) % 256):
            self.report_error("invalid checksum in gauge response")
            data = None

        else:
            # Pull out the response parts
            addr = int(reply[:3])
            # readwrite = int(reply[3:4])
            param_num = int(reply[5:8])
            data = reply[10:-4]

            # Check for errors
            if data == "NO_DEF":
                self.report_error("undefined parameter number")
            if data == "_RANGE":
                self.report_error("data is out of range")
            if data == "_LOGIC":
                self.report_error("logic access violation")

            # Confirm reply
            if int(addr) != self.address:
                self.report_error(f"invalid address {addr}")
                data = None
            if int(param_num) != self.last_command_num:
                self.report_error(f"Reply command {param_num} does not match "
                                  f"command sent: {self.last_command_num}")
                data = None

        # Return it
        return data

    def read_current_error(self) -> Union[ErrorCode, None]:
        """ Read the current error code """
        if self._send_command("current_error"):
            rdata = self._read_reply()
            if rdata is not None:
                if rdata in self.error_codes:
                    self.last_error_code = self.error_codes[rdata]
                    return self.error_codes[rdata]

                self.report_error(f"Unknown error code {rdata}")
                return None

            self.report_error("No response from MPT200 sensor")
        else:
            self.report_error("Unable to send error command")
        return None

    def read_firmware_version(self) -> tuple[int, int, int] | None:
        """ Read the gauge firmware version """
        if self._send_command("firmware_version"):
            rdata = self._read_reply()
            if rdata is not None:
                return int(rdata[0:2]), int(rdata[2:4]), int(rdata[4:])
        else:
            self.report_error("Unable to send firmware version command")
        return None

    def read_device_name(self) -> Union[str, None]:
        """ Read the gauge device name """
        if self._send_command("device_name"):
            rdata = self._read_reply()
            if rdata is not None:
                return rdata
            self.report_error("No response from MPT200 sensor")
        else:
            self.report_error("Unable to send device name command")
        return None

    def read_hardware_version(self) -> tuple[int, int, int] | None:
        """ Read the gauge hardware version """
        if self._send_command("hardware_version"):
            rdata = self._read_reply()
            if rdata is not None:
                return int(rdata[0:2]), int(rdata[2:4]), int(rdata[4:])
        else:
            self.report_error("Unable to send hardware version command")
        return None

    def read_serial_number(self) -> Union[str, None]:
        """ Read the gauge serial number """
        if self._send_command("serial_number"):
            rdata = self._read_reply()
            if rdata is not None:
                return rdata
            self.report_error("No response from MPT200 sensor")
        else:
            self.report_error("Unable to send serial number command")
        return None

    def read_order_number(self) -> Union[str, None]:
        """ Read the gauge order number """
        if self._send_command("order_number"):
            rdata = self._read_reply()
            if rdata is not None:
                return rdata
            self.report_error("No response from MPT200 sensor")
        else:
            self.report_error("Unable to send order number command")
        return None

    def read_pressure(self) -> Union[float, None]:
        """ Read the gauge pressure """

        if self._send_command("pressure_value"):

            rdata = self._read_reply()

            if rdata is not None:
                # Convert to a float
                mantissa = float(rdata[:4]) * 0.001
                exponent = int(rdata[4:])
                return float(mantissa * 10 ** (exponent - 20))
        else:
            self.report_error("Unable to send pressure command")
        return None

    def get_atomic_value(self, item: str ="") -> Union[float, int, str, None]:
        """ get a value from the device """
        if self.is_connected():
            if item == "pressure":
                value = self.read_pressure()
            # elif item == "error":
            #    value = self.read_error_code(self.serial, self.address)
            else:
                value = None
            return value
        self.report_error("Pfeiffer MPT200 pressure sensor not connected")
        return None

    def initialize(self) -> bool:
        """ initialize the Pfeiffer MPT200 pressure sensor """
        if not self.is_connected():
            self.report_error("Not connected to Pfeiffer MPT200 pressure sensor")
            return False

        self.software_version = self.read_firmware_version()
        # self.last_error_code = pvp.read_error_code(self.serial, self.address)
        if self.last_error_code != 0:
            self.report_error(f"Error code: {self.last_error_code}")
        # self.gauge_type = pvp.read_gauge_type(self.serial, self.address)

        self.initialized = True
        return True
