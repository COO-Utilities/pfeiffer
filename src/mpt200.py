""" Class for Pfeiffer MPT200 pressure sensor """
from typing import Union

import serial
import pfeiffer_vacuum_protocol as pvp

try:
    from hardware_device_base import HardwareSensorBase
except ModuleNotFoundError:
    from hardware_device_base.hardware_sensor_base import HardwareSensorBase  # type: ignore


class MPT200PressureSensor(HardwareSensorBase):  # pylint: disable=too-many-instance-attributes
    """ Class for Pfeiffer MPT200 pressure sensor

    This class provides methods to read the MPT200 pressure sensor

    """
    # commands that initial control functions
    control_commands = {
        'on_off': {'cmd': 41, 'size': 1},
        'switching_ranges': {'cmd': 49, 'size': 3},
        'set_pressure_switch_point1': {'cmd': 730, 'size': 6},
        'set_pressure_switch_point2': {'cmd': 732, 'size': 6},
        'set_pressure_value': {'cmd': 740, 'size': 6},  # only used for calibration
        'set_pressure_adjustment_point': {'cmd': 741, 'size': 3},
        'set_correction_factor_pirani': {'cmd': 742, 'size': 6},
        'set_correction_factor_cold_cathode': {'cmd': 743, 'size': 6}
    }
    # commands that request status or data
    status_requests = {
        'current_error': 303,
        'software_version': 312,
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

    def _send_command(self, command: str, data_str:str ="") -> int:  # pylint: disable=W0221
        """ Send a command to the Pfeiffer MPT200 pressure sensor """
        if command in self.status_requests:
            cmd = "{:03d}00{:03d}02=?".format(self.address, self.status_requests[command])
            cmd += "{:03d}\r".format(sum([ord(x) for x in cmd]) % 256)
            return self.serial.write(cmd.encode())
        if command in self.control_commands:
            if data_str is not None:
                cmd = "{:03d}10{:03d}{:02d}{:s}".format(self.address,
                                                        self.control_commands[command],
                                                        len(data_str), data_str)
                cmd += "{:03d}\r".format(sum([ord(x) for x in cmd]) % 256)
                return self.serial.write(cmd.encode())
            self.report_error("Control commands require data to be sent")
        return 0

    def _read_reply(self) -> Union[str, None]:
        """ read a reply from the device """
        self.report_warning("_read_reply not implemented")

    def get_atomic_value(self, item: str ="") -> Union[float, int, str, None]:
        """ get a value from the device """
        if self.is_connected():
            if item == "pressure":
                value = pvp.read_pressure(self.serial, self.address)
            elif item == "error":
                value = pvp.read_error_code(self.serial, self.address)
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

        self.software_version = pvp.read_software_version(self.serial, self.address)
        self.last_error_code = pvp.read_error_code(self.serial, self.address)
        if self.last_error_code != 0:
            self.report_error(f"Error code: {self.last_error_code}")
        # self.gauge_type = pvp.read_gauge_type(self.serial, self.address)

        self.initialized = True
        return True
