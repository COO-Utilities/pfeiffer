# pfeiffer
Pfeiffer pressure gauge modules

This repository provides two modules for communicating with Pfeiffer vacuum
gauges over their serial protocol (RS-232/RS-485, ASCII framed with a
checksum):

- **`src/mpt200.py`** — `MPT200PressureSensor`, a class-based driver for the
  Pfeiffer MPT200 pressure sensor. It subclasses `HardwareSensorBase` from the
  [`hardware_device_base`](https://github.com/COO-Utilities/hardware_device_base)
  package and manages its own `pyserial` connection. It supports:
  - Connecting/disconnecting over serial (`connect()` / `disconnect()`)
  - Reading device info: firmware/hardware version, device name, serial and
    order numbers
  - Reading/setting on-off state, switching ranges, pressure switch points,
    pressure adjustment point, and pressure correction factors
  - Reading the current pressure value and error code
- **`src/pfeiffer_vacuum_protocol.py`** — a lower-level, function-based
  implementation of the same protocol. Each function (e.g.
  `read_pressure()`, `write_pressure_setpoint()`, `read_error_code()`,
  `read_software_version()`, `read_gauge_type()`, `read_correction_value()` /
  `write_correction_value()`) takes an already-open serial connection object
  and gauge address, rather than owning the connection itself.

## Installation

Install the package along with its `dev` extras (needed for running tests):

```bash
pip install -e ".[dev]"
```

## Testing

Run the test suite with `pytest`:

```bash
pytest
```
