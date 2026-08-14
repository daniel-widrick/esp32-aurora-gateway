"""
Aurora ABC / IntelliZone-2 simulator.

Layered so the behavioural model is reusable beyond the bench:

    model.py  - registers and furnace behaviour, no transport
    rtu.py    - Modbus-RTU codec and slave dispatch, no I/O
    link.py   - RS-485 serial transport, impairments, statistics

Run it with:  py -m aurora_sim --port COM4
"""

from .dynamics import Dynamics
from .model import AuroraModel, decode_zone_config, encode_zone_config
from .rtu import RtuSlave, check_crc, crc16, expected_request_len, with_crc

__all__ = [
    "AuroraModel", "RtuSlave", "Dynamics",
    "crc16", "with_crc", "check_crc", "expected_request_len",
    "encode_zone_config", "decode_zone_config",
]
