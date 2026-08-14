"""
Modbus-RTU codec and slave dispatch. Transport-agnostic: hand it bytes, it
hands back bytes. The serial link, and any future TCP or honeypot front end,
sit on top of this.
"""

from __future__ import annotations

from .model import (AuroraModel, EXC_ILLEGAL_ADDRESS, EXC_ILLEGAL_FUNCTION,
                    EXC_ILLEGAL_VALUE)

FC_READ_HOLDING = 0x03
FC_WRITE_SINGLE = 0x06
FC_WRITE_MULTIPLE = 0x10


def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc


def with_crc(payload: bytes) -> bytes:
    crc = crc16(payload)
    return payload + bytes([crc & 0xFF, (crc >> 8) & 0xFF])


def check_crc(frame: bytes) -> bool:
    if len(frame) < 4:
        return False
    return crc16(frame[:-2]) == (frame[-2] | (frame[-1] << 8))


def expected_request_len(buf: bytes):
    """
    Length of the request in `buf` once complete; None if undecidable yet,
    -1 if it cannot start a request we understand.

    Length-based framing rather than the RTU 3.5-character idle gap, because a
    USB-serial adapter batches bytes on its latency timer (16 ms by default on
    FTDI parts) - roughly 8x t3.5 at 19200. That shreds gap-based framing: one
    request arrives as several reads with long gaps inside it.
    """
    if len(buf) < 2:
        return None
    fc = buf[1]
    if fc in (FC_READ_HOLDING, FC_WRITE_SINGLE):
        return 8
    if fc == FC_WRITE_MULTIPLE:
        if len(buf) < 7:
            return None
        return 9 + buf[6]
    return -1


class RtuSlave:
    """Dispatches validated RTU frames against an AuroraModel."""

    def __init__(self, model: AuroraModel, addr: int = 1, strict: bool = False):
        self.model = model
        self.addr = addr
        self.strict = strict
        self.last_request = None

    def _exception(self, fc: int, code: int) -> bytes:
        return with_crc(bytes([self.addr, fc | 0x80, code]))

    def handle(self, frame: bytes, now: float):
        """Return response bytes, or None to stay silent (as a real slave does
        for a bad CRC or another slave's address)."""
        if not check_crc(frame):
            return None
        if frame[0] != self.addr:
            return None

        fc = frame[1]
        if fc == FC_READ_HOLDING:
            return self._read(frame)
        if fc == FC_WRITE_SINGLE:
            return self._write_single(frame, now)
        if fc == FC_WRITE_MULTIPLE:
            return self._write_multiple(frame, now)
        return self._exception(fc, EXC_ILLEGAL_FUNCTION)

    def _read(self, frame: bytes):
        start = (frame[2] << 8) | frame[3]
        count = (frame[4] << 8) | frame[5]
        self.last_request = ("read", start, count)
        if count < 1 or count > 125:
            return self._exception(FC_READ_HOLDING, EXC_ILLEGAL_VALUE)

        values, exc = self.model.read(start, count, strict=self.strict)
        if exc is not None:
            return self._exception(FC_READ_HOLDING, exc)

        body = bytes([self.addr, FC_READ_HOLDING, count * 2])
        for v in values:
            body += bytes([(v >> 8) & 0xFF, v & 0xFF])
        return with_crc(body)

    def _write_single(self, frame: bytes, now: float):
        reg = (frame[2] << 8) | frame[3]
        value = (frame[4] << 8) | frame[5]
        self.last_request = ("write", reg, value)
        exc = self.model.write(reg, value, now)
        if exc is not None:
            return self._exception(FC_WRITE_SINGLE, exc)
        return with_crc(frame[:6])          # conforming slave echoes the request

    def _write_multiple(self, frame: bytes, now: float):
        start = (frame[2] << 8) | frame[3]
        count = (frame[4] << 8) | frame[5]
        nbytes = frame[6]
        self.last_request = ("write_multi", start, count)
        if nbytes != count * 2 or len(frame) != 9 + nbytes:
            return self._exception(FC_WRITE_MULTIPLE, EXC_ILLEGAL_VALUE)

        for i in range(count):
            value = (frame[7 + i * 2] << 8) | frame[8 + i * 2]
            exc = self.model.write(start + i, value, now)
            if exc is not None:
                return self._exception(FC_WRITE_MULTIPLE, exc)

        return with_crc(bytes([self.addr, FC_WRITE_MULTIPLE,
                               frame[2], frame[3], frame[4], frame[5]]))
