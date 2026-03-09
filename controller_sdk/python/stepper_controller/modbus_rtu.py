from __future__ import annotations

import time
from typing import List, Optional

try:
    import serial  # type: ignore
except Exception as exc:  # pragma: no cover - handled at runtime
    serial = None
    _serial_import_error = exc
else:
    _serial_import_error = None


class ModbusError(Exception):
    pass


class ModbusTimeout(ModbusError):
    pass


class ModbusCRCError(ModbusError):
    pass


class ModbusProtocolError(ModbusError):
    pass


class ModbusDeviceError(ModbusError):
    def __init__(self, function: int, code: int):
        super().__init__(f"Device error: function=0x{function:02X} code=0x{code:02X}")
        self.function = function
        self.code = code


class ModbusRTUClient:
    def __init__(
        self,
        port: str,
        baudrate: int = 115200,
        slave_id: int = 1,
        timeout: float = 0.2,
        parity: str = "N",
        stopbits: int = 1,
        bytesize: int = 8,
    ) -> None:
        if serial is None:
            raise ImportError("pyserial is required to use ModbusRTUClient") from _serial_import_error

        if not (1 <= slave_id <= 247):
            raise ValueError("slave_id must be 1..247")

        self.port = port
        self.baudrate = baudrate
        self.slave_id = slave_id
        self.timeout = timeout
        self.parity = parity
        self.stopbits = stopbits
        self.bytesize = bytesize
        self._ser: Optional[serial.Serial] = None

    def open(self) -> None:
        if self._ser and self._ser.is_open:
            return
        self._ser = serial.Serial(
            port=self.port,
            baudrate=self.baudrate,
            timeout=self.timeout,
            parity=self.parity,
            stopbits=self.stopbits,
            bytesize=self.bytesize,
        )

    def close(self) -> None:
        if self._ser:
            try:
                self._ser.close()
            finally:
                self._ser = None

    def _ensure_open(self) -> serial.Serial:
        if not self._ser or not self._ser.is_open:
            raise ModbusError("serial port is not open")
        return self._ser

    def read_holding_registers(self, address: int, count: int) -> List[int]:
        if count <= 0 or count > 125:
            raise ValueError("count must be 1..125")
        address = _normalize_address(address)

        payload = _u16be(address) + _u16be(count)
        frame = _build_frame(self.slave_id, 0x03, payload)

        ser = self._ensure_open()
        _flush_serial(ser)
        ser.write(frame)
        ser.flush()

        # Read header (slave, function, byte_count)
        header = _read_exact(ser, 3, self.timeout)
        _check_slave(header, self.slave_id)
        function = header[1]
        if function & 0x80:
            # Exception response: [slave][func|0x80][code][crc]
            rest = _read_exact(ser, 1, self.timeout)
            frame_rx = header + rest
            _verify_crc(frame_rx)
            raise ModbusDeviceError(function & 0x7F, header[2])

        if function != 0x03:
            raise ModbusProtocolError(f"unexpected function 0x{function:02X}")

        byte_count = header[2]
        if byte_count != count * 2:
            raise ModbusProtocolError("byte count mismatch")

        data_crc = _read_exact(ser, byte_count + 2, self.timeout)
        frame_rx = header + data_crc
        _verify_crc(frame_rx)

        data = data_crc[:-2]
        return [_u16_from_be(data[i : i + 2]) for i in range(0, len(data), 2)]

    def write_single_register(self, address: int, value: int) -> None:
        if not (0 <= value <= 0xFFFF):
            raise ValueError("value must be 0..65535")
        address = _normalize_address(address)

        payload = _u16be(address) + _u16be(value)
        frame = _build_frame(self.slave_id, 0x06, payload)

        ser = self._ensure_open()
        _flush_serial(ser)
        ser.write(frame)
        ser.flush()

        # Response is 8 bytes or exception 5 bytes
        header = _read_exact(ser, 2, self.timeout)
        _check_slave(header, self.slave_id)
        function = header[1]
        if function & 0x80:
            rest = _read_exact(ser, 3, self.timeout)
            frame_rx = header + rest
            _verify_crc(frame_rx)
            raise ModbusDeviceError(function & 0x7F, rest[0])
        if function != 0x06:
            raise ModbusProtocolError(f"unexpected function 0x{function:02X}")

        rest = _read_exact(ser, 6, self.timeout)
        frame_rx = header + rest
        _verify_crc(frame_rx)

        # Optional echo validation
        addr_rx = _u16_from_be(rest[0:2])
        val_rx = _u16_from_be(rest[2:4])
        if addr_rx != address or val_rx != value:
            raise ModbusProtocolError("echo mismatch")

    def write_multiple_registers(self, address: int, values: List[int]) -> None:
        if not values:
            raise ValueError("values must not be empty")
        if len(values) > 123:
            raise ValueError("count must be 1..123")
        for v in values:
            if not (0 <= v <= 0xFFFF):
                raise ValueError("each value must be 0..65535")

        address = _normalize_address(address)
        count = len(values)
        byte_count = count * 2

        payload = _u16be(address) + _u16be(count) + bytes([byte_count])
        for v in values:
            payload += _u16be(v)

        frame = _build_frame(self.slave_id, 0x10, payload)

        ser = self._ensure_open()
        _flush_serial(ser)
        ser.write(frame)
        ser.flush()

        header = _read_exact(ser, 2, self.timeout)
        _check_slave(header, self.slave_id)
        function = header[1]
        if function & 0x80:
            rest = _read_exact(ser, 3, self.timeout)
            frame_rx = header + rest
            _verify_crc(frame_rx)
            raise ModbusDeviceError(function & 0x7F, rest[0])
        if function != 0x10:
            raise ModbusProtocolError(f"unexpected function 0x{function:02X}")

        rest = _read_exact(ser, 6, self.timeout)
        frame_rx = header + rest
        _verify_crc(frame_rx)

        addr_rx = _u16_from_be(rest[0:2])
        count_rx = _u16_from_be(rest[2:4])
        if addr_rx != address or count_rx != count:
            raise ModbusProtocolError("echo mismatch")


def _normalize_address(address: int) -> int:
    if address < 0:
        raise ValueError("address must be >= 0")
    if address >= 40000:
        if address > 0xFFFF:
            raise ValueError("address overflow")
        return address
    abs_addr = 40000 + address
    if abs_addr > 0xFFFF:
        raise ValueError("address overflow")
    return abs_addr


def _build_frame(slave_id: int, function: int, payload: bytes) -> bytes:
    frame = bytes([slave_id, function]) + payload
    crc = _crc16(frame)
    return frame + crc.to_bytes(2, "little")


def _crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


def _verify_crc(frame: bytes) -> None:
    if len(frame) < 3:
        raise ModbusCRCError("frame too short")
    data = frame[:-2]
    crc_expected = _crc16(data)
    crc_actual = int.from_bytes(frame[-2:], "little")
    if crc_expected != crc_actual:
        raise ModbusCRCError("CRC mismatch")


def _read_exact(ser: serial.Serial, size: int, timeout: float) -> bytes:
    deadline = time.monotonic() + timeout
    buf = bytearray()
    while len(buf) < size:
        chunk = ser.read(size - len(buf))
        if chunk:
            buf.extend(chunk)
            continue
        if time.monotonic() > deadline:
            raise ModbusTimeout("timeout waiting for response")
    return bytes(buf)


def _flush_serial(ser: serial.Serial) -> None:
    try:
        ser.reset_input_buffer()
        ser.reset_output_buffer()
    except Exception:
        pass


def _check_slave(header: bytes, slave_id: int) -> None:
    if not header or header[0] != slave_id:
        raise ModbusProtocolError("slave id mismatch")


def _u16be(value: int) -> bytes:
    return bytes([(value >> 8) & 0xFF, value & 0xFF])


def _u16_from_be(b: bytes) -> int:
    return (b[0] << 8) | b[1]
