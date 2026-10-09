"""Reading an SO-ARM's Feetech STS servos over their serial bus.

Positions are converted to joint angles as `feetech_ros2_driver` converts them, ``(ticks - 2048) *
2 pi / 4096``, with each servo's homing offset (in its EEPROM, from LeRobot's calibration) applied
by the servo itself. The only write is `release`, which turns torque off.
"""

from __future__ import annotations

import math
import time

import serial

READ, WRITE = 0x02, 0x03
TORQUE_ENABLE = 40
HOMING_OFFSET = 31
PRESENT_POSITION = 56


def ticks_to_rad(ticks: int) -> float:
    return (ticks - 2048) * 2.0 * math.pi / 4096.0


def _sign_magnitude(value: int, sign_bit: int) -> int:
    return -(value & ((1 << sign_bit) - 1)) if value >> sign_bit & 1 else value


class Bus:
    def __init__(self, port: str, baud: int = 1_000_000, timeout_s: float = 0.02):
        self.serial = serial.Serial(port, baud, timeout=timeout_s)

    def close(self):
        self.serial.close()

    def _packet(self, servo: int, instruction: int, params: bytes) -> bytes:
        body = bytes([servo, len(params) + 2, instruction]) + params
        return b"\xff\xff" + body + bytes([(~sum(body)) & 0xFF])

    def _exchange(
        self, servo: int, instruction: int, params: bytes, reply_params: int
    ) -> bytes:
        self.serial.reset_input_buffer()
        self.serial.write(self._packet(servo, instruction, params))
        reply = self.serial.read(6 + reply_params)
        if (
            len(reply) < 6 + reply_params
            or reply[:2] != b"\xff\xff"
            or reply[2] != servo
        ):
            raise OSError(f"servo {servo} did not answer")
        if (~sum(reply[2:-1])) & 0xFF != reply[-1]:
            raise OSError(f"servo {servo} answered with a bad checksum")
        if reply[4]:
            raise OSError(f"servo {servo} reports error 0x{reply[4]:02x}")
        return reply[5 : 5 + reply_params]

    def read(self, servo: int, address: int, size: int) -> int:
        data = self._exchange(servo, READ, bytes([address, size]), size)
        return int.from_bytes(data, "little")

    def position_ticks(self, servo: int) -> int:
        return _sign_magnitude(self.read(servo, PRESENT_POSITION, 2), 15)

    def homing_offset(self, servo: int) -> int:
        return _sign_magnitude(self.read(servo, HOMING_OFFSET, 2), 11)

    def torque_on(self, servo: int) -> bool:
        return bool(self.read(servo, TORQUE_ENABLE, 1))

    def release(self, servo: int):
        """Turn the servo's torque off: the joint goes limp."""
        self._exchange(servo, WRITE, bytes([TORQUE_ENABLE, 0]), 0)

    def still_positions(
        self,
        servos: dict[str, int],
        samples: int = 15,
        period_s: float = 0.03,
        max_spread_ticks: int = 3,
    ) -> dict[str, float] | None:
        """Each joint's angle, radians, averaged over `samples` reads, or None if any joint moved
        more than `max_spread_ticks` meanwhile (the arm was not held still)."""
        reads = {name: [] for name in servos}
        for _ in range(samples):
            for name, servo in servos.items():
                reads[name].append(self.position_ticks(servo))
            time.sleep(period_s)
        if any(max(v) - min(v) > max_spread_ticks for v in reads.values()):
            return None
        return {name: ticks_to_rad(sum(v) / len(v)) for name, v in reads.items()}
