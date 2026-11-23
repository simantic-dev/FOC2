# AS5600 on a joint's motor shaft; the angle is the plant's rotor
# angle for this joint (arm.THETA) instead of the single motor's. Subclassed per joint (joint1.py, joint2.py), which set JOINT = 0 or 1.
import math

from arm import THETA

MAGNET_OFFSET = 1.0  # magnet glued on 1 rad off the rotor's d-axis, as in the single-motor fixture


class Slave:
    def __init__(self, ctx):
        self.j = self.JOINT
        self.regs = [0] * 0x100
        self.regs[0x0B] = 0x20  # MD
        self.regs[0x1A] = 0x80
        self.regs[0x1B], self.regs[0x1C] = 0x08, 0x00
        self.ptr = 0

    def on_address(self, addr, is_read):
        return True

    def on_write(self, data):
        if data:
            self.ptr = data[0]
            for i, b in enumerate(data[1:]):
                self.regs[(self.ptr + i) & 0xFF] = b
        return len(data)

    def on_read(self, count):
        raw = int((THETA[self.j] + MAGNET_OFFSET) / (2 * math.pi) * 4096) % 4096
        self.regs[0x0C], self.regs[0x0D] = raw >> 8, raw & 0xFF
        self.regs[0x0E], self.regs[0x0F] = raw >> 8, raw & 0xFF
        out = bytes(self.regs[(self.ptr + i) & 0xFF] for i in range(count))
        self.ptr = (self.ptr + count) & 0xFF
        return out
