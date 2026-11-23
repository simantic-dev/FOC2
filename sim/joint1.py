# Joint 1's scripts: the plant and the encoder with their joint index (a scenario part has no config key).
from encoder import Slave as _Slave
from plant import Peripheral as _Peripheral


class Peripheral(_Peripheral):
    JOINT = 0


class Slave(_Slave):
    JOINT = 0
