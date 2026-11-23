# One joint's drive: the B-G431B-ESC1 bridge + PMSM electrical model
# (dq currents, averaged bridge, shunt/VBUS/NTC pads: the same code), with the mechanical equation
# J dw/dt = T - B w - TL replaced by MuJoCo: this plant computes the electromagnetic torque
# 1.5 P PSI iq, applies N x that to its hinge in the shared world (arm.py), and reads the hinge angle and
# velocity back (rotor angle = N q + PHI0, speed = N qd). The rotor's inertia and damping are the hinge's
# `armature` and `damping` (J N^2, B N^2). Subclassed per joint (joint1.py, joint2.py), which set JOINT = 0 or 1.
#
# MODELED SIMPLIFICATION: averaged switching (no ripple, no dead-time loss); ideal sensors and an ideal
# gearbox (no backlash, friction or efficiency); the motor numbers are the example gimbal motor's.
import math

from arm import N, PHI0, THETA, TICK_US, WORLD

R, L, PSI, P = 5.0, 2e-3, 0.006, 14
VBUS = 12.0
RSHUNT, VINP_BIAS = 0.003, 0.103125
NET_GAIN = 4 / 7
TIM1 = 0x40012C00
CR1, ARR, CCR1, CCR2, CCR3, BDTR = 0x00, 0x2C, 0x34, 0x38, 0x3C, 0x44
CR1_DIR = 1 << 4
SUB = 2


class Peripheral:
    def __init__(self, ctx):
        self.ctx = ctx
        self.j = self.JOINT
        self.n = 0
        self.id = self.iq = 0.0
        self.tau = 0.0
        ctx.schedule_periodic(TICK_US)

    def on_access(self, req):
        if req.is_init:
            self.ctx.schedule_periodic(TICK_US)
        req.value = 0

    def on_timer(self):
        self.n += 1
        q, qd = WORLD.post_and_read(self.j, self.n, self.tau)
        theta_m, wm = N * q + PHI0, N * qd
        THETA[self.j] = theta_m  # the AS5600 of this joint reads it
        rd = lambda off: self.ctx.bus_read(TIM1 + off, 4)
        arr = rd(ARR) or 1
        duty = [rd(off) / arr for off in (CCR1, CCR2, CCR3)]
        on = rd(BDTR) & (1 << 15) != 0
        dt = TICK_US * 1e-6 / SUB
        mean = sum(duty) / 3
        vph = [VBUS * (d - mean) for d in duty]
        for _ in range(SUB):
            th = P * theta_m
            c = [math.cos(th - k * 2 * math.pi / 3) for k in range(3)]
            s = [math.sin(th - k * 2 * math.pi / 3) for k in range(3)]
            we = P * wm
            if on:
                vd = 2 / 3 * sum(v * ck for v, ck in zip(vph, c))
                vq = -2 / 3 * sum(v * sk for v, sk in zip(vph, s))
                self.id += (vd - R * self.id + we * L * self.iq) / L * dt
                self.iq += (vq - R * self.iq - we * L * self.id - we * PSI) / L * dt
            else:
                self.id = self.iq = 0.0
            theta_m += wm * dt
        self.tau = N * 1.5 * P * PSI * self.iq  # joint torque for the next tick
        th = P * theta_m
        ia = self.id * math.cos(th) - self.iq * math.sin(th)
        ib = self.id * math.cos(th - 2 * math.pi / 3) - self.iq * math.sin(th - 2 * math.pi / 3)
        sensing = rd(CR1) & CR1_DIR == 0  # counting up: the next update is the top
        for ch, i in ((0, ia), (8, ib), (16, -ia - ib)):  # OPAMP1-3 VINP0
            self.ctx.set_property("opamp", f"ch{ch}", VINP_BIAS - (NET_GAIN * RSHUNT * i if sensing else 0.0))
        self.ctx.set_property("adc1", "ch1", VBUS * 18 / 187)
        self.ctx.set_property("adc1", "ch5", 3.3 * 4.7 / 14.7)
