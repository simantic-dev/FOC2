# The CAN host for both joints (FOC2 IDs 1 and 2, 1 Mbit/s; FOC2 CAN protocol).
# At ARM_AT (after both boards' startup calibration) it arms both servos, takes each one's first
# reported position as that joint's q=0 (arm hanging), then commands joint-angle targets: motor goal =
# home + N x target. Each pose is held HOLD s. At the end of a hold it checks, over the last WINDOW s,
# that FOC2's own CAN feedback (converted to joint degrees) and MuJoCo's joint angle were both within
# TOL_DEG of the target: "CAN: PASS" / "ARM: PASS" once every pose passed (else "... FAIL pose k ...").
import struct

from arm import N, WORLD

IDS = (1, 2)
ARM_AT, KP, KD = 5.0, 60, 150
POSES = [(40.0, 20.0), (70.0, -40.0), (20.0, 60.0)]  # joint degrees from hanging down, (j1, j2)
LEAD, HOLD, WINDOW, TOL_DEG = 0.3, 1.2, 0.1, 3.0  # hold at q=0 for LEAD s first


class Node:
    def __init__(self, ctx):
        self.ctx = ctx
        self.armed = False
        self.home = {}
        self.fb = {}
        self.pose = -1
        self.t0 = None  # when both servos had reported a home
        self.err_can = [0.0, 0.0]
        self.err_arm = [0.0, 0.0]
        self.failed = {"CAN": False, "ARM": False}
        self.end = False
        ctx.schedule_periodic(10000)

    def target(self, t):
        k = int((t - self.t0 - LEAD) // HOLD)
        return (-1, (0.0, 0.0)) if k < 0 else (min(k, len(POSES) - 1), POSES[min(k, len(POSES) - 1)])

    def on_timer(self):
        t = self.ctx.now_us * 1e-6
        if t < ARM_AT or self.end:
            return
        if not self.armed:
            self.armed = True
            self.ctx.info("CAN: arm")
            for i in IDS:
                self.ctx.send(i, [0xFF] * 8)
            return
        if len(self.home) < len(IDS):
            return
        self.t0 = self.t0 or t
        k, tgt = self.target(t)
        if k != self.pose:
            self.pose = k
            WORLD.log_target(t, *tgt)
            self.ctx.info(f"CAN: pose {k} target {tgt}")
        for j, i in enumerate(IDS):
            goal = self.home[i] + N * tgt[j]
            self.ctx.send(i, list(struct.pack("<hhBB", int(goal * 10), 0, KP, KD)))
        t_end = self.t0 + LEAD + HOLD * (k + 1)
        if k >= 0 and t_end - WINDOW <= t < t_end:
            arm = WORLD.joint_deg()
            for j, i in enumerate(IDS):
                self.err_can[j] = max(self.err_can[j], abs((self.fb[i] - self.home[i]) / N - tgt[j]))
                self.err_arm[j] = max(self.err_arm[j], abs(arm[j] - tgt[j]))
        if k >= 0 and t + 0.01 >= t_end:
            for name, err in (("CAN", self.err_can), ("ARM", self.err_arm)):
                bad = max(err) > TOL_DEG
                self.failed[name] |= bad
                self.ctx.info(f"{name}: pose {k} worst error in last {WINDOW}s j1 {err[0]:.2f} j2 {err[1]:.2f} deg {'FAIL' if bad else 'ok'}")
            self.err_can, self.err_arm = [0.0, 0.0], [0.0, 0.0]
            if k == len(POSES) - 1:
                self.end = True
                WORLD.flush()
                for name in ("CAN", "ARM"):
                    self.ctx.info(f"{name}: {'FAIL' if self.failed[name] else 'PASS'} all {len(POSES)} poses within {TOL_DEG} deg")

    def on_frame(self, frame):
        i = frame["id"] - 0x10
        if i not in IDS or len(frame["data"]) != 4:
            return
        pos = struct.unpack("<h", bytes(frame["data"][:2]))[0] / 10
        self.fb[i] = pos
        if self.armed and i not in self.home:
            self.home[i] = pos
            self.ctx.info(f"CAN: servo {i} home {pos:.1f} deg")
