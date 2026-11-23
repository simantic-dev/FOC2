# The shared MuJoCo world of the 2-link arm. Both joints' plants (plant.py), both encoders (encoder.py)
# and the CAN host (can_host.py) import this module; sim runs every Python peer of a scenario in one
# interpreter, so the module-level WORLD is one object for both machines.
#
# Determinism: machines run in lockstep, one 10 us tick at a time, but the order in which j1's and j2's
# plants run inside a tick is not fixed (threads). So nobody reads "the latest" state. Each plant posts
# its joint torque for tick n; once BOTH joints have posted tick 5(k+1), whoever got there last advances
# MuJoCo one 50 us step with the mean of each joint's five posted torques (exact for the 10 us hold).
# A plant at tick n reads state k = n//5 - 1 (always complete, the joints are never more than one tick
# apart) and extrapolates it with the joint velocity. The result does not depend on thread order.
import math
import os
import threading

import mujoco

N = 5  # gear ratio, motor turns per joint turn (ideal gearbox: torque x N, speed / N, no loss)
TICK_US, STEP_TICKS = 10, 5
J_ROTOR, B_ROTOR = 5e-5, 2e-4  # the rotor's own inertia and damping (as the single-motor plant)
PHI0 = 4.2  # rotor mechanical angle at q=0, so FOC2's multi-turn reading starts at 298 deg (limit 0..1000)
LOG_EVERY = 20  # steps between qpos log rows (1 ms)

MJCF = f"""
<mujoco>
  <option timestep="{TICK_US * STEP_TICKS * 1e-6}" integrator="RK4" gravity="0 0 -9.81"/>
  <compiler inertiafromgeom="true"/>
  <asset><texture type="skybox" builtin="gradient" rgb1="0.85 0.9 1" rgb2="0.55 0.6 0.7" width="64" height="64"/></asset>
  <worldbody>
    <light pos="0 -1 1" dir="0 1 -1"/>
    <geom type="box" pos="0 0 0.02" size="0.06 0.04 0.02" rgba="0.3 0.3 0.35 1"/>
    <body name="link1">
      <joint name="j1" type="hinge" axis="0 -1 0" armature="{J_ROTOR * N * N}" damping="{B_ROTOR * N * N}"/>
      <geom type="capsule" fromto="0 0 0 0 0 -0.15" size="0.012" mass="0.05" rgba="0.9 0.5 0.1 1"/>
      <body name="link2" pos="0 0 -0.15">
        <joint name="j2" type="hinge" axis="0 -1 0" armature="{J_ROTOR * N * N}" damping="{B_ROTOR * N * N}"/>
        <geom type="capsule" fromto="0 0 0 0 0 -0.15" size="0.01" mass="0.05" rgba="0.2 0.6 0.9 1"/>
        <geom type="sphere" pos="0 0 -0.15" size="0.014" mass="0.01" rgba="0.9 0.2 0.2 1"/>
      </body>
    </body>
  </worldbody>
</mujoco>
"""


class World:
    def __init__(self):
        self.model = mujoco.MjModel.from_xml_string(MJCF)
        self.data = mujoco.MjData(self.model)
        self.lock = threading.Lock()
        self.posted = [0, 0]
        self.tau = [{}, {}]  # tick -> joint torque (Nm)
        self.state = {0: (0.0, 0.0, 0.0, 0.0)}  # step index -> q1, q2, qd1, qd2
        self.steps = 0
        self.log = open(os.environ.get("ARM_QPOS_LOG", "/tmp/foc2_arm_qpos.csv"), "w")
        self.log.write("# kind,t_s,a,b   Q = joint angles (rad); T = host targets (deg)\n")

    def post_and_read(self, j, n, tau):
        """Joint j's plant at tick n posts its joint torque; returns (q, qd) of joint j at tick n."""
        with self.lock:
            self.tau[j][n] = tau
            self.posted[j] = n
            while min(self.posted) >= STEP_TICKS * (self.steps + 1):
                self._step()
            s = max(n // STEP_TICKS - 1, 0)
            q1, q2, d1, d2 = self.state[s]
            q, qd = (q1, d1) if j == 0 else (q2, d2)
            return q + qd * (n * TICK_US - s * TICK_US * STEP_TICKS) * 1e-6, qd

    def _step(self):
        k = self.steps
        for j in (0, 1):
            ticks = range(STEP_TICKS * k + 1, STEP_TICKS * (k + 1) + 1)
            self.data.qfrc_applied[j] = sum(self.tau[j].pop(t) for t in ticks) / STEP_TICKS
        mujoco.mj_step(self.model, self.data)
        self.steps += 1
        self.state[self.steps] = (*self.data.qpos, *self.data.qvel)
        self.state.pop(self.steps - 4, None)
        if self.steps % LOG_EVERY == 0:
            self.log.write(f"Q,{self.steps * TICK_US * STEP_TICKS * 1e-6:.6f},{self.data.qpos[0]:.6f},{self.data.qpos[1]:.6f}\n")

    def joint_deg(self):
        """Latest completed step, for the host's checks (not used by anything that feeds the firmware)."""
        with self.lock:
            return [math.degrees(v) for v in self.state[self.steps][:2]]

    def log_target(self, t_s, deg1, deg2):
        with self.lock:
            self.log.write(f"T,{t_s:.6f},{deg1},{deg2}\n")

    def flush(self):
        with self.lock:
            self.log.flush()


THETA = [PHI0, PHI0]  # rotor angle of each joint as its plant last computed it; the encoders read it
WORLD = World()
