# Replays the plant's qpos log (ARM_QPOS_LOG, default /tmp/foc2_arm_qpos.csv) in MuJoCo offscreen into an
# MP4, with the host's target and the actual joint angles overlaid.
# Usage: python render.py [qpos.csv [out.mp4]]   (needs mujoco, imageio, imageio-ffmpeg)
import os
import sys

os.environ["ARM_QPOS_LOG"] = os.devnull  # importing arm opens the log for writing; do not truncate ours
import imageio.v2 as imageio
import mujoco
import numpy as np
from PIL import Image, ImageDraw, ImageFont

from arm import MJCF

FPS, W, H = 30, 640, 480
src = sys.argv[1] if len(sys.argv) > 1 else "/tmp/foc2_arm_qpos.csv"
out = sys.argv[2] if len(sys.argv) > 2 else "arm.mp4"
rows = [l.strip().split(",") for l in open(src) if l[0] in "QT"]
q = np.array([[float(r[1]), float(r[2]), float(r[3])] for r in rows if r[0] == "Q"])
tg = np.array([[float(r[1]), float(r[2]), float(r[3])] for r in rows if r[0] == "T"])
t0 = tg[0, 0] - 0.2  # start just before the first target
model = mujoco.MjModel.from_xml_string(MJCF)
data = mujoco.MjData(model)
cam = mujoco.MjvCamera()
cam.azimuth, cam.elevation, cam.distance, cam.lookat[:] = 90, 0, 0.55, (0.09, 0, -0.05)
font = ImageFont.load_default(size=20)
with mujoco.Renderer(model, H, W) as r, imageio.get_writer(out, fps=FPS, codec="libx264", macro_block_size=None) as w:
    for t in np.arange(t0, q[-1, 0], 1 / FPS):
        data.qpos[:] = [np.interp(t, q[:, 0], q[:, 1]), np.interp(t, q[:, 0], q[:, 2])]
        mujoco.mj_forward(model, data)
        r.update_scene(data, cam)
        img = Image.fromarray(r.render())
        k = max(np.searchsorted(tg[:, 0], t, side="right") - 1, 0)
        d = ImageDraw.Draw(img)
        d.text((10, 8), f"FOC2 x2 on pyrite + MuJoCo   t = {t:.2f} s", fill=(0, 0, 0), font=font)
        for j in (0, 1):
            d.text((10, 36 + 26 * j), f"joint {j + 1}: target {tg[k, 1 + j]:6.1f} deg   actual {np.degrees(data.qpos[j]):6.1f} deg",
                   fill=(0, 0, 0), font=font)
        w.append_data(np.asarray(img))
print("wrote", out)
