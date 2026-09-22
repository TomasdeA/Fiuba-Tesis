#!/usr/bin/env python3
"""Audit source RGB-D stamps/calibration and render representative RGB frames."""
import argparse
import json
import sqlite3
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image, CameraInfo

p = argparse.ArgumentParser()
p.add_argument('bag', type=Path)
p.add_argument('output', type=Path)
a = p.parse_args()
a.output.mkdir(parents=True, exist_ok=True)
c = sqlite3.connect(f'file:{next(a.bag.glob("*.db3"))}?mode=ro', uri=True)
topics = {n: i for i, n in c.execute('select id,name from topics')}
stamps = {}
frames = []
report = {}
for name in ['/camera/camera/color/image_raw', '/camera/camera/aligned_depth_to_color/image_raw']:
    ts = []
    wanted = iter([0, 5, 6, 7, 8, 10, 14, 18, 22, 28, 36, 46])
    target = next(wanted)
    for raw, in c.execute('select data from messages where topic_id=? order by timestamp', (topics[name],)):
        m = deserialize_message(raw, Image)
        stamp = m.header.stamp.sec + m.header.stamp.nanosec/1e9
        ts.append(stamp)
        if len(ts) == 1:
            report[name] = dict(width=m.width, height=m.height, encoding=m.encoding, frame=m.header.frame_id)
        if name == '/camera/camera/color/image_raw' and stamp-ts[0] >= target:
            im = np.frombuffer(m.data, dtype=np.uint8).reshape(m.height, m.step)[:, :m.width*3].reshape(m.height, m.width, 3)
            if m.encoding == 'bgr8':
                im = im[..., ::-1]
            frames.append((stamp-ts[0], im.copy()))
            target = next(wanted, float('inf'))
    stamps[name] = np.array(ts)
    report[name]['frames'] = len(ts)
    report[name]['interval_ms_quantiles'] = np.quantile(np.diff(ts)*1000, [0,.5,.95,1]).tolist()
for name, tid in topics.items():
    if 'camera_info' in name:
        raw, = c.execute('select data from messages where topic_id=? limit 1', (tid,)).fetchone()
        m = deserialize_message(raw, CameraInfo)
        report[name] = dict(width=m.width, height=m.height, k=list(m.k), p=list(m.p), d=list(m.d), model=m.distortion_model)
rgb, depth = stamps.values()
diff = np.min(np.abs(rgb[:,None]-depth[None,:]), axis=1)
report['nearest_rgb_depth_ms_quantiles'] = np.quantile(diff*1000, [0,.5,.95,1]).tolist()
(a.output/'input_audit.json').write_text(json.dumps(report, indent=2))
fig, axs = plt.subplots(3,4,figsize=(16,8), constrained_layout=True)
for ax, (t, im) in zip(axs.flat, frames):
    ax.imshow(im); ax.set_title(f'{t:.2f} s'); ax.axis('off')
fig.savefig(a.output/'scene.png', dpi=130)
print(json.dumps(report, indent=2))
