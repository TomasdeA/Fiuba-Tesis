#!/usr/bin/env python3
"""Independent short-duration rotation check; not ground truth or a VIO estimator.

Assumes the first four seconds are stationary. Initial acceleration defines up.
Reports both raw and bias-corrected gyro integration to expose sensitivity.
"""
import argparse
import json
import sqlite3
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu

p = argparse.ArgumentParser(description=__doc__)
p.add_argument('bag', type=Path)
p.add_argument('output', type=Path)
a = p.parse_args()
c = sqlite3.connect(f'file:{next(a.bag.glob("*.db3"))}?mode=ro', uri=True)
tid, = c.execute("select id from topics where name='/camera/camera/imu'").fetchone()
data = []
for raw, in c.execute('select data from messages where topic_id=? order by timestamp', (tid,)):
    m = deserialize_message(raw, Imu)
    w, v = m.angular_velocity, m.linear_acceleration
    data.append([m.header.stamp.sec+m.header.stamp.nanosec/1e9, w.x,w.y,w.z,v.x,v.y,v.z])
data = np.array(data)
t = data[:,0]-data[0,0]
initial = data[t<4]
bias = initial[:,1:4].mean(axis=0)
up = initial[:,4:7].mean(axis=0); up /= np.linalg.norm(up)
f0 = np.array([0.,0.,1.]); f0 -= up*np.dot(up,f0); f0 /= np.linalg.norm(f0)
result = {'gyro_bias_rad_s':bias.tolist(), 'initial_up':up.tolist(), 'time_s':t.tolist()}
for label, offset in [('raw',np.zeros(3)), ('bias_corrected',bias)]:
    r = Rotation.identity()
    yaw = [0.]
    for i in range(1,len(t)):
        dt = t[i]-t[i-1]
        if dt <= 0 or dt > .1:
            raise ValueError(f'Unexpected IMU gap: {dt}')
        w = (data[i,1:4]+data[i-1,1:4])/2-offset
        r = r*Rotation.from_rotvec(w*dt)
        f = r.apply([0,0,1]); f -= up*np.dot(up,f); f /= np.linalg.norm(f)
        yaw.append(np.arctan2(np.dot(up,np.cross(f0,f)),np.dot(f0,f)))
    result[label+'_yaw_deg'] = np.degrees(np.unwrap(yaw)).tolist()
    print(label, 'final yaw:', result[label+'_yaw_deg'][-1])
a.output.write_text(json.dumps(result))
