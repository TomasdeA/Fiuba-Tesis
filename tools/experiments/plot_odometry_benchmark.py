#!/usr/bin/env python3
"""Compare tracking and poses. Invalid zero poses are excluded, never interpolated."""
import argparse
import json
from pathlib import Path
import numpy as np
from scipy.spatial.transform import Rotation
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('root', type=Path)
parser.add_argument('--pattern', default='movement_*_v2')
parser.add_argument('--runs', nargs='+', help='Explicit run directory names')
parser.add_argument('--output-name', default='comparison')
parser.add_argument('--title', default='Comparación de odometría RGB-D')
a = parser.parse_args()
fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True,
                         gridspec_kw={'height_ratios': [2, 2, .7]}, constrained_layout=True)
summary = []
directories = [a.root/name for name in a.runs] if a.runs else sorted(a.root.glob(a.pattern))
for directory in directories:
    if not (directory/'data.json').exists():
        continue
    data = json.loads((directory/'data.json').read_text())
    if data['variant'] == 'optical_flow':
        continue  # F2M rejects this option in RTAB-Map 0.22.1.
    poses = data['poses']
    if not poses:
        continue
    t = np.array([p['stamp'] for p in poses]); t -= t[0]
    valid = np.array([p['valid'] for p in poses])
    indices = np.flatnonzero(valid)
    if not len(indices):
        continue
    p = np.array([p['p'] for p in poses]); q = np.array([p['q'] for p in poses])
    r0 = Rotation.from_quat(q[indices[0]])
    local = r0.inv().apply(p-p[indices[0]])
    if data['variant'] != 'no_imu':
        initial_forward = r0.apply([0,0,1])
        initial_forward[2] = 0
        initial_forward /= np.linalg.norm(initial_forward)
        local[:,2] = (p-p[indices[0]]) @ initial_forward
    forward = Rotation.from_quat(q[indices]).apply([0,0,1])
    if data['variant'] == 'no_imu':
        forward = r0.inv().apply(forward)
        heading = np.unwrap(np.arctan2(-forward[:,0], forward[:,2]))
    else:
        heading = np.unwrap(np.arctan2(forward[:,1], forward[:,0]))
    yaw = np.full(len(t), np.nan)
    yaw[indices] = np.degrees(heading-heading[0])
    local[~valid] = np.nan
    label = directory.name
    display_label = {
        'movement_full_baseline': 'Anterior · v2 completo',
        'movement_full_final': 'Nuevo · v2 completo',
        'rotation640_baseline': 'Anterior · 640×480',
        'rotation640_final': 'Nuevo · 640×480',
        'rotation1280_baseline': 'Anterior · 1280×720',
        'rotation1280_final': 'Nuevo · 1280×720',
    }.get(label, label)
    line, = axes[0].plot(t, local[:,2], label=display_label)
    axes[1].plot(t, yaw, color=line.get_color())
    row = len(summary)
    axes[2].scatter(t[valid], np.full(valid.sum(),row), s=4, color=line.get_color())
    segments = np.split(indices, np.flatnonzero(np.diff(indices)>1)+1)
    stats = json.loads((directory/'summary.json').read_text())
    stats.update(run=label, display_label=display_label, valid_segments=[[float(t[s[0]]),float(t[s[-1]])] for s in segments],
                 forward_min=float(np.nanmin(local[:,2])), forward_max=float(np.nanmax(local[:,2])),
                 yaw_final=float(yaw[indices[-1]]), endpoint_norm=float(np.linalg.norm(p[indices[-1]]-p[indices[0]])))
    summary.append(stats)
axes[0].set_ylabel('Avance respecto del inicio [m]')
axes[0].legend(fontsize=7, loc='upper left', ncol=2)
axes[1].set_ylabel('Yaw relativo [°]')
axes[2].set_ylabel('Seguimiento válido')
axes[2].set_yticks(range(len(summary)), [s['display_label'].replace('movement_','').replace('_v2','') for s in summary], fontsize=7)
axes[2].set_xlabel('Tiempo de la grabación [s]')
for ax in axes:
    ax.grid(alpha=.2)
fig.suptitle(a.title)
fig.savefig(a.root/(a.output_name+'.png'), dpi=150)
(a.root/(a.output_name+'.json')).write_text(json.dumps(summary,indent=2))
for s in summary:
    print(s)
