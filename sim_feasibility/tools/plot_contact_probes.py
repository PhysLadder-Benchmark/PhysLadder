"""Plots of the reviewed FEM and thread probes, with explicit run provenance."""
import csv
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

base=Path(__file__).resolve().parents[1]/'results/experiments'
runs={'deformable':'deformable_probe-20260925-221407', 'thread':'screw_probe-20260925-223020'}
fig, axes=plt.subplots(1,2,figsize=(11,3.8),constrained_layout=True)
for ax,(kind,name) in zip(axes,runs.items()):
    folder=base/name
    with (folder/'trajectory.csv').open() as f: rows=list(csv.DictReader(f))
    summary=json.loads((folder/'summary.json').read_text())
    for case in summary['cases']:
        rr=[r for r in rows if int(r['case_id'])==case['id']]
        t=np.array([float(r['time_s']) for r in rr])
        if kind=='deformable':
            y=(case['baseline_height_m']-np.array([float(r['height_m']) for r in rr]))*1000
            label=f"E={case['youngs_modulus_Pa']/1000:g} kPa"
        else:
            y=np.array([float(r['z']) for r in rr])*1000
            label=f"Target angular speed {case['target_speed_rad_s']:g} rad/s"
            if not case.get('bolt_collision', True):label += ', bolt collision OFF'
        ax.plot(t,y,label=label)
    if kind=='deformable':
        ax.axvspan(.5,1.6,color='grey',alpha=.12,label='2 N downward load')
        ax.set(title='Elastic FEM blocks (not bread damage)',ylabel='Height compression (mm)')
    else:
        ax.set(title='Thread probe: coupling NOT verified',ylabel='Nut root height (mm)')
    ax.set_xlabel('Simulation time (s)');ax.grid(alpha=.2);ax.legend(fontsize=8)
fig.savefig(base/'contact_probe_curves.png',dpi=160)
(base/'contact_plot_provenance.json').write_text(json.dumps(runs,indent=2))
print(json.dumps(runs))
