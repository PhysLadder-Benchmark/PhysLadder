"""Plot measured trajectories from completed Isaac mechanism runs."""
import csv
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

root = Path(__file__).resolve().parents[1]
base = root/'results/experiments'
fig, axes = plt.subplots(1, 3, figsize=(13, 3.8), constrained_layout=True)
provenance = {}
for ax, kind in zip(axes, ['insertion', 'friction', 'spring']):
    runs = sorted(base.glob(kind+'-*'))
    if not runs or not (runs[-1]/'summary.json').exists():
        ax.text(.5, .5, 'No completed run', ha='center');ax.set_title(kind);continue
    run = runs[-1]
    summary = json.loads((run/'summary.json').read_text(encoding='utf-8'))
    with (run/'trajectory.csv').open(encoding='utf-8') as f:
        rows = [{k:float(v) for k,v in r.items()} for r in csv.DictReader(f)]
    for case in summary['checks']:
        rr = [r for r in rows if int(r['case_id'])==case['id']]
        x = np.array([r['time_s'] for r in rr])
        if kind=='insertion':
            y = np.array([r['z']*1000 for r in rr])
            label = f"gap {case['gap_m']*1000:g} mm, offset {case['offset_m']*1000:g} mm"
            if not case['wall_collision']:label += ', collision OFF'
            ylabel='Pin center height (mm)'
        elif kind=='friction':
            y = np.array([r['x']-case['origin'][0] for r in rr])
            label = f"mu={case['friction']:g}, mass={case['mass_kg']:g} kg"
            ylabel='Horizontal displacement (m)'
        else:
            y = np.array([(case['origin'][2]-r['z'])*1000 for r in rr])
            label=f"k={case['stiffness_N_per_m']:g} N/m"
            ylabel='Compression (mm)'
        ax.plot(x,y,label=label,linewidth=1.7)
    ax.set(title=kind.capitalize(), xlabel='Simulation time (s)', ylabel=ylabel)
    ax.grid(alpha=.2);ax.legend(fontsize=6.5,loc='best')
    provenance[kind]={'run':run.name,'passed':summary['passed']}
fig.suptitle('Isaac Sim object-level mechanism probes (no robot policy)',fontsize=12)
fig.savefig(base/'mechanism_curves.png',dpi=180)
(base/'plot_provenance.json').write_text(json.dumps(provenance,indent=2),encoding='utf-8')
print(json.dumps(provenance))
