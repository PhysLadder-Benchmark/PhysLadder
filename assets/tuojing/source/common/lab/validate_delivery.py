"""Run finite independent Isaac processes; logs and per-case receipts are retained."""
from pathlib import Path
import subprocess,json,shutil
r=Path(__file__).resolve().parent
cases=[('motion',['--steps','240','--demo']),('axis51',['--steps','60','--scene','tuojing_lab_axis_51cm.usda']),('fov60',['--steps','60','--capture','--hfov','60']),('fov85',['--steps','60','--capture','--hfov','85']),('wrist_left',['--steps','120','--camera','WristLeft','--capture']),('wrist_right',['--steps','120','--camera','WristRight','--capture']),('main',['--steps','120','--capture'])]
results=[]
for name,args in cases:
 with (r/(name+'_delivery.log')).open('w') as log:
  subprocess.run(['/isaac-sim/python.sh',str(r/'launch_isaac.py'),'--headless',*args],stdout=log,stderr=subprocess.STDOUT,check=True,cwd=r)
 data=json.loads((r/'reopen_validation.json').read_text());assert data['fresh_process_load'] and not data['missing_dependencies'];shutil.copy2(r/'reopen_validation.json',r/(name+'_validation.json'));results.append({'case':name,'status':'passed','steps':data['steps']});print(name,'PASS',flush=True)
(r/'delivery_validation.json').write_text(json.dumps(results,indent=2))
