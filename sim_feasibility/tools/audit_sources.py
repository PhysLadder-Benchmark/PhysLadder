"""Record source revisions and regenerate the human-reviewed screening table.

Run with sim-research Python. This is a static audit, not a physics run.
"""
import csv
import json
from pathlib import Path
import re
import subprocess

root = Path(__file__).resolve().parents[1]
sources = root / 'results/sources'
sources.mkdir(parents=True, exist_ok=True)
repositories = []
for path in sorted((root/'third_party').iterdir()):
    if not (path/'.git').exists():
        continue
    def git(*args):
        return subprocess.check_output(['git','-C',str(path),*args],text=True).strip()
    repositories.append({'name':path.name,'url':git('remote','get-url','origin'),
                         'commit':git('rev-parse','HEAD'),'path':str(path.relative_to(root)),
                         'status':'source_checked_out','runtime_tested':False})
(sources/'repositories.json').write_text(json.dumps(repositories,indent=2),encoding='utf-8')

rows=[]
for line in (root/'docs/task_screening.md').read_text(encoding='utf-8').splitlines():
    if not re.match(r'\| L[123]-\d\d ',line):
        continue
    cells=[x.strip() for x in line.strip('|').split('|')]
    identifier,name=cells[0].split(' ',1)
    rows.append({'id':identifier,'task':name,
                 'conclusion':'可行性较强' if cells[1].startswith('可行性较强') else '短期内难以实现',
                 'qualification':cells[1],'reference':cells[2],'gap_and_test':cells[3],
                 'runtime_status':'未跑完整任务'})
if len(rows)!=24 or len({r['id'] for r in rows})!=24:
    raise RuntimeError('Task table no longer matches the reviewed 24-task source; review before exporting')
(root/'docs/task_screening.json').write_text(json.dumps(rows,ensure_ascii=False,indent=2),encoding='utf-8')
with (root/'docs/task_screening.csv').open('w',encoding='utf-8-sig',newline='') as f:
    writer=csv.DictWriter(f,fieldnames=list(rows[0]));writer.writeheader();writer.writerows(rows)
print(json.dumps({'repositories':len(repositories),'tasks':len(rows),
                  'strong':sum(r['conclusion']=='可行性较强' for r in rows),
                  'scope':'static review; no task runtime success inferred'},ensure_ascii=False))
