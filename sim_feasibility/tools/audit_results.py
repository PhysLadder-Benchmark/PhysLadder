"""Verify curated evidence, preserve original Python bytes, and index diagnostics.

Curated run IDs are deliberate: later partial/debug runs do not replace evidence.
"""
from pathlib import Path
import hashlib
import json

root = Path(__file__).resolve().parents[1]
base = root/'results/experiments'
selected = {
    'insertion':'insertion-20260925-215330',
    'friction':'friction-20260925-215528',
    'spring':'spring-20260925-220424',
    'deformable_probe':'deformable_probe-20260925-221407',
    'robot_smoke':'robot_smoke-20260925-215756',
    'screw_probe':'screw_probe-20260925-223020',
}
index = []
for kind, name in selected.items():
    folder = base/name
    raw = (folder/'run_manifest.json').read_bytes()
    manifest = json.loads(raw.decode('utf-16' if raw.startswith(b'\xff\xfe') else 'utf-8-sig'))
    source_file = root/'experiments'/('rigid_mechanisms.py' if kind in ['insertion','friction','spring'] else kind+'.py')
    source = source_file.read_bytes()
    variants = [source, source.replace(b', clipping_range=(0.005, 100)', b'')]
    if (folder/'script_snapshot.py').exists():variants.append((folder/'script_snapshot.py').read_bytes())
    original = next((v for v in variants if hashlib.sha256(v).hexdigest()==manifest['script_sha256'].lower()), None)
    if original is None:raise RuntimeError(f'Cannot verify original script bytes: {name}')
    (folder/'script_snapshot.py').write_bytes(original)
    (folder/'runtime_config.py').write_bytes((root/'experiments/runtime_config.py').read_bytes())
    summary = json.loads((folder/'summary.json').read_text())
    errors, physics_warnings = 0, 0
    with (folder/'run.log').open(encoding='utf-16',errors='replace') as f:
        for line in f:
            errors += '[Error]' in line or '[Fatal]' in line
            physics_warnings += '[Warning] [omni.physx' in line
    if errors:raise RuntimeError(f'Runtime errors invalidate the selected evidence: {name} ({errors})')
    if kind!='screw_probe' and not summary.get('passed'):raise RuntimeError(f'Failed checks: {name}')
    index.append({'kind':kind,'run':name,'script_sha256_verified':True,
                  'reported_pass':summary.get('passed'),
                  'conclusion':'螺纹机制未验证' if kind=='screw_probe' else '基础检查通过',
                  'error_lines':errors,'physx_warning_lines':physics_warnings,'scope':summary['scope']})
(base/'reviewed_runs.json').write_text(json.dumps(index,ensure_ascii=False,indent=2))
print(json.dumps({'reviewed_runs':len(index),'runtime_errors':sum(r['error_lines'] for r in index),
                  'mechanism_checks_passed':4,'robot_smoke_passed':1,'thread_verified':False}))
