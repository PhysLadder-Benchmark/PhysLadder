"""Fetch only RoboDojo's small Franka USD dependency set, without demonstrations."""
import concurrent.futures
import hashlib
import json
from pathlib import Path
import time
import urllib.request

root = Path(__file__).resolve().parents[1]
inventory = json.loads((root / 'results/sources/RoboDojo-Benchmark_RoboDojo_assets.json').read_text())
selected = [r for r in inventory if r['type'] == 'file'
            and r['path'].startswith('Assets/Robots/franka/')
            and '/.thumbs/' not in r['path']
            and Path(r['path']).suffix.lower() in ['.usd', '.usda', '.usdc', '.png', '.jpg', '.jpeg']]

def fetch(record):
    rel = record['path'].removeprefix('Assets/')
    dest = root / 'assets/robodojo' / rel
    dest.parent.mkdir(parents=True, exist_ok=True)
    url = 'https://huggingface.co/datasets/RoboDojo-Benchmark/RoboDojo/resolve/main/' + record['path']
    expected = record.get('lfs', {}).get('oid')
    for attempt in range(4):
        try:
            if dest.exists() and dest.stat().st_size == record['size']:
                data = dest.read_bytes()
            else:
                opener = urllib.request.build_opener(urllib.request.ProxyHandler())
                with opener.open(url, timeout=30) as r:
                    data = r.read()
            if len(data) != record['size']:
                raise ValueError('Size mismatch')
            digest = hashlib.sha256(data).hexdigest()
            if expected and digest != expected:
                raise ValueError('SHA-256 mismatch')
            dest.write_bytes(data)
            return {'path': rel, 'size': len(data), 'sha256': digest, 'status': 'downloaded'}
        except Exception as e:
            error = str(e)
            time.sleep(min(attempt + 1, 3))
    return {'path': rel, 'status': 'failed', 'error': error}

print(f"Fetching {len(selected)} files, {sum(r['size'] for r in selected)/1e6:.1f} MB", flush=True)
with concurrent.futures.ThreadPoolExecutor(max_workers=4) as pool:
    results = list(pool.map(fetch, selected))
(root / 'results/sources/franka_download.json').write_text(json.dumps(results, indent=2))
print(json.dumps({'downloaded': sum(r['status'] == 'downloaded' for r in results),
                  'failed': [r for r in results if r['status'] == 'failed']}), flush=True)
