#!/usr/bin/env python3
"""Fetch, verify and locate the versioned visual scene bundle (standard library)."""
import argparse,hashlib,json,os,shutil,subprocess,sys,tarfile,tempfile
from pathlib import Path
REPO=Path(__file__).resolve().parents[1]
META=REPO/'docs/visual_alignment/release.json'
def sha(p):
 h=hashlib.sha256()
 with p.open('rb') as f:
  for b in iter(lambda:f.read(1024*1024),b''):h.update(b)
 return h.hexdigest()
def verify(root):
 records=json.loads((root/'SHA256SUMS.json').read_text())
 for name,digest in records.items():
  path=(root/name).resolve()
  if not path.is_relative_to(root.resolve()) or not path.is_file() or sha(path)!=digest:raise RuntimeError('Missing/corrupt file: '+name)
 return len(records)
def main():
 ap=argparse.ArgumentParser(description=__doc__);ap.add_argument('--root',type=Path,default=REPO/'.cache/visual_alignment_r3_20261014')
 sp=ap.add_subparsers(dest='command',required=True)
 f=sp.add_parser('fetch');f.add_argument('--archive',type=Path);f.add_argument('--ssh',help='SSH host or configured alias with shared-disk access');f.add_argument('--port',type=int,help='Optional port override; omit to honor SSH alias config')
 sp.add_parser('verify');sp.add_parser('list')
 p=sp.add_parser('scene');p.add_argument('task');p.add_argument('--kind',choices=['static','video','environment','task_objects'],default='static')
 a=ap.parse_args();meta=json.loads(META.read_text());root=a.root.resolve()
 if a.command=='fetch':
  if root.exists():print(f'Existing bundle verified: {verify(root)} files; {root}');return
  root.parent.mkdir(parents=True,exist_ok=True)
  with tempfile.TemporaryDirectory(prefix='visual-fetch-',dir=root.parent) as tmp:
   tmp=Path(tmp);archive=a.archive or Path(meta['archive']['shared_path'])
   if not archive.is_file():
    if not a.ssh:ap.error('Archive not on this machine. Pass --archive FILE or --ssh USER@HOST --port PORT.')
    archive=tmp/'bundle.tar.gz'
    subprocess.run(['scp',*(['-P',str(a.port)] if a.port is not None else []),f"{a.ssh}:{meta['archive']['shared_path']}",str(archive)],check=True)
   if sha(archive)!=meta['archive']['sha256']:raise RuntimeError('Archive SHA-256 mismatch')
   staging=tmp/'unpacked';staging.mkdir()
   with tarfile.open(archive,'r:gz') as tar:
    for member in tar.getmembers():
     dest=(staging/member.name).resolve()
     if not dest.is_relative_to(staging.resolve()) or not (member.isfile() or member.isdir()):raise RuntimeError('Unsafe archive member: '+member.name)
    tar.extractall(staging)
   candidate=staging/meta['archive']['root_directory'];count=verify(candidate)
   os.rename(candidate,root)
  print(f'Installed and verified {count} files: {root}')
 elif a.command=='verify':print(f'OK: {verify(root)} files')
 else:
  index=json.loads((root/'scene_manifest.json').read_text())['tasks']
  if a.command=='list':
   for task,entry in sorted(index.items()):print(task,', '.join(k for k in ['static','video','environment','task_objects'] if k in entry))
  else:
   if a.task not in index or a.kind not in index[a.task]:ap.error('This task/entry is not in the release; run list. L3-11 has only its preserved static scene.')
   print(root/index[a.task][a.kind])
if __name__=='__main__':main()
