#!/usr/bin/env python3
"""Control the combined deployment; task variants share this one project."""
import argparse
from concurrent.futures import ThreadPoolExecutor
import json
import os
from pathlib import Path
import secrets
import shlex
import subprocess
import sys
import time

ROOT=Path(__file__).resolve().parents[2]
CONFIG=Path(os.environ.get('GPT_BENCHMARK_FLEET_CONFIG',
    ROOT/'tools/remote/fleet.local.json'))
if not CONFIG.exists():
    CONFIG=ROOT/'tools/remote/fleet.json'
CFG=json.loads(CONFIG.read_text(encoding='utf-8'))
STATE=Path.home()/'.local/state/gpt-benchmark-combined'
TOKEN=STATE/'bridge.token'


def ssh(machine,*args):
    port=CFG['machines'][str(machine)]
    return ['ssh','-o','BatchMode=yes','-o','ConnectTimeout=12','-o','ServerAliveInterval=20','-o','ServerAliveCountMax=3',
            *(['-J',CFG['jump_host']] if CFG.get('jump_host') and str(machine) not in CFG.get('direct_machines', []) else []),'-o','ControlMaster=auto','-o','ControlPersist=3600','-o','ControlPath=/tmp/gptbench-combined-%p','-p',str(port),*args,CFG['user']+'@'+CFG['host']]


def remote(machine,args,**kwargs):
    return subprocess.run(ssh(machine)+[shlex.join(args)],check=True,**kwargs)


def ensure_bridge():
    STATE.mkdir(parents=True,exist_ok=True);STATE.chmod(0o700)
    if not TOKEN.exists():
        fd=os.open(TOKEN,os.O_WRONLY|os.O_CREAT|os.O_EXCL,0o600)
        with os.fdopen(fd,'w') as f:f.write(secrets.token_urlsafe(48)+'\n')
    sys.path.insert(0,str(ROOT/'src'))
    from robot_tasks.policies.model_config import load_gpt_settings
    from robot_tasks.policies.codex_bridge import CodexBridgeClient
    os.environ['GPT_BENCHMARK_CODEX_BRIDGE_URL']=f'http://127.0.0.1:{CFG["bridge_port"]}'
    os.environ['GPT_BENCHMARK_CODEX_BRIDGE_TOKEN_FILE']=str(TOKEN)
    settings=load_gpt_settings(ROOT/'configs/models/gpt.toml')
    def health():
        try:CodexBridgeClient(settings).require_chatgpt_login();return True
        except Exception:return False
    if not health():
        python=ROOT/'build/alignment_20261015/usd-env/bin/python'
        if not python.exists():python=Path(sys.executable)
        with (STATE/'bridge.log').open('ab') as log:
            p=subprocess.Popen([str(python),str(ROOT/'tools/remote/codex_bridge_server.py'),'--token-file',str(TOKEN),'--port',str(CFG['bridge_port'])],stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        (STATE/'bridge.pid').write_text(str(p.pid))
        for _ in range(25):
            if health():break
            if p.poll() is not None:raise RuntimeError('Bridge failed; inspect '+str(STATE/'bridge.log'))
            time.sleep(.3)
        else:raise RuntimeError('Bridge startup timed out')
    print('LOCAL_CODEX_BRIDGE_READY',flush=True)


def connect(machine):
    # Private RPC token authenticates this deployment, not the OpenAI account.
    remote(machine,['bash','-c',f'umask 077; mkdir -p {shlex.quote(CFG["runtime"])}; t=$(mktemp {shlex.quote(CFG["runtime"]+"/bridge.XXXXXX")}); cat > \"$t\"; mv \"$t\" {shlex.quote(CFG["runtime"]+"/bridge.token")}'],input=TOKEN.read_bytes())
    forward=f'127.0.0.1:{CFG["bridge_port"]}:127.0.0.1:{CFG["bridge_port"]}'
    result=subprocess.run(ssh(machine,'-O','forward','-R',forward),capture_output=True,text=True)
    # An existing healthy tunnel is acceptable; otherwise surface forwarding failure.
    health=['bash',CFG['project']+'/tools/remote/remote_entry.sh','-c',
            'from robot_tasks.policies.codex_bridge import create_codex_client;from robot_tasks.policies.model_config import load_gpt_settings;from pathlib import Path;create_codex_client(load_gpt_settings(Path("configs/models/gpt.toml"))).require_chatgpt_login();print("REMOTE_CODEX_BRIDGE_READY")']
    try:remote(machine,health)
    except subprocess.CalledProcessError:
        raise RuntimeError(f'Machine {machine} tunnel failed: '+result.stderr.strip())
    print('CONNECTED',machine,flush=True)


def main():
    p=argparse.ArgumentParser();p.add_argument('action',choices=['connect','sync','bootstrap','start','status','stop','logs','collect']);p.add_argument('--machine',choices=list(CFG['machines']));p.add_argument('--tasks',nargs='+');p.add_argument('--dry-run',action='store_true');p.add_argument('--max-calls',type=int);p.add_argument('--wall-time-s',type=int)
    p.add_argument('--deployment',choices=['combined'],default='combined',
                   help='One project supports original and _variant01 task IDs (_v3 is still accepted)')
    a=p.parse_args()
    if a.action in ('start','logs') and not a.machine:raise SystemExit('--machine is required for start/logs')
    machines=[a.machine] if a.machine else list(CFG['machines'])
    if a.action=='sync':
        # The current cluster is the sole deployment target.
        m=a.machine or CFG.get('default_machine', 'cluster');port=CFG['machines'][m]
        cmd=['rsync','-az','--checksum','--delete-delay','--partial-dir=.rsync-partial','--info=progress2','--no-owner','--no-group','--safe-links','--exclude-from='+str(ROOT/'tools/remote/rsync-excludes.txt'),'--stats','-e',shlex.join([x.replace('/tmp/gptbench-mengjq-%p','/tmp/gptbench-jump-%p') for x in ssh(m)[:-1]]),str(ROOT)+'/',CFG['user']+'@'+CFG['host']+':'+CFG['project']+'/']
        subprocess.run(cmd,check=True);return
    if a.action=='bootstrap':
        first=a.machine or CFG.get('default_machine', 'cluster')
        remote(first,['bash',CFG['project']+'/tools/remote/bootstrap.sh'])
        with ThreadPoolExecutor(max_workers=5) as pool:
            list(pool.map(lambda m: remote(m,['bash',CFG['project']+'/tools/remote/bootstrap.sh','--host-only']),[m for m in machines if m!=first]))
        return
    if a.action in ('connect','start') and not a.dry_run:
        ensure_bridge()
        with ThreadPoolExecutor(max_workers=5) as pool:list(pool.map(connect,machines))
    if a.action=='connect':return
    if a.action in ('start','logs') and not a.machine:raise SystemExit('--machine is required for start/logs')
    if a.action=='collect':
        dest=ROOT/'runs'/'remote_collected';dest.mkdir(parents=True,exist_ok=True)
        subprocess.run(['rsync','-a','-e',shlex.join(ssh(a.machine or CFG.get('default_machine','cluster'))[:-1]),CFG['user']+'@'+CFG['host']+':'+CFG['project']+'/runs/remote/',str(dest)+'/'],check=True);return
    for machine in machines:
        args=['bash',CFG['project']+'/tools/remote/remote_entry.sh',CFG['project']+'/tools/remote/remote_job.py',a.action]
        if a.tasks:args+=['--tasks',*a.tasks]
        if a.dry_run:args+=['--dry-run']
        if a.max_calls is not None:args+=['--max-calls',str(a.max_calls)]
        if a.wall_time_s is not None:args+=['--wall-time-s',str(a.wall_time_s)]
        print('MACHINE',machine,flush=True);remote(machine,args)

if __name__=='__main__':main()
