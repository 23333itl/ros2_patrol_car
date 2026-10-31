#!/usr/bin/env python3
"""Run three isolated 1510-s simulator patrols and one controlled revisit test.

Source the workspace first. This test publishes navigation/motion requests only
inside its own Gazebo partitions and ROS domains; never use a hardware domain.
"""
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import argparse,json,os,signal,subprocess,sys,threading,time
parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--output',type=Path,required=True)
parser.add_argument('--profile',default='simulation_stable')
parser.add_argument('--domain-start',type=int,default=151)
args=parser.parse_args();root=args.output.resolve();root.mkdir(parents=True,exist_ok=False)
workspace=Path(__file__).resolve().parents[4]
stop=threading.Event()
signal.signal(signal.SIGINT,lambda *_:stop.set());signal.signal(signal.SIGTERM,lambda *_:stop.set())
def run(item):
 name,domain,controlled=item;folder=root/name;folder.mkdir()
 env=os.environ.copy();env.update(ROS_DOMAIN_ID=str(domain),ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',GZ_PARTITION='maze_fix_'+root.name+'_'+name,ROS_LOG_DIR=str(folder/'ros_logs'),GZ_HOMEDIR=str(folder/'gz'),MPLCONFIGDIR=str(folder/'mpl'),OMP_NUM_THREADS='1',OPENBLAS_NUM_THREADS='1')
 processes=[];streams=[];result=dict(status='incomplete',profile=args.profile)
 def start(label,command):
  f=(folder/(label+'.log')).open('w');streams.append(f)
  p=subprocess.Popen(command,env=env,stdout=f,stderr=f,start_new_session=True);processes.append(p);return p
 try:
  # Stagger discovery/configuration, so four Nav2 managers do not all start at once.
  if stop.wait(5 * (domain - args.domain_start)):return name,result
  launch=start('launch',['ros2','launch','maze_explorer','explore_maze.launch.py','world:='+str(workspace/'maze_output/maze.world'),'slam_profile:='+args.profile,'rviz:=false','gz_gui:=false','explore:='+('false' if controlled else 'true'),'map_save_path:='+str(folder/'map')])
  stop.wait(3)
  recorder=start('recorder',['ros2','run','maze_explorer','record_mapping_diagnostics.py','--duration','720' if controlled else '1510','--wall-timeout','3000','--seed','1980958899011398951','--run',str(max(1,domain-args.domain_start)),'--output',str(folder/'recording')])
  if controlled:start('route',[sys.executable,str(Path(__file__).with_name('drive_route.py')),str(folder/'route_events.json')])
  began=notice=time.monotonic();ready=False
  while recorder.poll() is None and not stop.wait(.5):
   if launch.poll() is not None:raise RuntimeError('Launch exited')
   if not ready:
    ready='Managed nodes are active' in (folder/'launch.log').read_text(errors='replace')
    if not ready and time.monotonic()-began>90:raise RuntimeError('Nav2 lifecycle startup failed; not a patrol acceptance run')
   if time.monotonic()-began>3050:raise RuntimeError('Wall timeout')
   if time.monotonic()-notice>=45:
    print(name,'wall seconds',round(time.monotonic()-began),flush=True);notice=time.monotonic()
  result.update(status='recorded' if recorder.poll()==0 else 'incomplete',recorder_exit=recorder.poll())
 except Exception as error:result['error']=str(error)
 finally:
  for p in reversed(processes):
   if p.poll() is None:
    os.killpg(p.pid,signal.SIGINT)
    try:p.wait(timeout=20)
    except subprocess.TimeoutExpired:
     os.killpg(p.pid,signal.SIGTERM)
     try:p.wait(timeout=5)
     except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
  for f in streams:f.close()
  (folder/'trial.json').write_text(json.dumps(result,indent=2));print(name,result,flush=True)
 return name,result
items=[('controlled',args.domain_start,True)]+[(f'navigation_{i}',args.domain_start+i,False) for i in range(1,4)]
with ThreadPoolExecutor(max_workers=4) as pool:results=dict(pool.map(run,items))
(root/'trials.json').write_text(json.dumps(results,indent=2))
