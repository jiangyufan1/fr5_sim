#!/usr/bin/python3
import subprocess,time,signal,json,sqlite3
from pathlib import Path
root=Path('/data/Projects/fr5_sim');out=root/'outputs'/('continuous_trial_'+time.strftime('%Y%m%d_%H%M%S'));out.mkdir(parents=True)
subprocess.run(['/usr/bin/python3',str(root/'scripts/check_gazebo_runtime.py')],check=True)
log=open(out/'recorder.log','w');bag=subprocess.Popen(['ros2','bag','record','-o',str(out/'rosbag'),'/clock','/joint_states','/wrist_camera/image','/wrist_camera/camera_info','/fairino5_controller/controller_state','/gripper_controller/controller_state','/tf','/tf_static'],stdout=log,stderr=subprocess.STDOUT)
result={'success':False,'continuous_recording':True,'phases':[],'simulation_only':True}
try:
 time.sleep(2)
 assert bag.poll() is None
 for script,extra in [('pick_trial.py',['--execute']),('lift_trial.py',[]),('place_trial.py',[])]:
  logfile=out/(script+'.log')
  with logfile.open('w') as f:subprocess.run(['/usr/bin/python3',str(root/'scripts'/script),*extra],stdout=f,stderr=subprocess.STDOUT,check=True)
  phase=next(line[7:] for line in logfile.read_text().splitlines() if line.startswith('OUTPUT '))
  result['phases'].append(phase);print('DONE',script,flush=True)
 result['success']=json.loads((Path(result['phases'][-1])/'result.json').read_text())['success']
finally:
 bag.send_signal(signal.SIGINT)
 try:bag.wait(timeout=15)
 except subprocess.TimeoutExpired:bag.terminate();bag.wait(timeout=5)
 dbs=list((out/'rosbag').glob('*.db3'))
 if dbs:
  con=sqlite3.connect(str(dbs[0]));result['topics']=dict(con.execute('select topics.name,count(messages.id) from topics left join messages on topics.id=messages.topic_id group by topics.name'))
 result['limitations']=['Raw ROS bag; not yet converted/resampled for ACT','Image and state header timestamps retained; synchronization quality requires validation','Ground-truth targeting and provisional hardware geometry']
 (out/'result.json').write_text(json.dumps(result,indent=2));print('OUTPUT',out,flush=True)
