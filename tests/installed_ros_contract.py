#!/usr/bin/env python3
"""ROS/Unix contract for an already-running xsim installed by the private Core fixture."""
import argparse, json, os, threading, time, xmlrpc.client
from pathlib import Path
from native_http import NativeProvider, UnixHTTP, assert_retired_ros_surfaces_absent
def main():
 p=argparse.ArgumentParser();p.add_argument('--socket',required=True);p.add_argument('--report',required=True);p.add_argument('--epoch-ns',type=int,required=True);a=p.parse_args()
 import rospy
 from geometry_msgs.msg import PoseStamped,TwistStamped,Twist
 from mavros_msgs.msg import State,PositionTarget
 from mavros_msgs.srv import CommandBool,SetMode
 from sensor_msgs.msg import PointCloud2
 rospy.init_node('xsim_installed_contract',disable_signals=True)
 serial=0
 def http(method,path,body=None):
  c=UnixHTTP(a.socket)
  try:c.request(method,path,None if body is None else json.dumps(body),{'Content-Type':'application/json'});r=c.getresponse();return r.status,json.loads(r.read())
  finally:c.close()
 def get(path):
  code,data=http('GET',path);assert code==200,(code,data);return data
 def until(fn,label,seconds=8):
  end=time.monotonic()+seconds
  while time.monotonic()<end:
   result=fn()
   if result:return result
   time.sleep(.005)
  raise AssertionError('timeout '+label)
 def mutate(path,body=None,method='POST'):
  nonlocal serial
  serial+=1;key=str(serial);payload={'instance_id':'private-installed','request_id':key,**(body or {})}
  code,result=http(method,path,payload);assert code==202,(code,result)
  r=until(lambda:get('/requests/'+key) if get('/requests/'+key)['phase'] in ('applied','failed','cancelled') else None,path)
  assert r['phase']=='applied' and r['result']['success'],r;return r['result']
 latest={};lock=threading.Lock();subs=[]
 def sub(key,topic,kind):
  def cb(msg):
   with lock:latest[key]=msg
  subs.append(rospy.Subscriber(topic,kind,cb,queue_size=32))
 def sample(key):
  with lock:return latest.get(key)
 pose_topics={body:('/'+body+'/mavros/local_position/pose' if body=='uav1' else '/'+body+'/pose') for body in ('uav1','ugv1','mecanum1')}
 for body in ('uav1','ugv1','mecanum1'):
  sub(body+'.mocap','/'+body+'/pose',PoseStamped)
  sub(body+'.pose',pose_topics[body],PoseStamped)
  sub(body+'.velocity',('/'+body+'/mavros/local_position/velocity_local' if body=='uav1' else '/'+body+'/twist'),TwistStamped)
 sub('state','/uav1/mavros/state',State);sub('points','/ugv1/simple_lidar/points',PointCloud2);sub('beams','/ugv1/simple_lidar/beams',PointCloud2);sub('reference','/xgc/scene/reference_cloud',PointCloud2)
 paused=mutate('/pause');base=get('/status');base_stamp=base['simulation_time_ns'];assert base_stamp>=a.epoch_ns
 providers={};report={'readiness':'Core actual /xsim /clock probe passed before provider enable','epoch_ns':a.epoch_ns,'models':{}}
 # Core's frozen fixture uses independent 1e-7 m mocap position noise per axis.
 # MAVROS local position remains the exact native FCU observation; ground pose
 # is directly published as canonical localization and uses an explicit eight-sigma noise bound.
 mocap_noise_std=1e-7;measurement_tolerance=8*mocap_noise_std+1e-12
 master=xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI'])
 def ros_graph():
  code,description,state=master.getSystemState('/xsim_installed_contract');assert code==1,description
  return assert_retired_ros_surfaces_absent(state,providers,('ugv1','mecanum1'))
 for body in ('uav1','ugv1','mecanum1'):
  providers[body]=NativeProvider(a.socket,body)
  r=providers[body].read();assert not r['enabled'] and r['generation']==0
  r=providers[body].start(0);assert r['success'] and r['enabled'] and r['generation']==1
 for body,pos in [('uav1',(2,-3,.2)),('ugv1',(0,1,0)),('mecanum1',(1,2,0))]:
  until(lambda:sample(body+'.pose') and sample(body+'.pose').header.stamp.to_nsec()==base_stamp,body+' initial pose')
  m=sample(body+'.pose');v=m.pose.position;tolerance=1e-12 if body=='uav1' else measurement_tolerance
  assert max(abs(x-y) for x,y in zip((v.x,v.y,v.z),pos))<tolerance,(body,v,pos,tolerance)
  assert m.header.frame_id==('map' if body=='uav1' else 'world')
  until(lambda:sample(body+'.mocap') and sample(body+'.mocap').header.stamp.to_nsec()==base_stamp,body+' initial mocap measurement')
  raw=sample(body+'.mocap');raw_position=raw.pose.position
  noise_error=[x-y for x,y in zip((raw_position.x,raw_position.y,raw_position.z),pos)]
  assert raw.header.frame_id=='world' and 1e-12<max(abs(x) for x in noise_error)<measurement_tolerance,(body,noise_error)
  report['models'][body]={'initial_pose':[v.x,v.y,v.z],'initial_stamp_ns':m.header.stamp.to_nsec(),'frame':m.header.frame_id,'provider_start_generation':1,
                         'pose_measurement_topic':pose_topics[body],'pose_tolerance':tolerance,'mocap_noise_std':mocap_noise_std,'initial_mocap_noise_error':noise_error}
 report['ros_graph']=ros_graph()
 arm=rospy.ServiceProxy('/uav1/mavros/cmd/arming',CommandBool);mode=rospy.ServiceProxy('/uav1/mavros/set_mode',SetMode)
 assert arm(False).success;assert mode(0,'POSCTL').mode_sent
 report['models']['uav1']['arm_false_success']=True;report['models']['uav1']['mode_sent']=True
 pubs={b:rospy.Publisher('/'+b+'/cmd_vel',Twist,queue_size=1) for b in ('ugv1','mecanum1')}
 for pub in pubs.values():until(lambda:pub.get_num_connections()>0,'ground input')
 for pub in pubs.values():m=Twist();m.linear.x=.3;m.linear.y=.2;m.angular.z=.1;pub.publish(m)
 time.sleep(.04);stamp=mutate('/step',{'steps':200})['simulation_time_ns'];assert stamp==base_stamp+200*base['model_step_ns']
 for body in ('uav1','ugv1','mecanum1'):
  until(lambda:sample(body+'.pose') and sample(body+'.pose').header.stamp.to_nsec()==stamp,body+' step output')
  m=sample(body+'.pose');v=m.pose.position;report['models'][body]['after_200_steps_pose']=[v.x,v.y,v.z];report['models'][body]['after_200_steps_stamp_ns']=m.header.stamp.to_nsec()
 assert sample('ugv1.pose').pose.position.x>0;assert sample('mecanum1.pose').pose.position.x>1
 until(lambda:sample('points') and sample('beams') and sample('reference'),'integrated point clouds')
 cloud=sample('points');beams=sample('beams');ref=sample('reference');assert cloud.width>0 and beams.width==2700 and ref.width>0
 assert cloud.header.frame_id==beams.header.frame_id==ref.header.frame_id=='world'
 assert cloud.header.stamp.to_nsec() >= a.epoch_ns
 assert [f.name for f in cloud.fields]==['x','y','z','vehicle_id'];assert [f.name for f in beams.fields]==['x','y','z','dx','dy','dz','range','hit','vehicle_id']
 report['sensor']={'backend':'cpu','points':cloud.width,'beam_count':beams.width,'reference_points':ref.width,'stamp_ns':cloud.header.stamp.to_nsec(),'frame':cloud.header.frame_id,'fields':[f.name for f in cloud.fields]}
 for body in providers:
  before=providers[body].read()['generation'];r=mutate('/reset',{'entity_id':providers[body].entity_id,'generation':before})
  assert r['entity_id']==providers[body].entity_id and r['generation']==before+1 and r['enabled'],r
  r=providers[body].read();assert r['generation']==2
  stale=providers[body].stop(1);assert not stale['success'] and stale['reason']==1 and stale['generation']==2
  report['models'][body]['reset_generation']=2;report['models'][body]['stale_stop_reason']=1
 stamp=mutate('/step',{'steps':200})['simulation_time_ns']
 for body,pos in [('ugv1',(0,1,0)),('mecanum1',(1,2,0))]:
  until(lambda:sample(body+'.pose') and sample(body+'.pose').header.stamp.to_nsec()==stamp,body+' reset generation output')
  v=sample(body+'.pose').pose.position;assert max(abs(x-y) for x,y in zip((v.x,v.y,v.z),pos))<measurement_tolerance,(body,v,pos)
  report['models'][body]['reset_measured_pose']=[v.x,v.y,v.z]
 added=mutate('/entities',{'entity':{'name':'temporary','kind':'scout'} });assert len(get('/entities')['entities'])==4
 mutate('/entities/'+str(added['entity_id']),{'generation':added['generation']},'DELETE');assert len(get('/entities')['entities'])==3
 assert get('/status')['paused'];report['paused_management_add_remove']=True
 report['ros_graph']=ros_graph()
 Path(a.report).write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report))
if __name__=='__main__':main()
