#!/usr/bin/env python3
"""ROS/Unix contract for an already-running xsim installed by the private Core fixture."""
import argparse, http.client, json, socket, threading, time
from pathlib import Path
class UnixHTTP(http.client.HTTPConnection):
 def __init__(self,path):super().__init__('localhost',timeout=5);self.path=path
 def connect(self):self.sock=socket.socket(socket.AF_UNIX,socket.SOCK_STREAM);self.sock.connect(self.path)
def main():
 p=argparse.ArgumentParser();p.add_argument('--socket',required=True);p.add_argument('--report',required=True);p.add_argument('--epoch-ns',type=int,required=True);a=p.parse_args()
 import rospy
 from geometry_msgs.msg import PoseStamped,TwistStamped,Twist
 from mavros_msgs.msg import State,PositionTarget
 from mavros_msgs.srv import CommandBool,SetMode
 from sensor_msgs.msg import PointCloud2
 from std_srvs.srv import Trigger
 from xgc2_lightweight_sim_msgs.srv import SetProvider
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
 for body in ('uav1','ugv1','mecanum1'):
  sub(body+'.truth','/xgc/simulation/body/'+body+'/pose',PoseStamped)
  sub(body+'.pose','/'+body+('/mavros/local_position/pose' if body=='uav1' else '/simulation/pose'),PoseStamped)
  sub(body+'.velocity','/'+body+('/mavros/local_position/velocity_local' if body=='uav1' else '/simulation/velocity'),TwistStamped)
 sub('state','/uav1/mavros/state',State);sub('points','/ugv1/simple_lidar/points',PointCloud2);sub('beams','/ugv1/simple_lidar/beams',PointCloud2);sub('reference','/xgc/scene/reference_cloud',PointCloud2)
 paused=mutate('/pause');base=get('/status');base_stamp=base['simulation_time_ns'];assert base_stamp>=a.epoch_ns
 providers={};report={'readiness':'Core actual /xsim /clock probe passed before provider enable','epoch_ns':a.epoch_ns,'models':{}}
 for body in ('uav1','ugv1','mecanum1'):
  service='/xgc/lightweight/providers/'+body;rospy.wait_for_service(service,5);providers[body]=rospy.ServiceProxy(service,SetProvider)
  r=providers[body](0,0);assert r.accepted and not r.enabled and r.generation==0
  r=providers[body](1,0);assert r.accepted and r.enabled and r.generation==1
 for body,pos in [('uav1',(2,-3,.2)),('ugv1',(0,1,0)),('mecanum1',(1,2,0))]:
  until(lambda:sample(body+'.pose') and sample(body+'.pose').header.stamp.to_nsec()==base_stamp,body+' initial pose')
  m=sample(body+'.pose');v=m.pose.position;assert max(abs(x-y) for x,y in zip((v.x,v.y,v.z),pos))<1e-12,(body,v,pos)
  assert m.header.frame_id==('map' if body=='uav1' else 'world')
  report['models'][body]={'initial_pose':[v.x,v.y,v.z],'initial_stamp_ns':m.header.stamp.to_nsec(),'frame':m.header.frame_id,'provider_start_generation':1}
 arm=rospy.ServiceProxy('/uav1/mavros/cmd/arming',CommandBool);mode=rospy.ServiceProxy('/uav1/mavros/set_mode',SetMode)
 assert arm(False).success;assert mode(0,'POSCTL').mode_sent
 report['models']['uav1']['arm_false_success']=True;report['models']['uav1']['mode_sent']=True
 pubs={b:rospy.Publisher('/'+b+'/cmd_vel',Twist,queue_size=1) for b in ('ugv1','mecanum1')}
 for pub in pubs.values():until(lambda:pub.get_num_connections()>0,'ground input')
 for pub in pubs.values():m=Twist();m.linear.x=.3;m.linear.y=.2;m.angular.z=.1;pub.publish(m)
 time.sleep(.04);stamp=mutate('/step',{'steps':200})['simulation_time_ns'];assert stamp==base_stamp+200000000
 for body in ('uav1','ugv1','mecanum1'):
  until(lambda:sample(body+'.pose') and sample(body+'.pose').header.stamp.to_nsec()==stamp,body+' step output')
  m=sample(body+'.pose');v=m.pose.position;report['models'][body]['after_200_steps_pose']=[v.x,v.y,v.z];report['models'][body]['after_200_steps_stamp_ns']=m.header.stamp.to_nsec()
 assert sample('ugv1.pose').pose.position.x>0;assert sample('mecanum1.pose').pose.position.x>1
 until(lambda:sample('points') and sample('beams') and sample('reference'),'integrated point clouds')
 cloud=sample('points');beams=sample('beams');ref=sample('reference');assert cloud.width>0 and beams.width==2700 and ref.width>0
 assert cloud.header.frame_id==beams.header.frame_id==ref.header.frame_id=='world'
 assert (cloud.header.stamp.to_nsec()-a.epoch_ns)%1000000==0
 assert [f.name for f in cloud.fields]==['x','y','z','vehicle_id'];assert [f.name for f in beams.fields]==['x','y','z','dx','dy','dz','range','hit','vehicle_id']
 report['sensor']={'backend':'cpu','points':cloud.width,'beam_count':beams.width,'reference_points':ref.width,'stamp_ns':cloud.header.stamp.to_nsec(),'frame':cloud.header.frame_id,'fields':[f.name for f in cloud.fields]}
 for body in providers:
  rospy.wait_for_service('/'+body+'/simulation/reset',5);assert rospy.ServiceProxy('/'+body+'/simulation/reset',Trigger)().success
  r=providers[body](0,0);assert r.generation==2
  stale=providers[body](2,1);assert not stale.accepted and stale.reason==1 and stale.generation==2
  report['models'][body]['reset_generation']=2;report['models'][body]['stale_stop_reason']=1
 stamp=mutate('/step',{'steps':200})['simulation_time_ns']
 for body,pos in [('ugv1',(0,1,0)),('mecanum1',(1,2,0))]:
  until(lambda:sample(body+'.pose') and sample(body+'.pose').header.stamp.to_nsec()==stamp,body+' reset generation output')
  v=sample(body+'.pose').pose.position;assert max(abs(x-y) for x,y in zip((v.x,v.y,v.z),pos))<1e-12
 added=mutate('/entities',{'entity':{'name':'temporary','kind':'scout'} });assert len(get('/entities')['entities'])==4
 mutate('/entities/'+str(added['entity_id']),{'generation':added['generation']},'DELETE');assert len(get('/entities')['entities'])==3
 assert get('/status')['paused'];report['paused_management_add_remove']=True
 Path(a.report).write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report))
if __name__=='__main__':main()
