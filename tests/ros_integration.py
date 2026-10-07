#!/usr/bin/env python3
"""Private ROS master + installed xsim. No station, Host or Adapter mutation."""
import argparse, concurrent.futures, json, math, os, signal, socket, subprocess, threading, time, xmlrpc.client
from pathlib import Path
from http import client as http_client
from native_http import NativeProvider, UnixHTTP, assert_retired_ros_surfaces_absent


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--xsim', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--replays', required=True)
    parser.add_argument('--slow-service-test', required=True)
    args = parser.parse_args()
    out = Path(args.output)
    out.mkdir(parents=True, exist_ok=True)
    with socket.socket() as free:
        free.bind(('127.0.0.1', 0))
        port = free.getsockname()[1]
    os.environ.update(ROS_MASTER_URI=f'http://127.0.0.1:{port}', ROS_IP='127.0.0.1', ROS_HOSTNAME='127.0.0.1', ROS_HOME=str(out/'ros-home'), ROS_LOG_DIR=str(out/'ros-log'))
    children = []
    logs = []
    def spawn(cmd, name):
        log = open(out/name, 'w'); logs.append(log)
        p = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        children.append(p)
        return p
    def until(fn, label, seconds=8):
        end = time.monotonic()+seconds
        while time.monotonic()<end:
            try:
                result = fn()
                if result: return result
            except (OSError, http_client.HTTPException, xmlrpc.client.Error):
                pass
            time.sleep(.005)
        raise AssertionError('timeout: '+label)
    try:
        master = spawn(['roscore', '-p', str(port)], 'roscore.log')
        rpc = xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI'])
        until(lambda: rpc.getPid('/xsim_private_validation')[0] == 1, 'private master')
        import rospy
        from geometry_msgs.msg import PoseStamped, TwistStamped, Twist
        from mavros_msgs.msg import State, PositionTarget, AttitudeTarget
        from mavros_msgs.srv import CommandBool, CommandLong, SetMode
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import Imu, PointCloud2
        rospy.set_param('/use_sim_time', True)
        rospy.init_node('xsim_private_contract', disable_signals=True)
        epoch = 1700000000000000000
        instance = 'xsim-private-contract'
        config = {'instance_id':instance, 'epoch_ns':epoch, 'model_step_ns':1000000, 'output_period_ns':10000000,
                  'telemetry_rates_hz':{key:100 for key in ('localization','local','imu','imu_raw','state','extended_state','target')},
                  'paused':True, 'publish_clock':True,
                  'scene':{'obstacles':[{'type':'box','position':[5,0,0], 'size':[1,6,4]}]},
                  'entities':[{'name':kind, 'kind':kind, 'position':[2,-1,0], 'yaw':.35,
                               'ros':{'mocap_noise':[0,0,0],'mocap_seed':1},
                               **({'local_origin':[2,-1,0], 'sensor':{'backend':'cpu','rate_hz':20,'h_res':72,'v_res':16,'h_fov_deg':90,'v_fov_deg':30}} if kind=='fs150' else {})}
                              for kind in ('fs150','scout','mecanum')]}
        (out/'world.json').write_text(json.dumps(config, indent=2))
        sock = str(out/'world.sock')
        server = spawn([args.xsim,'--config',str(out/'world.json'),'--socket',sock], 'xsim.log')
        serial = 0
        def http(method, path, body=None):
            conn=UnixHTTP(sock)
            try:
                conn.request(method,path,json.dumps(body) if body is not None else None, {'Content-Type':'application/json'})
                response=conn.getresponse(); data=json.loads(response.read())
                return response.status,data
            finally: conn.close()
        def get(path):
            code,data=http('GET',path); assert code==200,(code,data);return data
        def mutation(path, body=None, method='POST'):
            nonlocal serial
            serial+=1; payload={'instance_id':instance,'request_id':str(serial), **(body or {})}
            code,data=http(method,path,payload);assert code==202,(code,data)
            def applied():
                result=get('/requests/'+str(serial))
                return result if result['phase'] in ('applied','cancelled','failed') else None
            receipt=until(applied,path,10)
            assert receipt['phase']=='applied' and receipt['result']['success'],receipt
            return receipt['result']
        until(lambda: server.poll() is None and Path(sock).exists(), 'xsim socket')
        until(lambda: len(get('/entities')['entities'])==3,'initial roster')
        state_lock=threading.Lock(); latest={}; observed={}; subscriptions=[]
        def subscribe(key, topic, typ):
            def record(m):
                with state_lock:
                    latest[key]=m; observed.setdefault(key,[]).append(m)
            subscriptions.append(rospy.Subscriber(topic,typ,record,queue_size=1000))
        for kind in ('fs150','scout','mecanum'):
            base='/'+kind
            subscribe(kind+'.mocap',base+'/pose',PoseStamped)
            if kind=='fs150':
                subscribe(kind+'.pose',base+'/mavros/local_position/pose',PoseStamped)
                subscribe(kind+'.velocity',base+'/mavros/local_position/velocity_local',TwistStamped)
                subscribe(kind+'.odom',base+'/mavros/local_position/odom',Odometry)
        subscribe('fs150.state','/fs150/mavros/state',State)
        subscribe('fs150.imu','/fs150/mavros/imu/data',Imu)
        subscribe('fs150.target','/fs150/mavros/setpoint_raw/target_attitude',AttitudeTarget)
        subscribe('fs150.cloud','/fs150/cloud',PointCloud2)
        subscribe('fs150.raw_imu','/fs150/mavros/imu/data_raw',Imu)
        for kind in ('fs150','scout','mecanum'):
            subscribe(kind+'.mocap_velocity','/'+kind+'/twist',TwistStamped)
        # A paused world emits a stamp once; wait for real TCPROS connections.
        until(lambda: all(sub.get_num_connections() > 0 for sub in subscriptions),
              'direct canonical and FCU connections')
        providers={}
        for kind in ('fs150','scout','mecanum'):
            providers[kind]=NativeProvider(sock,kind)
            r=providers[kind].read();assert not r['enabled'] and r['generation']==0
        rospy.wait_for_service('/fs150/mavros/cmd/arming',5)
        offline=rospy.ServiceProxy('/fs150/mavros/cmd/arming',CommandBool)(True)
        assert not offline.success and offline.result==4
        time.sleep(.15)
        for provider in providers.values():
            r=provider.start(0);assert r['success'] and r['enabled'] and r['generation']==1
        # Both accepted active-generation fences preserve the running provider.
        for generation in (1,0):
            r=providers['fs150'].start(generation)
            assert r['success'] and r['enabled'] and r['generation']==1,r
        provider_path='/entities/'+str(providers['fs150'].entity_id)+'/provider'
        retry_payload={'instance_id':instance,'request_id':'provider-idempotent',
                       'generation':1,'action':'start','timeout_ms':5000}
        code,receipt=http('POST',provider_path,retry_payload);assert code==202,(code,receipt)
        def provider_applied():
            receipt=get('/requests/provider-idempotent')
            return receipt if receipt['phase'] in ('applied','cancelled','failed') else None
        receipt=until(provider_applied,'provider idempotent applied')
        assert receipt['phase']=='applied' and receipt['result']['applied'] and receipt['result']['success'],receipt
        assert receipt['result']['enabled'] and receipt['result']['generation']==1,receipt
        code,replayed=http('POST',provider_path,retry_payload)
        assert code==200 and replayed==receipt,(code,replayed,receipt)
        code,_=http('POST',provider_path,{**retry_payload,'action':'stop'});assert code==409
        for invalid in ({'request_id':'provider-numeric-action','action':1},
                        {'request_id':'provider-negative-generation','generation':-1}):
            code,result=http('POST',provider_path,{**retry_payload,**invalid});assert code==400,(code,result)
        code,result=http('DELETE',provider_path,{**retry_payload,'request_id':'provider-wrong-method'})
        assert code==405,(code,result)
        state=providers['fs150'].read();assert state['enabled'] and state['generation']==1,state
        def sample(key):
            with state_lock:return latest.get(key)
        def step(n):
            r=mutation('/step',{'steps':n});return r['simulation_time_ns']
        def ros_graph():
            code,description,state=rpc.getSystemState('/xsim_private_validation')
            assert code==1,description
            checked=assert_retired_ros_surfaces_absent(state,providers,('scout','mecanum'))
            publishers={name:set(nodes) for name,nodes in state[0]}
            for kind in providers:
                for suffix in ('/pose','/twist'):
                    assert publishers.get('/'+kind+suffix)=={'/xsim'},publishers
            return checked
        stamp=step(10)
        for kind in providers:
            key=kind+('.pose' if kind=='fs150' else '.mocap')
            until(lambda: sample(key) and sample(key).header.stamp.to_nsec()==stamp,kind+' pose')
        until(lambda: sample('fs150.mocap') and sample('fs150.mocap').header.stamp.to_nsec()==stamp,'FS150 world measurement')
        p=sample('fs150.pose'); measurement=sample('fs150.mocap')
        assert abs(p.pose.position.x)<1e-12 and abs(p.pose.position.y)<1e-12
        assert abs(measurement.pose.position.x-2)<1e-12 and abs(measurement.pose.position.y+1)<1e-12
        assert measurement.header.frame_id=='world' and p.header.stamp.to_nsec()==epoch+10_000_000
        assert sample('fs150.imu').header.frame_id=='base_link'
        graph_report=ros_graph()
        assert p.header.frame_id=='map'
        until(lambda: sample('fs150.raw_imu'), 'raw IMU')
        assert sample('fs150.raw_imu').orientation_covariance[0]==-1
        for kind in ('fs150','scout','mecanum'):
            until(lambda: sample(kind+'.mocap_velocity'), kind+' mocap twist')
            assert sample(kind+'.mocap_velocity').header.frame_id=='world'
        rospy.wait_for_service('/fs150/mavros/cmd/arming',5)
        mode=rospy.ServiceProxy('/fs150/mavros/set_mode',SetMode)
        arm=rospy.ServiceProxy('/fs150/mavros/cmd/arming',CommandBool)
        forced=rospy.ServiceProxy('/fs150/mavros/cmd/command',CommandLong)
        assert mode(0,'OFFBOARD').mode_sent
        step(10);assert sample('fs150.state').mode=='POSCTL'
        assert mode(0,'UNSUPPORTED').mode_sent
        assert not mode(1,'POSCTL').mode_sent
        r=forced(False,400,0,1,21196,0,0,0,0,0);assert not r.success and r.result==3
        r=arm(True);assert r.success and r.result==0
        publisher=rospy.Publisher('/fs150/mavros/setpoint_raw/local',PositionTarget,queue_size=1)
        until(lambda: publisher.get_num_connections()>0,'setpoint ingress')
        def setpoint():
            p=PositionTarget();p.header.stamp=rospy.Time.now();p.coordinate_frame=1;p.type_mask=8|16|32|64|128|256|2048
            p.position.x=.5;p.position.y=.5;p.position.z=1;p.yaw=.6;publisher.publish(p);time.sleep(.015)
        setpoint();assert mode(0,'OFFBOARD').mode_sent
        for _ in range(30): setpoint();stamp=step(20)
        until(lambda: sample('fs150.mocap').header.stamp.to_nsec()==stamp,'flight measurement')
        flight=sample('fs150.mocap');assert flight.pose.position.z>.02
        assert sample('fs150.target').thrust>0
        r=arm(False);assert not r.success and r.result==2
        until(lambda: sample('fs150.cloud') and sample('fs150.cloud').width>0,'real CPU point cloud')
        cloud=sample('fs150.cloud');assert epoch<cloud.header.stamp.to_nsec()<=stamp
        fs_report={'nonzero_world_position':[measurement.pose.position.x,measurement.pose.position.y,measurement.pose.position.z],
                   'world_measurement_topic':'/fs150/pose','mocap_noise_std':[0,0,0],
                   'initial_local_position':[0,0,0], 'initial_stamp_ns':epoch+10_000_000,
                   'flight_position':[flight.pose.position.x,flight.pose.position.y,flight.pose.position.z],
                   'target_thrust':sample('fs150.target').thrust,'arm_ground_result':0,'airborne_disarm_result':r.result,
                   'mode_sent_without_stream':True,'mode_without_stream':'POSCTL','forced_arm_result':3,
                   'cloud_points':cloud.width,'cloud_stamp_ns':cloud.header.stamp.to_nsec()}
        fs_report['native_provider_boundaries']=['active_current_generation','active_previous_generation',
            'identical_request_replay','request_id_payload_conflict','numeric_action_rejected',
            'negative_generation_rejected','provider_wrong_method_rejected']
        # Paused provider/reset is an execution boundary, without any clock advance.
        frozen=get('/status')['simulation_time_ns'];r=providers['fs150'].stop(1);assert r['success'] and not r['enabled']
        until(lambda: sample('fs150.state') and not sample('fs150.state').connected, 'paused provider state transition')
        rospy.wait_for_service('/fs150/mavros/cmd/arming',5)
        offline=arm(True);assert not offline.success and offline.result==4
        r=providers['fs150'].start(1);assert r['success'] and r['generation']==2
        r=providers['fs150'].stop(1);assert not r['success'] and r['reason']==1 and r['generation']==2
        assert get('/status')['simulation_time_ns']==frozen
        # A future command from the old generation cannot survive provider reset.
        old=PositionTarget();old.header.stamp=rospy.Time.from_sec((frozen+200_000_000)/1e9);old.coordinate_frame=1;old.type_mask=3527;old.velocity.x=1
        publisher.publish(old);time.sleep(.03)
        r=providers['fs150'].stop(2);assert r['success']
        r=providers['fs150'].start(2);assert r['success'] and r['generation']==3
        step(300);assert not sample('fs150.state').armed
        fs_report['provider_reset_generation']=3;fs_report['stale_stop_reason']=1;fs_report['offline_arm_result']=4;fs_report['paused_state_transition']=True
        fs_report['future_old_generation_did_not_arm_or_move']=True
        # Ground ROS outputs against the trusted exact-baseline replay checkpoints.
        reports={'fs150':fs_report}
        pubs={k:rospy.Publisher('/'+k+'/cmd_vel',Twist,queue_size=1) for k in ('scout','mecanum')}
        for k,pub in pubs.items():until(lambda: pub.get_num_connections()>0,k+' velocity ingress')
        commands=[(1,0,0),(1,0,.5),(0,0,0),(-1,0,0),(0,1,0),(.7,-.5,-.4),(0,0,0),(0,0,0)]
        references={k:json.loads((Path(args.replays)/(k+'-replay.json')).read_text()) for k in pubs}
        max_error={k:0.0 for k in pubs};ground_samples={k:[] for k in pubs}
        for idx,u in enumerate(commands):
            for pub in pubs.values():
                m=Twist();m.linear.x,m.linear.y,m.angular.z=u;pub.publish(m)
            time.sleep(.03);stamp=step(1000)
            for k in pubs:
                until(lambda: all(sample(k+suffix) and sample(k+suffix).header.stamp.to_nsec()==stamp for suffix in ('.mocap','.mocap_velocity')),k+' matched measurement checkpoint')
                p=sample(k+'.mocap').pose.position;v=sample(k+'.mocap_velocity').twist.linear
                actual=[p.x,p.y,p.z,v.x,v.y,v.z]
                ref=references[k]['checkpoints'][idx];expected=ref['position']+ref['velocity']
                error=max(abs(a-b) for a,b in zip(actual,expected));max_error[k]=max(max_error[k],error)
                assert error<1e-9,(k,idx,error,actual,expected)
                assert sample(k+'.mocap').header.frame_id == sample(k+'.mocap_velocity').header.frame_id == 'world'
                ground_samples[k].append({'step':(idx+1)*1000,'stamp_ns':stamp,'position':actual[:3],'velocity':actual[3:]})
        for k in pubs:
            before=providers[k].read()['generation']
            r=mutation('/reset',{'entity_id':providers[k].entity_id,'generation':before})
            assert r['entity_id']==providers[k].entity_id and r['generation']==before+1 and r['enabled'],r
            after=providers[k].read()['generation'];assert after==before+1
            stale=providers[k].stop(before);assert not stale['success'] and stale['reason']==1
            stamp=step(20)
            until(lambda: sample(k+'.mocap').header.stamp.to_nsec()==stamp,k+' reset measurement')
            p=sample(k+'.mocap').pose.position;assert abs(p.x-2)<1e-12 and abs(p.y+1)<1e-12
            reports[k]={'max_abs_pose_velocity_error':max_error[k],'tolerance':1e-9,'checkpoints':ground_samples[k],
                        'provider_initial_generation':1,'reset_generation':after,'stale_provider_reason':stale['reason'],
                        'reset_position':[p.x,p.y,p.z],'world_frame':'world',
                        'measurement_topics':['/'+k+'/pose','/'+k+'/twist'],
                        'mocap_noise_std':[0,0,0]}
        graph_report=ros_graph()
        # Failed preparation must never commit an entity or silently select CPU.
        code,receipt=http('POST','/entities',{'instance_id':instance,'request_id':'unavailable-gpu','entity':{'name':'unavailable_gpu','kind':'scout','sensor':{'backend':'gpu'}}})
        assert code==202,receipt
        until(lambda: get('/requests/unavailable-gpu')['phase']=='failed','explicit unavailable GPU')
        failed=get('/requests/unavailable-gpu');assert not failed['result']['applied'] and 'no CPU fallback' in failed['error']
        assert not any(e['name']=='unavailable_gpu' for e in get('/entities')['entities'])
        # Management idempotency, instance fencing and unrelated slow HTTP clients.
        code,_=http('POST','/pause',{'instance_id':'wrong','request_id':'wrong'});assert code==409
        payload={'instance_id':instance,'request_id':'idempotent'}
        assert http('POST','/pause',payload)[0]==202
        until(lambda: get('/requests/idempotent')['phase']=='applied','idempotent applied')
        assert http('POST','/pause',payload)[0]==200
        slow=socket.socket(socket.AF_UNIX,socket.SOCK_STREAM);slow.connect(sock);slow.sendall(b'POST /pause HTTP/1.1\r\nContent-Length: 800\r\n\r\n{')
        prior=get('/status')['steps'];mutation('/resume');time.sleep(.25);assert get('/status')['steps']>prior;mutation('/pause');slow.close()
        for kind,report in reports.items():
            report.update(result='PASS',baseline='fed3fdc3d6625d9020d363bb5b9c3c8f8f2c72a6',
                scope='Private ROS direct canonical/MAVROS/UGV contract. Full supervised production Adapter lifecycle not executed.',
                direct_canonical=True, ros_graph=graph_report)
            (out/(kind+'-ros.json')).write_text(json.dumps(report,indent=2)+'\n')
        # One small expansion fixture, same code/config shape at 20 and 100.
        for e in get('/entities')['entities']:
            mutation('/entities/'+str(e['id']),{'generation':e['generation']},'DELETE')
        scales=[]
        for count in (20,100):
            for i in range(count):
                kind=('fs150','scout','mecanum')[i%3]
                spec={'name':'scale_'+str(i),'kind':kind,'position':[i*.1,0,0]}
                if i%5==0:spec['sensor']={'backend':'cpu','rate_hz':30,'h_res':720,'v_res':120,'h_fov_deg':180,'v_fov_deg':60}
                added=mutation('/entities',{'entity':spec})
                until(lambda: any(e['id']==added['entity_id'] for e in get('/entities')['entities']), 'added entity projection')
                r=NativeProvider(sock,'scale_'+str(i)).start(0);assert r['success']
            initial=get('/status');start=time.monotonic();mutation('/resume');time.sleep(2);mutation('/pause');elapsed=time.monotonic()-start;end=get('/status')
            proc={}
            for line in Path('/proc/'+str(server.pid)+'/status').read_text().splitlines():
                if line.startswith(('Threads:','VmRSS:','VmHWM:')):
                    key,val=line.split(':',1);proc[key]=val.strip()
            scale={'entities':count,'elapsed_wall_s':elapsed,'completed_steps':end['steps']-initial['steps'],
                   'interval_rtf':(end['simulation_time_ns']-initial['simulation_time_ns'])*1e-9/elapsed,'process':proc,'status':end}
            assert scale['completed_steps']>0
            scales.append(scale)
            for e in get('/entities')['entities']:mutation('/entities/'+str(e['id']),{'generation':e['generation']},'DELETE')
        assert scales[0]['process']['Threads']==scales[1]['process']['Threads'],scales
        (out/'scale.json').write_text(json.dumps(scales,indent=2)+'\n')
        os.killpg(server.pid,signal.SIGTERM);assert server.wait(timeout=10)==0
        assert not Path(sock).exists()
        slow_result=subprocess.run([args.slow_service_test],capture_output=True,text=True,timeout=15)
        (out/'slow-service.log').write_text(slow_result.stdout+slow_result.stderr)
        assert slow_result.returncode==0,slow_result.stdout+slow_result.stderr
        (out/'result.json').write_text(json.dumps({'result':'PASS','shutdown_exit':0,'socket_removed':True,'scales':[{'entities':x['entities'],'steps':x['completed_steps'],'rtf':x['interval_rtf'],'process':x['process']} for x in scales]},indent=2)+'\n')
        print((out/'result.json').read_text())
    finally:
        for p in reversed(children):
            if p.poll() is None:
                os.killpg(p.pid,signal.SIGTERM)
                try:p.wait(timeout=8)
                except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()
        for log in logs:log.close()

if __name__=='__main__':main()
