"""Opt-in isolated wheel-rig measurement through the sole safety velocity gate."""
import argparse
import json
import math
import os
import hashlib
from pathlib import Path
import time


def analyze_trace(result):
    """Separate command latency from geometry using observed yaw excitation."""
    if not result.get('limits_at_start'):
        return
    cx,cy=result['limits_at_start']['rotation_estimate']['center_m']
    for trial in result['trials']:
        if 'requested' in trial:
            v,w=trial['requested'];a=trial['actual_yaw']
            predicted=[cx*(1-math.cos(a))+cy*math.sin(a)+v*math.sin(a)/w,
                       cy*(1-math.cos(a))-cx*math.sin(a)+v*(1-math.cos(a))/w]
            trial['prediction_using_observed_yaw']=predicted
            trial['geometry_residual_m']=math.dist(predicted,trial['actual_body_delta'])
            trial['prediction_note']='Observed yaw excitation; constant accepted v/omega ratio. Not open-loop timing proof.'
        else:
            start=trial['last_raw_sim_s']
            rows=[r for r in result['trace'] if start<=r['sim_s']<start+2.]
            nonzero=[r for r in rows if r['kind']=='cmd_vel' and any(r['value'])]
            zero=next((r for r in rows if r['kind']=='cmd_vel' and not any(r['value'])
                       and nonzero and r['sim_s']>nonzero[0]['sim_s']),None)
            odom=[r for r in rows if r['kind']=='odom']
            moving=[r for r in odom if abs(r['value'][3])>1e-4 or abs(r['value'][4])>1e-3]
            settled=next((r for r in odom if moving and r['sim_s']>moving[-1]['sim_s']),None)
            trial['final_command_zero_delay_s']=zero['sim_s']-start if zero else None
            trial['measured_settle_delay_s']=settled['sim_s']-start if settled else None
            trial['settle_speed_thresholds']={'linear_mps':1e-4,'angular_radps':1e-3}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--out', required=True)
    parser.add_argument('--release-for-trial', action='store_true')
    args = parser.parse_args()
    assert os.environ.get('ROS_DOMAIN_ID') == '227'
    assert os.environ.get('GZ_PARTITION') == 'pinky_calmap227'
    folder = Path('/tmp/pinky-calmap227')
    manifest = json.loads((folder/'run_manifest.json').read_text())
    assert manifest['run_id'] == 'c8cde754db8042a8aa56a0a649e729db'
    assert manifest['plant'] == 'wheel' and manifest['ros_domain'] == 227
    import rclpy
    from rclpy.node import Node
    from rclpy.parameter import Parameter
    from rclpy.qos import QoSProfile, DurabilityPolicy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String, Bool
    rclpy.init()
    node = Node('motion_contract_measurement', parameter_overrides=[Parameter('use_sim_time', value=True)])
    raw = node.create_publisher(Twist, '/cmd_vel_raw', 10)
    stop = node.create_publisher(String, '/estop/cmd', 10)
    data, received, trace = {}, {}, []
    def now():
        return node.get_clock().now().nanoseconds*1e-9
    def record(key, value):
        data[key], received[key] = value, time.monotonic()
        trace.append({'sim_s': now(), 'wall_s': time.monotonic(), 'kind':key, 'value':value})
    def odom(msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        record('odom', [msg.pose.pose.position.x, msg.pose.pose.position.y, yaw,
                        msg.twist.twist.linear.x, msg.twist.twist.angular.z])
    node.create_subscription(Odometry, '/odom', odom, 20)
    node.create_subscription(Twist, '/cmd_vel', lambda m:record('cmd_vel',[m.linear.x,m.angular.z]),20)
    for topic,key in [('/safety/motion_limits','limits'),('/calibration/status','calibration'),
                      ('/safety/decision','decision')]:
        node.create_subscription(String,topic,lambda m,k=key:record(k,json.loads(m.data)),20)
    node.create_subscription(String,'/wander/state',lambda m:record('wander',m.data),10)
    node.create_subscription(Bool,'/estop/state',lambda m:record('estop',m.data),
                             QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    def pump(seconds):
        deadline=time.monotonic()+seconds
        while time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.025)
    def send(v=0.,w=0.):
        msg=Twist();msg.linear.x=float(v);msg.angular.z=float(w)
        raw.publish(msg);record('cmd_vel_raw',[v,w])
    def safe(v=0.,w=0.,remaining=.003):
        if (any(time.monotonic()-received.get(k,-1e6)>.8 for k in ('limits','odom')) or
                time.monotonic()-received.get('calibration',-1e6)>3.):
            raise RuntimeError('stale telemetry')
        if data.get('estop') is not False or data.get('wander')!='stop' or not data['calibration'].get('ready'):
            raise RuntimeError('stop/calibration/controller prerequisite changed')
        lim=data['limits']
        if not lim.get('rotation_scan_observed'):
            raise RuntimeError('rotation scan incomplete')
        if w and not lim.get('can_rotate'):
            raise RuntimeError('rotation blocked')
        if v:
            cap=lim.get('rotation_translation_limits_m')
            if not cap or cap[0 if v>0 else 1] < remaining:
                raise RuntimeError('translation capsule insufficient')
    result={'manifest':manifest,'physical_robot_verified':False,'trials':[]}
    result['runtime_source_sha256']={str(p):hashlib.sha256(p.read_bytes()).hexdigest()
        for p in Path('/tmp/pinky-navigation-fix/move_control').rglob('*.py')}
    try:
        pump(3.)
        assert [p.node_name for p in node.get_publishers_info_by_topic('/cmd_vel')]==['safety_node']
        if args.release_for_trial:
            assert data.get('wander')=='stop' and data.get('calibration',{}).get('ready')
            stop.publish(String(data='release'));pump(.5)
        safe()
        result['calibration_at_start']=data['calibration']
        result['limits_at_start']=data['limits']
        # Restore durable measured pivot clearance, never beyond one 3 cm move.
        start=data['odom'][:]; t=now(); wall=time.monotonic()
        while data['limits'].get('rotation_pivot_clearance_m',0)<.020:
            distance=math.dist(start[:2],data['odom'][:2])
            if distance>=.03 or now()-t>=7 or time.monotonic()-wall>20:
                raise RuntimeError('Could not restore 20mm pivot clearance within relocation budget')
            safe(-.006,remaining=min(.003,.03-distance))
            send(-.006);pump(.05)
        send();pump(1.)
        for v,w in ((.006,.04),(.006,-.04),(-.006,.04),(-.006,-.04)):
            safe(v,w)
            begin=data['odom'][:];t=now();wall=time.monotonic()
            while now()-t<.3:
                if time.monotonic()-wall>5:raise RuntimeError('simulation clock stalled')
                safe(v,w);send(v,w);pump(.05)
            end=data['odom'][:];elapsed=now()-t
            send();pump(1.)
            cx,cy=data['limits']['rotation_estimate']['center_m']
            angle=end[2]-begin[2]; angle=math.atan2(math.sin(angle),math.cos(angle))
            dx,dy=end[0]-begin[0],end[1]-begin[1]
            c,s=math.cos(begin[2]),math.sin(begin[2])
            actual=[c*dx+s*dy,-s*dx+c*dy]
            # Constant-rate trial model; raw/final command trace identifies clipping.
            a=w*elapsed
            expected=[cx*(1-math.cos(a))+cy*math.sin(a)+v*math.sin(a)/w,
                      cy*(1-math.cos(a))-cx*math.sin(a)+v*(1-math.cos(a))/w]
            result['trials'].append({'requested':[v,w],'duration_s':elapsed,'start':begin,'end':end,
                                     'actual_body_delta':actual,'actual_yaw':angle,'predicted_body_delta':expected})
        for v,w in ((.014,0.),(0.,.1)):
            safe(v,w,remaining=.012)
            t=now();send(v,w);wall=time.monotonic()
            # Intentionally no further raw commands: measure safety watchdog.
            while now()-t<2.:
                safe(v,w,remaining=.003)
                if time.monotonic()-wall>8:raise RuntimeError('watchdog clock stalled')
                pump(.05)
            result['trials'].append({'watchdog_requested':[v,w],'last_raw_sim_s':t,'end':data['odom'][:]})
            send();pump(1.)
        result['status']='measured'
    except Exception as exc:
        result['status']='HOLD';result['reason']=str(exc)
    finally:
        for _ in range(10):send();pump(.05)
        stop.publish(String(data='stop'));pump(.5)
        result['limits_at_end']=data.get('limits');result['trace']=trace
        analyze_trace(result)
        out=Path(args.out);out.parent.mkdir(parents=True,exist_ok=True)
        out.write_text(json.dumps(result,indent=2))
        print(json.dumps({k:v for k,v in result.items() if k not in ('trace','manifest','calibration_at_start')}))
        node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
