import math,pickle,rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
r=rosbag2_py.SequentialReader()
r.open(rosbag2_py.StorageOptions(uri='debug/nav_test_b_to_a_bidirectional/bag',storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
types={t.name:t.type for t in r.get_all_topics_and_types()}
print(types)
def yaw(q): return math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
D={k:[] for k in ['odom','nav','sm','cmd','plan','unsm','tgp','opt','tf_map_odom','tf_odom_base','fb','btlog','coll']}
clock=None;t0=None;W0=[None]
def path(m):
    return (m.header.frame_id+'|%.4f'%(m.header.stamp.sec+m.header.stamp.nanosec*1e-9),[(p.pose.position.x,p.pose.position.y,yaw(p.pose.orientation)) for p in m.poses])
cnt={}
while r.has_next():
    n,d,t=r.read_next()
    cnt[n]=cnt.get(n,0)+1
    if n=='/clock':
        m=deserialize_message(d,get_message(types[n]));clock=m.clock.sec+m.clock.nanosec*1e-9
        if t0 is None:t0=clock
        continue
    if clock is None: continue
    ts=clock-t0
    if W0[0] is None: W0[0]=t
    wt=(t-W0[0])*1e-9
    ts=(ts,wt)
    if n=='/odom':
        m=deserialize_message(d,get_message(types[n]));p=m.pose.pose
        D['odom'].append((ts[0],ts[1],p.position.x,p.position.y,yaw(p.orientation),m.twist.twist.linear.x,m.twist.twist.angular.z,m.header.frame_id,m.child_frame_id))
    elif n in('/cmd_vel_nav','/cmd_vel_smoothed','/cmd_vel'):
        m=deserialize_message(d,get_message(types[n]));s=m.twist
        D[{'/cmd_vel_nav':'nav','/cmd_vel_smoothed':'sm','/cmd_vel':'cmd'}[n]].append((ts[0],ts[1],s.linear.x,s.angular.z))
    elif n=='/plan': D['plan'].append((ts[0],ts[1],)+path(deserialize_message(d,get_message(types[n]))))
    elif n=='/unsmoothed_plan': D['unsm'].append((ts[0],ts[1],)+path(deserialize_message(d,get_message(types[n]))))
    elif n=='/transformed_global_plan': D['tgp'].append((ts[0],ts[1],)+path(deserialize_message(d,get_message(types[n]))))
    elif n=='/optimal_trajectory': D['opt'].append((ts[0],ts[1],)+path(deserialize_message(d,get_message(types[n]))))
    elif n=='/tf':
        m=deserialize_message(d,get_message(types[n]))
        for tr in m.transforms:
            a,b=tr.header.frame_id,tr.child_frame_id;x=tr.transform
            if (a,b)==('map','odom'):D['tf_map_odom'].append((ts[0],ts[1],x.translation.x,x.translation.y,yaw(x.rotation)))
            if (a,b)==('odom','base_footprint'):D['tf_odom_base'].append((ts[0],ts[1],x.translation.x,x.translation.y,yaw(x.rotation)))
    elif n=='/navigate_to_pose/_action/feedback':
        m=deserialize_message(d,get_message(types[n]));D['fb'].append((ts[0],ts[1],m.feedback.distance_remaining,m.feedback.number_of_recoveries,m.feedback.navigation_time.sec))
    elif n=='/collision_monitor_state':
        m=deserialize_message(d,get_message(types[n]));D['coll'].append((ts[0],ts[1],m.action_type,m.polygon_name))
print(cnt)
print({k:len(v) for k,v in D.items()}, 'sim duration',clock-t0)
pickle.dump(D,open('/tmp/claude-1000/-home-alaa-radhwan-Projects-warehouse-amr/7ff4b44a-4e85-43ee-9032-a10b5a9916f5/scratchpad/bag.pkl','wb'))
