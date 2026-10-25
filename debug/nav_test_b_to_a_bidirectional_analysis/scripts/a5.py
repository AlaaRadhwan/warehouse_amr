import sys;sys.path.insert(0,'/tmp/claude-1000/-home-alaa-radhwan-Projects-warehouse-amr/7ff4b44a-4e85-43ee-9032-a10b5a9916f5/scratchpad')
import rosbag2_py,numpy as np,pickle
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
r=rosbag2_py.SequentialReader();r.open(rosbag2_py.StorageOptions(uri='debug/nav_test_b_to_a_bidirectional/bag',storage_id='mcap'),rosbag2_py.ConverterOptions('',''))
ty={t.name:t.type for t in r.get_all_topics_and_types()}
imu=[];dd=[];fused=[];W0=None
while r.has_next():
    n,d,t=r.read_next()
    if W0 is None: W0=t
    if n not in('/imu/data_raw','/diff_drive_controller/odom','/odom'): continue
    m=deserialize_message(d,get_message(ty[n]));wt=(t-W0)*1e-9
    if n=='/imu/data_raw': imu.append((wt,m.angular_velocity.z))
    elif n=='/odom': fused.append((wt,m.twist.twist.linear.x,m.twist.twist.angular.z))
    else: dd.append((wt,m.twist.twist.linear.x,m.twist.twist.angular.z))
pickle.dump((imu,dd,fused),open('/tmp/claude-1000/-home-alaa-radhwan-Projects-warehouse-amr/7ff4b44a-4e85-43ee-9032-a10b5a9916f5/scratchpad/imu_dd.pkl','wb'))
imu=np.array(imu);dd=np.array(dd);fused=np.array(fused)
win=lambda a:a[(a[:,0]>=125)&(a[:,0]<=190)]
print('spin window wall 125-190: mean angular z')
print('  IMU gyro z                         %+.4f'%win(imu)[:,1].mean())
print('  /diff_drive_controller/odom (wheels) %+.4f'%win(dd)[:,2].mean())
print('  /odom (fused EKF)                    %+.4f'%win(fused)[:,2].mean())
