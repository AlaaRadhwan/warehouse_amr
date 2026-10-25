import sys;sys.path.insert(0,'/tmp/claude-1000/-home-alaa-radhwan-Projects-warehouse-amr/7ff4b44a-4e85-43ee-9032-a10b5a9916f5/scratchpad')
from sweep import trial
print('--- az_max headroom (fixed goal yaw 0, start 80deg spinning) ---')
for az in (1.0,1.25,1.5):
    for g in (1.04,1.08,1.15): trial('az_max %.2f, gain %.2f'%(az,g),gain=g,az=az)
