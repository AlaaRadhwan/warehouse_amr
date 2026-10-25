import sys;sys.path.insert(0,'/tmp/claude-1000/-home-alaa-radhwan-Projects-warehouse-amr/7ff4b44a-4e85-43ee-9032-a10b5a9916f5/scratchpad')
import model3 as M3, model as M, numpy as np, math
def trial(name,seeds=(0,1),T=30,**kw):
    out=[]
    for s in seeds:
        ts,ys,rs=M3.run(seed=s,T=T,yaw0=math.radians(80),**kw);e=M.wrap(ys);ok=(np.abs(e)<0.2)&(np.abs(rs)<0.1)
        out.append((rs[-200:].mean(),ok.mean()*100,(np.abs(rs)<0.1).any() and ts[np.argmax(ok)] if ok.any() else -1))
    print('%-58s end-rate %s  checker-ok%% %s  first-ok(s) %s'%(name,['%+.2f'%o[0] for o in out],['%.0f'%o[1] for o in out],['%.1f'%o[2] if o[2]>=0 else 'never' for o in out]))
print('--- fixed goal yaw 0, start at 80deg spinning -1.0; plant gain = measured/commanded ---')
for g in (1.0,1.01,1.02,1.04,1.08,1.15): trial('gain %.2f (wz_max 1.0)'%g,gain=g)
print('--- gain 1.08, vary wz_max ---')
for L in (0.6,0.8,1.0,1.2): trial('wz_max %.1f, gain 1.08'%L,gain=1.08,wzmax=L,rate0=-L)
print('--- gain 1.08, other MPPI knobs ---')
trial('az_max 2.0',gain=1.08,az=2.0); trial('wz_std 0.8',gain=1.08,std=0.8); trial('temperature 1.0',gain=1.08,temp=1.0)
