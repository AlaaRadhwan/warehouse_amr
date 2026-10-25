import sys;sys.path.insert(0,'/tmp/claude-1000/-home-alaa-radhwan-Projects-warehouse-amr/7ff4b44a-4e85-43ee-9032-a10b5a9916f5/scratchpad')
from lib import *
import model as M, numpy as np, math, matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
# recorded target sequence (odom frame) vs sim time in spin window
rows=[(stamp(r[2]),r[3][-1][2],len(r[3])) for r in D['tgp'] if 113<=r[1]<=196.3]
ts_=np.array([r[0] for r in rows]);tg_=np.array([r[1] for r in rows]);n_=np.array([r[2] for r in rows])
t0=ts_[0];T=ts_[-1]-t0
def lookup(arr):
    return lambda t: arr[min(np.searchsorted(ts_-t0,t),len(arr)-1)]
o=interp_odom(t0)
# real dy fit for true-goal-in-odom: from n==1 tgp
sel=n_==1; p=np.polyfit(ts_[sel]-t0,np.unwrap(tg_[sel]),1)
true_goal=lambda t: M.wrap(np.polyval(p,t))
# also: only-goal when n==1 else recorded cusp (this IS the recording); and 'cusp ignored' = true goal
res={}
print('replay from sim t0=%.1f for %.0fs; start yaw(odom) %.0f deg, rate -1.0'%(t0,T,math.degrees(o[2])))
for name,tgt in (('recorded tgp-end target',lookup(tg_)),('constant true goal (cusp ignored)',true_goal)):
    for seed in range(3):
        ts,ys,rs=M.run(seed=seed,T=min(T,50),yaw0=o[2],rate0=-1.0,seq0=-1.0,target=tgt)
        goal=np.array([tgt(t) for t in ts]);e=M.wrap(ys-goal)
        ok=(np.abs(e)<0.2)&(np.abs(rs)<0.1)
        print('%-38s seed%d  mean rate %+.2f  frac(rate<-0.5) %.2f  max rate %+.2f  checker-satisfied samples %.1f%%  first at %s  crossings %d'%(name,seed,rs.mean(),(rs<-0.5).mean(),rs.max(),100*ok.mean(),('%.1fs'%ts[np.argmax(ok)]) if ok.any() else 'never',np.sum((e[1:]*e[:-1]<0)&(np.abs(e[1:])<1))))
        res[(name,seed)]=(ts,ys,rs,goal)
# real stats for comparison
nav=[r for r in D['nav'] if 113<=r[1]<=196.3]
print('REAL cmd_vel_nav in window: mean rate %+.2f frac(<-0.5) %.2f max %+.2f'%(np.mean([r[3] for r in nav]),np.mean([r[3]<-0.5 for r in nav]),max(r[3] for r in nav)))
fig,ax=plt.subplots(2,1,figsize=(12,6),sharex=True)
for (name,seed),(ts,ys,rs,g) in res.items():
    if seed!=0: continue
    ax[0].plot(ts,np.degrees(M.wrap(ys-g)),lw=.8,label=name);ax[1].plot(ts,rs,lw=.8,label=name)
ax[0].set_ylabel('yaw err vs target [deg]');ax[1].set_ylabel('model wz [rad/s]');ax[1].set_xlabel('sim s from window start');ax[0].legend();plt.tight_layout();plt.savefig('debug/nav_test_b_to_a_bidirectional_analysis/fig3_reduced_model_replay.png',dpi=110)
