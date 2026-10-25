import sys;sys.path.insert(0,'/tmp/claude-1000/-home-alaa-radhwan-Projects-warehouse-amr/7ff4b44a-4e85-43ee-9032-a10b5a9916f5/scratchpad')
from lib import *
import numpy as np, matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt, bisect
WIN=(113.0,196.3)
nav=D['nav'];navt=[r[1] for r in nav]
rows=[]
for o,g in zip(D['opt'],D['tgp']):
    ts,wt,fr,P=o
    if not(WIN[0]<=wt<=WIN[1]): continue
    st=stamp(fr);ro=interp_odom(st)
    yaws=np.unwrap([p[2] for p in P]);n=len(P)
    tg=g[3][-1];tgyaw=tg[2]
    i=min(bisect.bisect_left(navt,wt),len(nav)-1);wz=nav[i][3]
    pred_w=(yaws[-1]-yaws[0])/((n-1)*0.05) if n>1 else 0
    # horizon 56 poses; time to cross tgp-end yaw within trajectory
    e0=w(tgyaw-ro[2]);eN=w(tgyaw-yaws[-1])
    total_rot=yaws[-1]-ro[2] if False else yaws[-1]-yaws[0]
    # overshoot: does trajectory pass the target yaw (unwrapped sign change of e along traj)?
    e=np.array([w(tgyaw-y) for y in yaws]);cross=np.sum(np.sign(e[1:])*np.sign(e[:-1])<0 if False else (np.abs(np.diff(np.unwrap(e)))>0) & False)
    ee=np.array([abs(w(tgyaw-y)) for y in yaws])
    crossed=np.any((np.sign(np.array([w(tgyaw-y) for y in yaws]))[1:]!=np.sign(np.array([w(tgyaw-y) for y in yaws]))[:-1])&(ee[1:]<1.0))
    # true goal heading in odom
    q=np.diff(yaws)/0.05;rows.append(dict(w_first=q[:5].mean(),w_mid=q[25:30].mean(),w_last=q[-5:].mean(),wt=wt,n=n,gn=len(g[3]),robot=ro[2],tgyaw=tgyaw,pfin=yaws[-1],pw=pred_w,wz=wz,e0=e0,eN=eN,mind=ee.min(),crossed=crossed,tot=total_rot,dist=math.hypot(P[-1][0]-P[0][0],P[-1][1]-P[0][1])))
import statistics as s
print('optimal_trajectory cycles in spin window:',len(rows),' poses/traj:',rows[0]['n'])
print('predicted mean angular rate over horizon: mean %.3f  min %.3f  max %.3f rad/s ; commanded wz mean %.3f'%(s.mean(r['pw'] for r in rows),min(r['pw'] for r in rows),max(r['pw'] for r in rows),s.mean(r['wz'] for r in rows)))
print('predicted total rotation over 2.8 s horizon: mean %.0f deg (full lap = 360)'%deg(s.mean(r['tot'] for r in rows)))
print('predicted xy travel over horizon: mean %.3f m max %.3f m'%(s.mean(r['dist'] for r in rows),max(r['dist'] for r in rows)))
for nm,sel in (('all',lambda r:True),('tgp n==1',lambda r:r['gn']==1),('tgp n>1',lambda r:r['gn']>1)):
    R=[r for r in rows if sel(r)]
    if not R: continue
    print('%-9s n=%4d |e0|(now) mean %3.0f deg -> |eN|(end of horizon) mean %3.0f deg; improves in %.0f%% ; traj passes within <1rad and crosses target yaw: %.0f%% ; min |e| along traj mean %.0f deg'%(nm,len(R),deg(s.mean(abs(r['e0']) for r in R)),deg(s.mean(abs(r['eN']) for r in R)),100*s.mean(float(abs(r['eN'])<abs(r['e0'])) for r in R),100*s.mean(float(r['crossed']) for r in R),deg(s.mean(r['mind'] for r in R))))
print('\nrepresentative cycles (wall, tgp_n, robot yaw, tgp-end yaw, pred final yaw, pred mean w, cmd w, |e0|->|eN| deg):')
for r in rows[::int(len(rows)/16)]:
    print('  %6.1f n=%d robot=%7.1f tgp_end=%7.1f pred_final=%7.1f pred_w=%+.2f cmd_w=%+.2f  e0=%6.1f -> eN=%6.1f'%(r['wt'],r['gn'],deg(r['robot']),deg(r['tgyaw']),deg(w(r['pfin'])),r['pw'],r['wz'],deg(r['e0']),deg(r['eN'])))
fig,ax=plt.subplots(2,1,figsize=(13,7),sharex=True)
t=[r['wt'] for r in rows]
ax[0].plot(t,[deg(r['robot']) for r in rows],'.',ms=1.5,label='robot yaw (odom)');ax[0].plot(t,[deg(r['tgyaw']) for r in rows],'.',ms=1.5,label='tgp-end yaw (odom)');ax[0].plot(t,[deg(w(r['pfin'])) for r in rows],'.',ms=1.5,label='predicted final yaw');ax[0].legend(ncol=3);ax[0].set_ylabel('yaw [deg]')
ax[1].plot(t,[r['pw'] for r in rows],'.',ms=1.5,label='predicted mean w over horizon');ax[1].plot(t,[r['wz'] for r in rows],'.',ms=1.5,label='cmd_vel_nav w');ax[1].legend();ax[1].set_ylabel('rad/s');ax[1].set_xlabel('wall time [s]')
plt.tight_layout();plt.savefig('debug/nav_test_b_to_a_bidirectional_analysis/fig2_optimal_trajectory_vs_yaw.png',dpi=110)
