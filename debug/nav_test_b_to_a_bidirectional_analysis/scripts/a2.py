import sys;sys.path.insert(0,'/tmp/claude-1000/-home-alaa-radhwan-Projects-warehouse-amr/7ff4b44a-4e85-43ee-9032-a10b5a9916f5/scratchpad')
from lib import *
import numpy as np, matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
WIN=(113.0,196.3)
# dy(t) (map->odom yaw) from tgp single-pose paths: goal in odom has yaw -dy
dyt=[];dyv=[]
for ts,wt,fr,P in D['tgp']:
    if len(P)==1: dyt.append(wt);dyv.append(-P[0][2])
dyt=np.array(dyt);dyv=np.array(dyv)
m=(dyt>=WIN[0])&(dyt<=WIN[1])
b,a=np.polyfit(dyt[m],np.unwrap(dyv[m]),1)
print('dy(map->odom yaw) from n=1 tgp: n=%d, linear fit dy = %.2f deg %+.3f deg/s * (t-113); resid std %.2f deg'%(m.sum(),deg(a+b*WIN[0]),deg(b),deg(np.std(np.unwrap(dyv[m])-(a+b*dyt[m])))))
dy=lambda t:a+b*t
# series
def arr(k,i): return np.array([r[i] for r in D[k]])
ow=arr('odom',1);oyaw=arr('odom',4);owz=arr('odom',6);ovx=arr('odom',5)
def wall_series(k): return arr(k,1),arr(k,2),arr(k,3)  # wall, vx, wz
nt,nvx,nwz=wall_series('nav');st_,svx,swz=wall_series('sm');ct,cvx,cwz=wall_series('cmd')
err_map=np.array([w(0-(y+dy(t))) for t,y in zip(ow,oyaw)])  # goal_yaw - robot_yaw (map)
# tgp-end error (what MPPI sees): last tgp pose yaw - robot yaw in odom, at tgp stamp
tgt=np.array([r[1] for r in D['tgp']]);tgn=np.array([len(r[3]) for r in D['tgp']])
tge=np.array([w(r[3][-1][2]-interp_odom(stamp(r[2]))[2]) for r in D['tgp']])
tgxy=np.array([math.hypot(r[3][-1][0]-r[3][0][0],r[3][-1][1]-r[3][0][1]) for r in D['tgp']])
fig,ax=plt.subplots(4,1,figsize=(13,12),sharex=True)
ax[0].plot(ow,np.degrees(err_map),lw=.8);ax[0].axhline(0,c='k',lw=.5);ax[0].axhspan(-11.46,11.46,color='g',alpha=.15,label='yaw tol 0.20 rad');ax[0].set_ylabel('goal yaw - robot yaw [deg]\n(map; wrapped)');ax[0].legend(loc='upper right')
ax[1].plot(nt,nwz,lw=.7,label='/cmd_vel_nav');ax[1].plot(st_,swz,lw=.7,label='/cmd_vel_smoothed');ax[1].plot(ct,cwz,lw=.7,label='/cmd_vel');ax[1].plot(ow,owz,lw=.7,label='odom twist');ax[1].axhline(0,c='k',lw=.5);ax[1].set_ylabel('angular z [rad/s]');ax[1].legend(ncol=4,loc='lower left')
ax[2].plot(tgt,np.degrees(tge),'.',ms=2);ax[2].axhline(0,c='k',lw=.5);ax[2].set_ylabel('tgp-end yaw - robot yaw [deg]\n(odom; what MPPI path/goal-angle sees)')
ax[3].plot(tgt,tgn,'.',ms=2,label='tgp length (poses)');ax[3].plot(tgt,tgxy*10,'.',ms=2,label='first->last tgp dist x10 [m]');ax[3].set_ylabel('transformed plan');ax[3].legend();ax[3].set_xlabel('wall time [s] since bag start')
for x in ax: x.set_xlim(95,200);x.axvline(113,c='r',lw=.5);x.axvline(196.3,c='r',lw=.5)
plt.tight_layout();plt.savefig('debug/nav_test_b_to_a_bidirectional_analysis/fig1_yaw_error_and_angular_velocity.png',dpi=110)
# stats
sel=lambda t:(t>=WIN[0])&(t<=WIN[1])
print('\nAngular z stats in spin window [wall %.0f-%.0f]:'%WIN)
for nm,t,wz in (('cmd_vel_nav',nt,nwz),('cmd_vel_smoothed',st_,swz),('cmd_vel',ct,cwz),('odom',ow,owz)):
    s=sel(t);x=wz[s];print('  %-17s n=%5d mean %+.3f  median %+.3f  min %+.3f  max %+.3f  frac(w<-0.5) %.2f  frac(w>0.1) %.3f'%(nm,s.sum(),x.mean(),np.median(x),x.min(),x.max(),(x<-0.5).mean(),(x>0.1).mean()))
s=sel(nt)
# error seen at nav tick
e_at=np.interp(nt[s],ow,err_map) if False else np.array([err_map[min(np.searchsorted(ow,t),len(ow)-1)] for t in nt[s]])
sg=np.sign(e_at)*np.sign(nwz[s])
print('  true map yaw error sign vs w sign: agree(rotating toward goal via shortest way) %.2f  disagree %.2f'%((sg>0).mean(),(sg<0).mean()))
near=np.abs(e_at)<np.radians(15)
print('  cycles with |yaw err|<15deg (near goal heading): %d  mean w there %+.3f  frac w<-0.5: %.2f'%(near.sum(),nwz[s][near].mean(),(nwz[s][near]<-0.5).mean()))
cross=[]
for i in range(1,len(ow)):
    if sel(np.array([ow[i]]))[0] and err_map[i-1]*err_map[i]<0 and abs(err_map[i])<1: cross.append(i)
print('  goal-yaw crossings in window: %d; odom wz at crossings: mean %+.3f min %+.3f max %+.3f; |odom v| there mean %.3f'%(len(cross),np.mean([owz[i] for i in cross]),min(owz[i] for i in cross),max(owz[i] for i in cross),np.mean([abs(ovx[i]) for i in cross])))
print('  first 8 crossings (wall t, wz):',[(round(ow[i],1),round(owz[i],2)) for i in cross[:8]])
print('\nTGP end (what MPPI targets) in spin window:  n>1 share %.2f'%((tgn[sel(tgt)]>1).mean()))
s2=sel(tgt)&(tgn>1); s1=sel(tgt)&(tgn==1)
print('  n>1: first->last dist mean %.2f m (max %.2f);  tgp-end yaw err vs robot: mean |err| %.0f deg'%(tgxy[s2].mean(),tgxy[s2].max(),np.degrees(np.abs(tge[s2])).mean()))
print('  n==1: mean |err| %.0f deg'%np.degrees(np.abs(tge[s1])).mean())
# does w sign follow sign of tgp-end error?
idx=[min(np.searchsorted(nt,t),len(nt)-1) for t in tgt[sel(tgt)]]
e_t=tge[sel(tgt)];wzs=nwz[idx]
for nm,mk in (('n>1',tgn[sel(tgt)]>1),('n==1',tgn[sel(tgt)]==1)):
    x=np.sign(e_t[mk])*np.sign(wzs[mk]);print('  w sign follows tgp-end yaw error sign (%s): agree %.2f disagree %.2f; mean w %+.3f'%(nm,(x>0).mean(),(x<0).mean(),wzs[mk].mean()))
