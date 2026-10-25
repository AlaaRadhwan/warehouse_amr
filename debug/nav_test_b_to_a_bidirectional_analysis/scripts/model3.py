import sys;sys.path.insert(0,'/tmp/claude-1000/-home-alaa-radhwan-Projects-warehouse-amr/7ff4b44a-4e85-43ee-9032-a10b5a9916f5/scratchpad')
from lib import *
import model as M, numpy as np, math
SGF=np.array([-21,14,39,54,59,54,39,14,-21],float)/231
def sgfilt(seq,hist):
    n=len(seq)-1;out=seq.copy()
    pts=list(hist)+[seq[0],seq[1],seq[2],seq[3],seq[4]]  # m4..m1, 0..p4
    for idx in range(n):
        out[idx]=np.dot(SGF,pts[:9]);pts=pts[1:]+[seq[idx+5] if idx+5<n else seq[n]]
    return out
def run(seed=0,T=40.0,std=0.4,wzmax=1.0,az=1.0,temp=0.3,gamma=0.015,weight=3.0,B=2000,N=56,dt=0.05,yaw0=1.0,rate0=-1.0,target=lambda t:0.0,gain=1.0,use_sg=True):
    rng=np.random.default_rng(seed);yaw=yaw0;rate=rate0*gain;seq=np.full(N,rate0);hist=[rate0]*4
    ts=[];ys=[];rs=[];cs=[]
    for k in range(int(T/dt)):
        t=k*dt;goal=target(t)
        cw=seq+rng.normal(0,std,(B,N));wz=np.empty((B,N));wz[:,0]=rate;last=wz[:,0].copy();cwc=cw.copy()
        for j in range(1,N):
            c=np.clip(cw[:,j-1],last-az*dt,last+az*dt);cwc[:,j-1]=c;wz[:,j]=c;last=c
        yaws=yaw+np.cumsum(wz*dt,axis=1)
        cost=weight*np.mean(np.abs(M.wrap(yaws-goal)),axis=1)+gamma/std**2*np.sum(seq[None,:]*(cwc-seq[None,:]),axis=1)
        e=np.exp(-(cost-cost.min())/temp);sm=e/e.sum()
        seq=np.clip(np.sum(cwc*sm[:,None],axis=0),-wzmax,wzmax)
        last=seq[0]
        for i in range(1,N): seq[i]=np.clip(seq[i],last-az*dt,last+az*dt);last=seq[i]
        if use_sg:
            seq=sgfilt(seq,hist);hist=hist[1:]+[seq[1]]
        cmd=seq[1]
        seq=np.roll(seq,-1);seq[-1]=seq[-2]
        rate=cmd*gain;yaw=M.wrap(yaw+rate*dt)
        ts.append(t);ys.append(yaw);rs.append(rate);cs.append(cmd)
    return np.array(ts),np.array(ys),np.array(rs)
if __name__=='__main__':
    rows=[(stamp(r[2]),r[3][-1][2]) for r in D['tgp'] if 113<=r[1]<=196.3]
    tt=np.array([r[0] for r in rows]);tg=np.array([r[1] for r in rows]);t0=tt[0]
    rec=lambda t:tg[min(np.searchsorted(tt-t0,t),len(tg)-1)]
    o=interp_odom(t0)
    def rep(name,**kw):
        for seed in range(3):
            ts,ys,rs=run(seed=seed,**kw);g=np.array([kw.get('target',lambda t:0.0)(t) for t in ts]);e=M.wrap(ys-g)
            ok=(np.abs(e)<0.2)&(np.abs(rs)<0.1)
            print('%-52s s%d mean rate %+.2f frac(<-0.5) %.2f  checker-ok %.0f%%  crossings %d'%(name,seed,rs.mean(),(rs<-0.5).mean(),100*ok.mean(),np.sum((e[1:]*e[:-1]<0)&(np.abs(e[1:])<1))))
    print('REAL window: mean rate -0.96, frac(<-0.5) 0.96, checker-ok 0%')
    rep('SG + fixed goal yaw 0, start spinning',T=30)
    rep('SG + recorded tgp-end target, gain 1.0',T=50,yaw0=o[2],target=rec)
    rep('SG + recorded tgp-end target, gain 1.08',T=50,yaw0=o[2],target=rec,gain=1.08)
    rep('no-SG + recorded target, gain 1.08',T=50,yaw0=o[2],target=rec,gain=1.08,use_sg=False)
