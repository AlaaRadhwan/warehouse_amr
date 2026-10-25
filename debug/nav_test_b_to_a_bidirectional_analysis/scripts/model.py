import numpy as np,math,sys
def wrap(a): return (a+np.pi)%(2*np.pi)-np.pi
def run(seed=0,T=20.0,std=0.4,wzmax=1.0,az=1.0,temp=0.3,gamma=0.015,weight=3.0,B=2000,N=56,dt=0.05,
        yaw0=math.radians(60),rate0=-1.0,seq0=-1.0,target=lambda t:0.0,plant_gain=1.0,sg=False,shift=True):
    rng=np.random.default_rng(seed)
    yaw=yaw0;rate=rate0*plant_gain;seq=np.full(N,seq0);
    ts=[];ys=[];rs=[]
    for k in range(int(T/dt)):
        t=k*dt;goal=target(t)
        noise=rng.normal(0,std,(B,N));cw=seq+noise
        # predict: clamp per-step delta from measured rate (state.wz[:,0])
        wz=np.empty((B,N));wz[:,0]=rate;last=wz[:,0].copy();cwc=cw.copy()
        for j in range(1,N):
            c=np.clip(cw[:,j-1],last-az*dt,last+az*dt);cwc[:,j-1]=c;wz[:,j]=c;last=c
        yaws=yaw+np.cumsum(wz*dt,axis=1)
        cost=weight*np.mean(np.abs(wrap(yaws-goal)),axis=1)
        cost+=gamma/std**2*np.sum(seq[None,:]*(cwc-seq[None,:]),axis=1)
        cn=cost-cost.min();e=np.exp(-cn/temp);sm=e/e.sum()
        seq=np.sum(cwc*sm[:,None],axis=0)
        seq=np.clip(seq,-wzmax,wzmax)
        last=seq[0]
        for i in range(1,N):
            seq[i]=np.clip(seq[i],last-az*dt,last+az*dt);last=seq[i]
        cmd=seq[0]
        if shift: seq=np.roll(seq,-1);seq[-1]=seq[-2]
        rate=cmd*plant_gain;yaw=wrap(yaw+rate*dt)
        ts.append(t);ys.append(yaw);rs.append(rate)
    return np.array(ts),np.array(ys),np.array(rs)
def summarize(name,ts,ys,rs,goal=0.0,tol=0.2,vtol=0.1):
    e=wrap(ys-goal)
    ok=(np.abs(e)<tol)&(np.abs(rs)<vtol)
    cross=np.sum((e[1:]*e[:-1]<0)&(np.abs(e[1:])<1))
    first=ts[np.argmax(ok)] if ok.any() else None
    print('%-46s final |err| %5.1f deg  final rate %+.2f  goal-reached-by-checker at %s  yaw crossings %d  mean rate last 5s %+.2f'%(name,math.degrees(abs(e[-1])),rs[-1],('%.1fs'%first) if first is not None else 'never',cross,rs[-100:].mean()))
if __name__=='__main__':
    for seed in range(3):
        summarize('A: n==1 goal yaw 0, spinning -1.0, seed%d'%seed,*run(seed=seed))
    print()
    for seed in range(3):
        summarize('A2: same but start at rest, 60deg off, s%d'%seed,*run(seed=seed,rate0=0.0,seq0=0.0))
