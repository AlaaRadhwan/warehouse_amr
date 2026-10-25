import pickle,math,bisect
D=pickle.load(open('/tmp/claude-1000/-home-alaa-radhwan-Projects-warehouse-amr/7ff4b44a-4e85-43ee-9032-a10b5a9916f5/scratchpad/bag.pkl','rb'))
w=lambda a:(a+math.pi)%(2*math.pi)-math.pi
deg=math.degrees
SIM0=D['plan'][0][2].split('|')[1]; SIM0=float(SIM0)-D['plan'][0][0]
def stamp(fr): return float(fr.split('|')[1])-SIM0
def frame(fr): return fr.split('|')[0]
od=D['odom']; odt=[o[0] for o in od]
def interp_odom(t):
    i=bisect.bisect_left(odt,t)
    if i<=0: return od[0][2:5]
    if i>=len(od): return od[-1][2:5]
    a,b=od[i-1],od[i];f=(t-a[0])/(b[0]-a[0]) if b[0]>a[0] else 0
    return (a[2]+f*(b[2]-a[2]),a[3]+f*(b[3]-a[3]),a[4]+f*w(b[4]-a[4]))
def sim2wall(t):
    # piecewise from odom records
    i=min(max(bisect.bisect_left(odt,t),0),len(od)-1);return od[i][1]
def wall2sim(wt):
    wts=[o[1] for o in od];i=min(max(bisect.bisect_left(wts,wt),0),len(od)-1);return od[i][0]
def T_from_plan(p):
    ts,wt,fr,P=p
    if len(P)<2: return None
    st=stamp(fr);o=interp_odom(st)
    dy=w(P[0][2]-o[2]);c,s=math.cos(dy),math.sin(dy)
    return (st,wt,P[0][0]-(c*o[0]-s*o[1]),P[0][1]-(s*o[0]+c*o[1]),dy)
Ts=[t for t in map(T_from_plan,D['plan']) if t]
Tst=[t[0] for t in Ts]
def T_at(st):
    i=min(max(bisect.bisect_left(Tst,st),0),len(Ts)-1);return Ts[i]
def to_map(x,y,yaw,T): c,s=math.cos(T[4]),math.sin(T[4]);return (T[2]+c*x-s*y,T[3]+s*x+c*y,w(yaw+T[4]))
