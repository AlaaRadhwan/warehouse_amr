import pickle,math,bisect
D=pickle.load(open('/tmp/claude-1000/-home-alaa-radhwan-Projects-warehouse-amr/7ff4b44a-4e85-43ee-9032-a10b5a9916f5/scratchpad/bag.pkl','rb'))
w=lambda a:(a+math.pi)%(2*math.pi)-math.pi
deg=math.degrees
odt=[o[0] for o in D['odom']]
def odom_at(t):
    i=min(max(bisect.bisect_left(odt,t),0),len(odt)-1);return D['odom'][i]
# map->odom from plan start pose (only multi-pose plans: first pose = robot)
rows=[]
for p in D['plan']:
    ts,wt,fr,P=p
    if len(P)<2: continue
    o=odom_at(ts)
    dy=w(P[0][2]-o[4]);c,s=math.cos(dy),math.sin(dy)
    tx=P[0][0]-(c*o[2]-s*o[3]);ty=P[0][1]-(s*o[2]+c*o[3])
    rows.append((wt,tx,ty,deg(dy)))
print('map->odom estimates from plan[0] (wall t, tx,ty,yaw deg): n=',len(rows))
for r in rows[::12]+[rows[-1]]: print('  t=%.0f tx=%.3f ty=%.3f yaw=%.1f'%r)
late=[r for r in rows if r[0]>100]
import statistics as st
print('late (t>100): tx %.3f±%.3f ty %.3f±%.3f yaw %.1f±%.1f'%(st.mean(r[1] for r in late),st.pstdev(r[1] for r in late),st.mean(r[2] for r in late),st.pstdev(r[2] for r in late),st.mean(r[3] for r in late),st.pstdev(r[3] for r in late)))
pickle.dump(rows,open('/tmp/claude-1000/-home-alaa-radhwan-Projects-warehouse-amr/7ff4b44a-4e85-43ee-9032-a10b5a9916f5/scratchpad/m2o.pkl','wb'))
