# Generates architecture.svg next to this script: python3 make_architecture_svg.py  (hand-placed coordinates; the SVG itself is the editable source - open in Inkscape / diagrams.net / a text editor).
W,H=1200,448
BLUE,BLUEF,INK='#1f4e8c','#eaf2fb','#0f2a4a'
GREY,GREYF='#5b6573','#eef0f3'
ARR='#334155'
o=[]
def rect(x,y,w,h,fill,stroke,sw=2,dash=None,rx=10):
    d=f' stroke-dasharray="{dash}"' if dash else ''
    o.append(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="{rx}" fill="{fill}" stroke="{stroke}" stroke-width="{sw}"{d}/>')
def text(x,y,s,size=13,weight=400,fill=INK,anchor='start'):
    o.append(f'<text x="{x}" y="{y}" font-size="{size}" font-weight="{weight}" fill="{fill}" text-anchor="{anchor}">{s}</text>')
def block(x,y,w,h,title,lines,fill=BLUEF,stroke=BLUE):
    rect(x,y,w,h,fill,stroke,2)
    text(x+12,y+26,title,15.5,700)
    for i,l in enumerate(lines): text(x+12,y+48+i*18,l,12.5,400,'#334155')
def lab(x,y,s,anchor='middle'):
    text(x,y,s,12,600,ARR,anchor)
def arrow(x1,y1,x2,y2,both=False):
    ms=' marker-start="url(#ah)"' if both else ''
    o.append(f'<line x1="{x1}" y1="{y1}" x2="{x2}" y2="{y2}" stroke="{ARR}" stroke-width="2.2" marker-end="url(#ah)"{ms}/>')
def path(pts):
    d='M'+' L'.join(f'{x},{y}' for x,y in pts)
    o.append(f'<path d="{d}" fill="none" stroke="{ARR}" stroke-width="2.2" stroke-linejoin="round" marker-end="url(#ah)"/>')
o.append(f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {W} {H}" width="{W}" height="{H}" font-family="DejaVu Sans, Verdana, Arial, sans-serif">')
o.append('<title>Warehouse AMR - System Architecture</title><desc>Mission layer, Nav2 behavior tree with Route Server and NavFn planning, MPPI controller, velocity output, ros2_control and Gazebo, with AMCL and EKF localization, occupancy map and route graph.</desc>')
o.append(f'<defs><marker id="ah" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" markerHeight="7" orient="auto-start-reverse"><path d="M0,0 L10,5 L0,10 z" fill="{ARR}"/></marker></defs>')
rect(0,0,W,H,'#ffffff','#ffffff',0,rx=0)
text(22,36,'Warehouse AMR — System Architecture',24,700)
text(22,58,'ROS 2 Jazzy · Nav2 · Gazebo Harmonic · simulation only',14,400,'#475569')
rect(12,74,1176,362,'#f6f9fd',BLUE,2.5,rx=14)
X=[22+245*i for i in range(5)]; BW=176
y1=92; h1=92; y2=250; mid=y1+h1/2
block(X[0],y1,BW,h1,'Mission layer',['GoToStation action','station → nav goal'])
block(X[1],y1,BW,h1,'Behavior tree',['coordinates planning,','sends path to MPPI'])
block(X[2],y1,BW,h1,'MPPI controller',['2D path, reverse ok','PreferForwardCritic'])
block(X[3],y1,BW,h1,'Velocity output',['velocity smoother','+ collision monitor'])
block(X[4],y1,BW,h1,'ros2_control',['diff-drive controller','wheel sep. × 0.92754'])
for i,l in enumerate(['goal','path','cmd_vel','cmd_vel']):
    arrow(X[i]+BW+3,mid,X[i+1]-3,mid); lab(X[i]+BW+34.5,mid-9,l)
# planning container
rect(X[1],y2-12,BW,176,'#ffffff',BLUE,1.6,'6 4')
text(X[1]+12,y2+4,'Planning',12.5,700,BLUE)
block(X[1]+8,y2+12,BW-16,58,'Route Server',['lane-based route'])
block(X[1]+8,y2+96,BW-16,58,'NavFn',['first / last-mile plans'])
arrow(X[1]+BW/2,y1+h1+3,X[1]+BW/2,y2-15,both=True); lab(X[1]+BW/2+10,y1+h1+36,'plan','start')
# data blocks (left)
block(X[0],y2+12,BW,58,'Route graph',['lane network'],GREYF,GREY)
block(X[0],y2+96,BW,58,'Occupancy map',['AMCL + costmaps'],GREYF,GREY)
arrow(X[0]+BW+3,y2+41,X[1]+5,y2+41); lab(X[0]+BW+36,y2+32,'lanes')
arrow(X[0]+BW+3,y2+125,X[1]+5,y2+125); lab(X[0]+BW+4,y2+116,'costmap','start')
path([(X[0]+BW/2,y2+157),(X[0]+BW/2,y2+176),(X[2]+36,y2+176),(X[2]+36,y2+107)]); lab(X[1]+BW+30,y2+170,'map')
# localization
block(X[2],y2,BW,104,'AMCL',['LiDAR + occupancy map','map → odom TF'])
block(X[3],y2,BW,104,'EKF',['wheel odom + IMU','odom → base TF'])
arrow(X[2]-3,y2+52,X[1]+BW+3,y2+52); lab(X[2]-34.5,y2+43,'pose')
arrow(X[3]-3,y2+52,X[2]+BW+3,y2+52); lab(X[3]-34.5,y2+43,'odom')
arrow(X[2]+40,y2-3,X[2]+40,y1+h1+3); lab(X[2]+50,y1+h1+36,'pose','start')
# EKF -> MPPI (odometry) elbow
path([(X[3]+28,y2-3),(X[3]+28,y1+h1+24),(X[2]+BW-28,y1+h1+24),(X[2]+BW-28,y1+h1+3)]); lab(X[2]+BW+34,y1+h1+18,'odom')
# Gazebo
block(X[4],y2,BW,104,'Gazebo',['robot, wheels, LiDAR','IMU, physics, world'],GREYF,GREY)
arrow(X[4]+BW-30,y1+h1+3,X[4]+BW-30,y2-3); lab(X[4]+BW-40,y1+h1+36,'wheel cmds','end')
arrow(X[4]-3,y2+52,X[3]+BW+3,y2+52); lab(X[4]-34.5,y2+43,'IMU')
# ros2_control wheel odometry -> EKF
path([(X[4]+30,y1+h1+3),(X[4]+30,y1+h1+40),(X[3]+BW-28,y1+h1+40),(X[3]+BW-28,y2-3)]); lab(X[4]-34.5,y1+h1+34,'wheel odom')
# LiDAR -> AMCL
path([(X[4]+BW/2,y2+107),(X[4]+BW/2,y2+138),(X[2]+BW/2,y2+138),(X[2]+BW/2,y2+107)]); lab((X[3]+BW/2+X[4]+BW/2)/2-30,y2+130,'LiDAR')
o.append('</svg>')
import os
open(os.path.join(os.path.dirname(os.path.abspath(__file__)),'architecture.svg'),'w').write('\n'.join(o))
