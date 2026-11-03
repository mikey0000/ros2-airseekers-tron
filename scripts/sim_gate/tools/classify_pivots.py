import json,sys,math,re,collections
run=sys.argv[1]
D=json.load(open(run+'/data.json'))
gt=D['gt']; hl=D['hl']
def state(t):
    s='?'
    for r in hl:
        if r[0]<=t: s=r[1]
        else: break
    return s
eps=[];cur=None
for a,b in zip(gt,gt[1:]):
    dt=min(0.1,b[0]-a[0]); v,w=abs(a[4]),abs(a[5])
    if v<0.02 and w>=0.1:
        if cur is None: cur=dict(t=a[0],t1=a[0],x=a[1],y=a[2],dur=0.0,turn=0.0,st=state(a[0]))
        cur['t1']=b[0]; cur['dur']+=dt; d=b[3]-a[3]; cur['turn']+=(d+math.pi)%(2*math.pi)-math.pi
    elif cur is not None and (a[0]-cur['t1'])>0.3:
        eps.append(cur);cur=None
if cur: eps.append(cur)
mov=sum(min(0.1,b[0]-a[0]) for a,b in zip(gt,gt[1:]) if abs(a[4])>=0.02 or abs(a[5])>=0.1)
ev=[]
for line in open(run+'/mission.log',errors='ignore'):
    m=re.search(r'(pivot vertex|leg start|K-turn about-turn|route corner) at \((-?[\d.]+), (-?[\d.]+)\)',line)
    if m:
        kind=m.group(1); x,y=float(m.group(2)),float(m.group(3))
        g=re.search(r'pivot (-?\d+) deg',line); ang=abs(int(g.group(1))) if g else 0
        sub=re.search(r'sub-path (\d+)',line)
        ev.append((kind,x,y,ang,int(sub.group(1)) if sub else -1))
cat=collections.OrderedDict()
def add(k,e):
    c=cat.setdefault(k,[0,0.0,0.0]); c[0]+=1; c[1]+=e['dur']; c[2]+=abs(math.degrees(e['turn']))
for e in eps:
    best=None
    for k in ev:
        d=math.hypot(k[1]-e['x'],k[2]-e['y'])
        if d<0.35 and (best is None or d<best[0]): best=(d,k)
    if e['st'] in ('TRANSIT','RETURNING_HOME','UNDOCKING') or (best and best[1][0]=='route corner'):
        k='route/transit corner'
    elif best is None: k='unmatched (shaper pivot-instead / approach)'
    elif best[1][0]=='leg start': k='leg start (after transit)'
    elif best[1][0]=='K-turn about-turn': k='K-turn about-turn'
    elif best[1][3]>=100: k='row-end pivot vertex (>=100 deg)'
    else: k='lap/edge corner pivot vertex (<100 deg)'
    add(k,e); e['k']=k
print('moving %.0f s, pivot %.0f s (%.0f %%), %d episodes'%(mov,sum(e['dur'] for e in eps),100*sum(e['dur'] for e in eps)/mov,len(eps)))
for k,(n,s,deg) in cat.items(): print('  %-44s n %2d  %5.1f s  %4.1f %% of moving  %5.0f deg  (%.1f deg/s)'%(k,n,s,100*s/mov,deg,deg/s if s else 0))
if len(sys.argv)>2:
    for e in eps: print('%7.1f %-15s (%.2f,%.2f) %.1f s %+.0f  %s'%(e['t']-gt[0][0],e['st'],e['x'],e['y'],e['dur'],math.degrees(e['turn']),e['k']))
