"""ROS-independent candidate association and exact team IDL conversion."""
import math
import re
import uuid

COLORS = {'unknown': 0, 'red': 1, 'blue': 2, 'yellow': 3, 'green': 4}

class Tracker:
    """Bounded pixel association, not physical identity under camera motion."""
    def __init__(self, max_distance=80., max_gap_s=1., confirmation_hits=2,
                 ambiguity_px=8., prefix=None, allow_unknown=False):
        self.distance=max_distance; self.gap=max_gap_s; self.hits=confirmation_hits
        self.margin=ambiguity_px; self.prefix=prefix or uuid.uuid4().hex[:12]
        self.tracks={}; self.sequence=0; self.last=None; self.allow_unknown=allow_unknown

    def update(self, candidates, timestamp):
        if not math.isfinite(timestamp) or (self.last is not None and timestamp <= self.last):
            raise ValueError('timestamp must be finite and strictly increasing')
        self.last=timestamp
        lost=[]
        for key,t in list(self.tracks.items()):
            if timestamp-t['last_seen'] > self.gap:
                lost.append(dict(t, tracking_status='lost'))
                del self.tracks[key]
        candidates=[dict(c) for c in candidates if c.get('detection_valid',True)
                    and c['color'] in COLORS and (c['color']!='unknown' or self.allow_unknown)
                    and math.isfinite(c['confidence']) and 0.2 <= c['confidence'] <= 1
                    and len(c['pixel_center'])==2
                    and all(math.isfinite(x) for x in c['pixel_center'])]
        edges={}
        for i,c in enumerate(candidates):
            edges[i]=sorted((math.dist(c['pixel_center'],t['pixel_center']),key)
                            for key,t in self.tracks.items()
                            if math.dist(c['pixel_center'],t['pixel_center']) <= self.distance)
        ambiguous=set()
        for i,e in edges.items():
            if len(e)>1 and e[1][0]-e[0][0] < self.margin: ambiguous.add(i)
        for key in self.tracks:
            contenders=sorted((d,i) for i,e in edges.items() for d,k in e if k==key)
            if len(contenders)>1 and contenders[1][0]-contenders[0][0]<self.margin:
                ambiguous.update(i for d,i in contenders if d-contenders[0][0]<self.margin)
        used=set(); assignments={}
        for d,i,key in sorted((d,i,key) for i,e in edges.items() for d,key in e):
            if i not in ambiguous and i not in assignments and key not in used:
                assignments[i]=key; used.add(key)
        observed=[]
        for i,c in enumerate(candidates):
            if i in ambiguous: continue
            key=assignments.get(i)
            # An unmatched candidate near an existing track is ambiguous, not a new battery.
            if key is None and edges[i]: continue
            if key is None:
                self.sequence+=1; key=f'{self.prefix}-{self.sequence:06d}'
                self.tracks[key]={'object_id':key,'color':c['color'],'hits':0,'votes':{}}
            t=self.tracks[key]
            t['votes'][c['color']]=t['votes'].get(c['color'],0)+1
            t['hits']+=1
            stable=max(t['votes'],key=t['votes'].get)
            status='confirmed' if t['hits']>=self.hits else 'tentative'
            if c['color']!=stable: status='color_uncertain'
            t.update(c,object_id=key,color=stable,last_seen=timestamp,timestamp=timestamp,
                     tracking_status=status)
            observed.append(dict(t))
        observed_ids={t['object_id'] for t in observed}
        missed=[dict(t,tracking_status='missed') for k,t in self.tracks.items()
                if k not in observed_ids]
        return {'observed':observed,'missed':missed,'lost':lost,
                'ambiguous_candidate_indices':sorted(ambiguous)}

def header(frame_id,timestamp):
    if not isinstance(frame_id,str) or not re.fullmatch(r'[A-Za-z][A-Za-z0-9_/]*',frame_id):
        raise ValueError('explicit valid frame_id required')
    if not math.isfinite(timestamp) or timestamp<=0: raise ValueError('positive capture timestamp required')
    ns=round(timestamp*1e9)
    return {'frame_id':frame_id,'stamp':{'sec':ns//1000000000,'nanosec':ns%1000000000}}

def make_array(tracking,frame_id,timestamp):
    h=header(frame_id,timestamp); detections=[]; seen=set()
    for t in tracking['observed']:
        if t['tracking_status']!='confirmed': continue
        if not t['object_id'] or t['object_id'] in seen: raise ValueError('duplicate/empty ID')
        seen.add(t['object_id'])
        # Strict JSON null becomes IEEE NaN in native ROS. Never a zero pose.
        invalid={'header':h,'pose':{'position':dict.fromkeys(('x','y','z')),
                                  'orientation':dict.fromkeys(('x','y','z','w'))}}
        detections.append({'object_id':t['object_id'],'color':COLORS[t['color']],
                           'body_pose':invalid,'ring_pose':invalid,
                           'confidence':float(t['confidence']),'ring_pose_valid':False})
    return {'header':h,'detections':detections}

def motion_rejection(message,now_s,max_age_s=.5,mode='replay'):
    ids=[d['object_id'] for d in message['detections']]
    if len(ids)!=len(set(ids)): return 'DUPLICATE_ID'
    if mode!='live': return 'REPLAY_NOT_MOTION_INPUT'
    stamp=message['header']['stamp']; age=now_s-stamp['sec']-stamp['nanosec']*1e-9
    if not math.isfinite(age) or age<0 or age>max_age_s: return 'STALE_OR_FUTURE'
    for d in message['detections']:
        if not d['ring_pose_valid']: return 'RING_INVALID'
        for name in ('body_pose','ring_pose'):
            p=d[name]
            if p['header']!=message['header']: return 'POSE_HEADER_MISMATCH'
            values=list(p['pose']['position'].values())+list(p['pose']['orientation'].values())
            if any(v is None or not math.isfinite(v) for v in values): return 'POSE_INVALID'
            q=list(p['pose']['orientation'].values())
            if abs(sum(x*x for x in q)-1)>1e-3: return 'QUATERNION_INVALID'
    return 'EMPTY' if not ids else 'NO_REJECTION'

def to_ros(data,array_type,detection_type):
    """Dependency injection supports tests; production uses generated mtc_interfaces."""
    result=array_type()
    def set_header(target,h):
        target.frame_id=h['frame_id']
        target.stamp.sec=h['stamp']['sec']; target.stamp.nanosec=h['stamp']['nanosec']
    set_header(result.header,data['header'])
    ids=set()
    for value in data['detections']:
        if value['object_id'] in ids: raise ValueError('duplicate ID')
        ids.add(value['object_id'])
        d=detection_type()
        d.object_id=value['object_id']; d.color=value['color']
        d.confidence=value['confidence']; d.ring_pose_valid=False
        for name in ('body_pose','ring_pose'):
            target=getattr(d,name); set_header(target.header,value[name]['header'])
            for group in ('position','orientation'):
                for key,v in value[name]['pose'][group].items():
                    setattr(getattr(target.pose,group),key,float('nan') if v is None else float(v))
        result.detections.append(d)
    return result
