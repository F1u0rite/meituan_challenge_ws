"""Temporal admission for native RGB-D; never re-pair or rewrite timestamps."""
from collections import Counter
import math


class RGBDPairGuard:
    def __init__(self, max_skew_ms=100.):
        self.max_skew_ms=max_skew_ms
        self.last_accepted={};self.last_observed={}
        self.accepted=0;self.rejected=0;self.reason_counts=Counter()
        self.gaps=Counter();self.observed_repeats=Counter();self.last_report=None

    def inspect(self, timestamps):
        reasons=[];delta=None
        domains=[str(timestamps.get(s+'_timestamp_domain','')).removeprefix('timestamp_domain.') for s in ('color','depth')]
        if domains[0]!=domains[1] or domains[0] not in ('global_time','system_time'):
            reasons.append('TIME_DOMAIN_NOT_COMPARABLE')
        ids={};sdk=[]
        for stream in ('color','depth'):
            ident=timestamps.get(stream+'_frame_number');stamp=timestamps.get(stream+'_sdk_ms')
            if type(ident) is not int or ident<0:reasons.append('INVALID_'+stream.upper()+'_FRAME_ID')
            else:
                ids[stream]=ident
                previous=self.last_observed.get(stream)
                if previous is not None:
                    if ident==previous:self.observed_repeats[stream]+=1
                    elif ident>previous+1:self.gaps[stream]+=ident-previous-1
                    elif ident<previous:reasons.append('FRAME_ID_BACKWARDS')
                self.last_observed[stream]=ident
                if ident<=self.last_accepted.get(stream,-1):reasons.append('REUSED_'+stream.upper()+'_FRAME')
            if type(stamp) not in (int,float) or not math.isfinite(stamp):
                reasons.append('INVALID_'+stream.upper()+'_TIMESTAMP')
            else:sdk.append(stamp)
        if len(sdk)==2 and 'TIME_DOMAIN_NOT_COMPARABLE' not in reasons:
            delta=abs(sdk[0]-sdk[1])
            if delta>self.max_skew_ms:reasons.append('RGBD_SKEW_EXCEEDED')
            for stream,stamp in zip(('color','depth'),sdk):
                prior=self.last_accepted.get(stream+'_sdk_ms')
                if prior is not None and stamp<=prior:reasons.append('NON_INCREASING_'+stream.upper()+'_TIME')
        qualified=not reasons
        if qualified:
            self.accepted+=1
            self.last_accepted.update(ids)
            self.last_accepted.update({s+'_sdk_ms':timestamps[s+'_sdk_ms'] for s in ('color','depth')})
        else:self.rejected+=1;self.reason_counts.update(reasons)
        self.last_report=dict(qualified=qualified,reasons=reasons,delta_ms=delta,
            max_skew_ms=self.max_skew_ms,domains=domains,frame_ids=ids)
        return self.last_report

    def snapshot(self):
        return dict(accepted=self.accepted,rejected=self.rejected,reasons=dict(self.reason_counts),
            observed_repeats=dict(self.observed_repeats),observed_id_gaps=dict(self.gaps),
            gap_scope='SDK observed frame IDs, not USB packet loss',last=self.last_report)
