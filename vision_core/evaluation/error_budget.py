"""Worst-case bound ledger; no unknown term is replaced by zero."""
import math
TERMS=('vision','camera_extrinsic','hand_eye','tool_tcp','robot_positioning','manufacturing')
def compute_budget(config):
    result={'status':'INCOMPLETE_BUDGET','real_world_safety':'NOT_VALIDATED','method':'NONOVERLAPPING_WORST_CASE_BOUND_SUM'}
    unknown=[]
    for term in TERMS:
        row=config.get('terms',{}).get(term,{})
        for key in ['lateral_mm','vertical_mm','yaw_deg']:
            value=row.get(key)
            if value is None:unknown.append(term+'.'+key)
            elif not isinstance(value,(float,int)) or not math.isfinite(value) or value<0:raise ValueError('Invalid error bound')
    result['unknown_terms']=unknown
    if unknown or not config.get('terms_are_nonoverlapping') or config.get('effective_yaw_lever_arm_mm') is None:return result
    rows=config['terms'];lateral=sum(rows[t]['lateral_mm'] for t in TERMS);vertical=sum(rows[t]['vertical_mm'] for t in TERMS)
    yaw=sum(rows[t]['yaw_deg'] for t in TERMS);lever=config['effective_yaw_lever_arm_mm']
    if not isinstance(lever,(float,int)) or not math.isfinite(lever) or lever<0 or yaw>=90:raise ValueError('Invalid lever or angular bounds')
    coupled=lever*math.sin(math.radians(yaw))
    result.update(status='NOMINAL_BOUND_LEDGER_ONLY',total_lateral_mm=lateral,total_vertical_mm=vertical,total_yaw_deg=yaw,
        yaw_induced_lateral_mm=coupled,total_lateral_with_yaw_mm=lateral+coupled,
        nominal_remaining_lateral_mm=config['nominal_lateral_clearance_mm']-lateral-coupled,
        nominal_remaining_vertical_mm=config['nominal_vertical_clearance_mm']-vertical)
    return result
