"""Human polygon regions for red surface, full object, and negative background."""
from datetime import datetime, timezone
from pathlib import Path
import json
import uuid
import numpy as np
from synthetic.io import write_png
from .annotation import roi_mask, read_mask
from .dataset import load_raw

ROLES = ('color_surface', 'full_object', 'negative_background')


def read_regions(sample):
    path = Path(sample)/'annotation_regions.json'
    if not path.exists():
        return {'sample_id': Path(sample).name, 'raw_image': 'raw.png',
                'annotation_kind': 'human_polygon', 'regions': []}
    data = json.loads(path.read_text(encoding='utf-8'))
    if data.get('sample_id') != Path(sample).name:
        raise ValueError('Annotation sample ID mismatch')
    return data


def annotation_source(sample):
    data = read_regions(sample)
    return data.get('annotation_source',
                    'codex_provisional' if data.get('annotation_kind') == 'codex_visual_provisional'
                    else 'user')


def review_status(sample):
    data = read_regions(sample)
    return data.get('review_status',
                    'pending_user_review' if annotation_source(sample) == 'codex_provisional'
                    else 'user_annotated')


def set_review_status(sample, status):
    """Only an explicit reviewer action may accept or reject a Codex proposal."""
    if status not in ('user_accepted', 'user_rejected'):
        raise ValueError('Invalid review status')
    sample = Path(sample)
    data = read_regions(sample)
    if annotation_source(sample) != 'codex_provisional':
        raise ValueError('Only Codex-provisional images need this review')
    if status == 'user_accepted' and not complete(sample):
        raise ValueError('All three region roles are required before acceptance')
    data['annotation_source'] = 'codex_provisional'
    data['review_status'] = status
    data['reviewed_at'] = datetime.now(timezone.utc).isoformat()
    (sample/'annotation_regions.json').write_text(
        json.dumps(data, ensure_ascii=False, indent=2), encoding='utf-8')
    return status


def replace_role_for_review(sample, role, vertices):
    """Keep original proposals in audit history while saving a user edit."""
    if annotation_source(sample) != 'codex_provisional':
        raise ValueError('User-authored annotations are read-only in this reviewer')
    region = save_region(sample, role, vertices)
    sample = Path(sample)
    data = read_regions(sample)
    for item in data['regions']:
        if item['role'] == role and item['annotation_id'] != region['annotation_id'] and item.get('active', True):
            item['active'] = False
            item['replaced_at'] = datetime.now(timezone.utc).isoformat()
        if item['annotation_id'] == region['annotation_id']:
            item['annotator'] = 'user_edit'
            item['annotation_source'] = 'user_edit'
    data['annotation_source'] = 'codex_provisional'
    data['review_status'] = 'pending_user_review'
    (sample/'annotation_regions.json').write_text(
        json.dumps(data, ensure_ascii=False, indent=2), encoding='utf-8')
    return region


def save_region(sample, role, vertices):
    if role not in ROLES:
        raise ValueError('Unknown annotation role')
    sample = Path(sample)
    image = load_raw(sample)
    mask = roi_mask(image.shape, 'polygon', vertices=vertices)
    annotation_id = uuid.uuid4().hex
    filename = f'region_{annotation_id}.png'
    write_png(sample/filename, mask)
    data = read_regions(sample)
    region = {'annotation_id': annotation_id, 'role': role, 'mode': 'polygon',
              'vertices_px': np.asarray(vertices, dtype=int).tolist(),
              'mask_path': filename, 'active': True,
              'annotated_at': datetime.now(timezone.utc).isoformat()}
    data['regions'].append(region)
    (sample/'annotation_regions.json').write_text(json.dumps(data, indent=2), encoding='utf-8')
    return region


def undo_last_region(sample):
    sample = Path(sample)
    data = read_regions(sample)
    for region in reversed(data['regions']):
        if region.get('active', True):
            region['active'] = False
            region['undone_at'] = datetime.now(timezone.utc).isoformat()
            (sample/'annotation_regions.json').write_text(json.dumps(data, indent=2), encoding='utf-8')
            return region
    return None


def active_regions(sample, role=None):
    return [r for r in read_regions(sample)['regions']
            if r.get('active', True) and (role is None or r['role'] == role)]


def complete(sample):
    return all(active_regions(sample, role) for role in ROLES)


def combined_mask(sample, role):
    image = load_raw(sample)
    mask = np.zeros(image.shape[:2], bool)
    for region in active_regions(sample, role):
        mask |= read_mask(sample, region)
    return mask


def selected_annotation_status(selection_path):
    selection = json.loads(Path(selection_path).read_text(encoding='utf-8'))
    return [{'sample_id': row['sample_id'], 'sample_path': row['sample_path'],
             'complete': bool(complete(row['sample_path'])),
             'region_counts': {role: len(active_regions(row['sample_path'], role)) for role in ROLES}}
            for row in selection['selected']]
