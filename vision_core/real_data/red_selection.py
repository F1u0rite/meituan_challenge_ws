"""Integrity audit and diverse, time-stratified selection of real red captures."""
from pathlib import Path
import hashlib
import json
import cv2
import numpy as np
import yaml
from synthetic.io import write_png
from .dataset import load_raw


def inspect_session(session, color_config=None):
    session = Path(session)
    color_config = color_config or Path(__file__).resolve().parents[1]/'config'/'realsense_color.yaml'
    red_ranges = yaml.safe_load(Path(color_config).read_text(encoding='utf-8'))['hsv_ranges']['red']
    info = json.loads((session/'session.json').read_text(encoding='utf-8'))
    records, problems = [], []
    seen = set()
    for folder in sorted((session/'samples').iterdir()):
        if not folder.is_dir():
            continue
        try:
            raw_path, metadata_path = folder/'raw.png', folder/'metadata.json'
            if not raw_path.is_file() or not metadata_path.is_file():
                raise ValueError('raw.png or metadata.json missing')
            metadata = json.loads(metadata_path.read_text(encoding='utf-8'))
            image = load_raw(folder)
            if metadata['sample_id'] != folder.name or metadata['session_id'] != info['session_id']:
                raise ValueError('sample/session ID mismatch')
            if folder.name in seen:
                raise ValueError('duplicate sample ID')
            seen.add(folder.name)
            if metadata['capture_label'] != 'red' or metadata['pixel_format'] != 'BGR8':
                raise ValueError('capture label or pixel format mismatch')
            if [image.shape[1], image.shape[0]] != [metadata['image_width'], metadata['image_height']]:
                raise ValueError('dimensions differ from metadata')
            if not (folder/'overlay.png').is_file():
                problems.append({'sample_id': folder.name, 'problem': 'overlay.png missing'})
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            red = np.zeros(hsv.shape[:2], np.uint8)
            for h0,s0,v0,h1,s1,v1 in red_ranges:
                red |= cv2.inRange(hsv, (h0,s0,v0), (h1,s1,v1))
            n, labels, stats, centers = cv2.connectedComponentsWithStats(red)
            if n > 1:
                largest = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
                x,y,w,h,area = stats[largest]
                u,v = centers[largest]
            else:
                x=y=w=h=area=u=v=0
            thumb = cv2.resize(image, (12, 8), interpolation=cv2.INTER_AREA).astype(float).ravel()/255
            feature = np.concatenate((thumb,
                [u/image.shape[1], v/image.shape[0], w/image.shape[1], h/image.shape[0],
                 area/(image.shape[0]*image.shape[1]),
                 metadata['image_quality']['metrics']['value_mean']/255,
                 min(metadata['image_quality']['metrics']['laplacian_variance']/200, 2)]))
            records.append({'sample_id': folder.name, 'sample_path': str(folder.resolve()),
                            'frame_number': metadata['frame_number'],
                            'quality_status': metadata['image_quality']['status'],
                            'sha256_raw': hashlib.sha256(raw_path.read_bytes()).hexdigest(),
                            'rough_red_bbox_px': [int(x),int(y),int(w),int(h)],
                            '_feature': feature, '_image': image})
        except (ValueError, KeyError, OSError, cv2.error, json.JSONDecodeError) as exc:
            problems.append({'sample_id': folder.name, 'problem': str(exc)})
    return info, records, problems


def select_representatives(records, count=28):
    if count < 25 or count > 30:
        raise ValueError('Representative count must be 25..30')
    accepted = sorted((r for r in records if r['quality_status'] == 'ACCEPTED'),
                      key=lambda r: r['frame_number'])
    if len(accepted) < count:
        raise ValueError(f'Only {len(accepted)} ACCEPTED frames; need {count}')
    features = np.vstack([r['_feature'] for r in accepted])
    scale = np.maximum(features.std(axis=0), .08)
    normalized = (features-features.mean(axis=0))/scale
    # Seven temporal bins with farthest-point picks within each bin. Consecutive files
    # cannot dominate merely because an r burst produced many near-duplicate frames.
    bins = np.array_split(np.arange(len(accepted)), 7)
    quotas = [count//7 + (i < count % 7) for i in range(7)]
    chosen = []
    for indices, quota in zip(bins, quotas):
        pool = list(map(int, indices))
        for _ in range(quota):
            if not chosen:
                pick = pool[len(pool)//2]
            else:
                distances = [min(float(np.linalg.norm(normalized[i]-normalized[j])) for j in chosen)
                             for i in pool]
                pick = pool[int(np.argmax(distances))]
            chosen.append(pick)
            pool.remove(pick)
    return sorted((accepted[i] for i in chosen), key=lambda r: r['frame_number'])


def prepare(session, output_dir, count=28):
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    info, records, problems = inspect_session(session)
    selected = select_representatives(records, count)
    def public(record):
        return {k:v for k,v in record.items() if not k.startswith('_')}
    manifest = {'session_id': info['session_id'], 'split': info['split'],
                'total_samples': len(list((Path(session)/'samples').iterdir())),
                'verified_samples': len(records), 'problems': problems,
                'quality_counts': {level: sum(r['quality_status'] == level for r in records)
                                   for level in ('ACCEPTED','WARNING','REJECTED')},
                'selection_method': '7 time strata; farthest image appearance, red candidate size/location, brightness and sharpness within each stratum',
                'selected': [public(r) for r in selected],
                'warning_samples': [public(r) for r in records if r['quality_status'] == 'WARNING']}
    (output_dir/'selection.json').write_text(json.dumps(manifest, indent=2), encoding='utf-8')
    tiles=[]
    for index, record in enumerate(selected, 1):
        tile=cv2.resize(record['_image'], (320,180))
        panel=np.full((205,320,3),20,np.uint8); panel[:180]=tile
        cv2.putText(panel,f'{index:02d} frame {record["frame_number"]}',(7,198),
                    cv2.FONT_HERSHEY_SIMPLEX,.5,(255,255,255),1,cv2.LINE_AA)
        tiles.append(panel)
    while len(tiles)%4:
        tiles.append(np.full_like(tiles[0],20))
    write_png(output_dir/'selected_contact_sheet.png',
              np.vstack([np.hstack(tiles[i:i+4]) for i in range(0,len(tiles),4)]))
    return manifest
