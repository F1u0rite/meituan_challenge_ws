import numpy as np
from synthetic.scene_generator import SceneGenerator
from synthetic.geometry import rectangle_corners,overlaps


def test_seed_reproducible():
    gen=SceneGenerator()
    a,b=gen.generate(42),gen.generate(42)
    assert a.ground_truth==b.ground_truth
    assert np.array_equal(a.image,b.image)


def test_batteries_inside_and_separate():
    gen=SceneGenerator(); cfg=gen.config
    r=cfg['battery_region']; b=cfg['battery']
    for seed in range(100):
        polygons=[]
        for item in gen.generate(seed).ground_truth['batteries']:
            poly=rectangle_corners(item['x_mm'],item['y_mm'],b['width_mm'],b['height_mm'],item['yaw_deg'])
            assert (poly[:,0]>=r['x_mm']-1e-8).all()
            assert (poly[:,0]<=r['x_mm']+r['width_mm']+1e-8).all()
            assert (poly[:,1]>=r['y_mm']-1e-8).all()
            assert (poly[:,1]<=r['y_mm']+r['height_mm']+1e-8).all()
            assert all(not overlaps(poly, other) for other in polygons)
            polygons.append(poly)
