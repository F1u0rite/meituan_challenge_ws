from types import SimpleNamespace
from synthetic.scene_generator import load_config
from evaluation.evaluator import Evaluator,aggregate


def test_evaluator_math():
    gt={'batteries':[{'id':'B1','color':'red','x_mm':10,'y_mm':20,'yaw_deg':179}]}
    pred=[SimpleNamespace(color='red',x_mm=13,y_mm=24,yaw_deg=1)]
    result=Evaluator(load_config()).evaluate(gt,pred)
    assert result.cases[0]['xy_error_mm']==5
    assert result.cases[0]['yaw_error_deg']==2
    summary=aggregate([result])
    assert summary['detection_success_rate']==1
    assert summary['xy_error_mm']['median']==5
