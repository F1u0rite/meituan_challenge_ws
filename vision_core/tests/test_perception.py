from synthetic.scene_generator import SceneGenerator
from perception.battery_detector import BatteryDetector
from evaluation.evaluator import Evaluator


def test_four_batteries_in_canonical_image():
    gen=SceneGenerator(); scene=gen.generate(42)
    predictions=BatteryDetector(gen.config).detect(scene.image)
    assert len(predictions)==4
    result=Evaluator(gen.config).evaluate(scene.ground_truth,predictions)
    assert not result.failed_cases
