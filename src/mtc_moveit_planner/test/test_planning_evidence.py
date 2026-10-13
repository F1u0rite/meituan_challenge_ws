"""Evidence gates: failure attribution, child lifecycle, and immutable output.

These tests do not start ROS or send robot goals. Codex-assisted integration
checks; planning/FSM/Action/Motion semantics are unchanged.
"""
import importlib.util
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

PACKAGE = Path(__file__).resolve().parents[1]
SCRIPTS = PACKAGE / "scripts"
sys.path.insert(0, str(SCRIPTS))
spec = importlib.util.spec_from_file_location("check_planning_evidence", SCRIPTS / "check_planning_evidence.py")
checker = importlib.util.module_from_spec(spec)
spec.loader.exec_module(checker)

START = "[INFO] [plan_fixed_pose-1]: process started with pid [100]\n"
END = "[INFO] [plan_fixed_pose-1]: process has finished cleanly [pid 100]\n"
OK = "[plan_fixed_pose-1] [INFO] [123] [mtc_plan_fixed_pose]: PLAN OK: duration=1 -> /tmp/plan.yaml\n"
REJECT = "[plan_fixed_pose-1] [ERROR] [123] [mtc_plan_fixed_pose]: PLAN FAILED [NO_PLAN] no valid IK candidate\n"


class PlanningEvidenceTest(unittest.TestCase):
    def test_positive_plan_requires_clean_tool_exit(self):
        self.assertTrue(checker.classify(START + OK + END)[0])

    def test_sequence_success(self):
        text = (START + OK + END).replace("PLAN OK:", "SEQUENCE PLAN OK")
        self.assertTrue(checker.classify(text)[0])

    def test_success_does_not_pass_collision_probe(self):
        passed, detail = checker.classify(START + OK + END, True)
        self.assertFalse(passed)
        self.assertIn("unexpected planning success", detail)

    def test_no_plan_cannot_prove_collision(self):
        passed, detail = checker.classify(START + REJECT, True)
        self.assertFalse(passed)
        self.assertIn("rejection observed [NO_PLAN]", detail)
        self.assertIn("collision cause UNVERIFIED", detail)

    def test_other_rejections_cannot_prove_collision(self):
        for reason in ("INVALID_START", "TF_UNAVAILABLE", "TIMEOUT"):
            with self.subTest(reason=reason):
                passed, detail = checker.classify(REJECT.replace("NO_PLAN", reason), True)
                self.assertFalse(passed)
                self.assertIn("UNVERIFIED", detail)

    def test_scene_initialization_failure_is_not_goal_acceptance(self):
        passed, detail = checker.classify("failed to apply planning scene obstacles", True)
        self.assertFalse(passed)
        self.assertIn("NO_PLANNING_RESULT", detail)
        self.assertNotIn("unexpected planning success", detail)

    def test_missing_conclusion_fails_closed(self):
        for text in ("", START, "Planning request accepted", "process has died exit code 2"):
            with self.subTest(text=text):
                self.assertEqual(checker.classify(text)[0], False)
                self.assertIn("NO_PLANNING_RESULT", checker.classify(text)[1])

    def test_crash_after_success_is_not_success(self):
        self.assertFalse(checker.classify(START + OK + "[ERROR] [plan_fixed_pose-1]: process has died [pid 100, exit code -11]\n")[0])

    def test_missing_or_mismatched_exit_fails(self):
        for tail in ("", END.replace("100", "101")):
            with self.subTest(tail=tail):
                self.assertFalse(checker.classify(START + OK + tail)[0])

    def test_conflicting_or_multiple_runs_are_rejected(self):
        for text in (START + OK + REJECT + END, START + OK + OK + END):
            with self.subTest(text=text):
                self.assertFalse(checker.classify(text)[0])

    def test_cli_missing_file_is_failure(self):
        with tempfile.TemporaryDirectory() as tmp:
            p = subprocess.run([sys.executable, str(SCRIPTS / "check_planning_evidence.py"), str(Path(tmp) / "missing.log")], capture_output=True, text=True)
        self.assertEqual(p.returncode, 1)
        self.assertIn("UNREADABLE_EVIDENCE", p.stdout)

    def test_existing_output_is_never_overwritten(self):
        with tempfile.TemporaryDirectory() as tmp:
            base = Path(tmp)
            directory = base / "prior evidence"
            directory.mkdir()
            summary = directory / "SUMMARY.txt"
            summary.write_bytes(b"original failure evidence\x00\n")
            file = base / "file"
            file.write_bytes(b"original")
            link = base / "link"
            link.symlink_to(directory, target_is_directory=True)
            for output in (directory, file, link):
                with self.subTest(output=output):
                    p = subprocess.run(["bash", str(SCRIPTS / "run_planning_evidence.sh"), str(output)], capture_output=True, text=True, timeout=5)
                    self.assertEqual(p.returncode, 2)
                    self.assertIn("must be a new directory", p.stderr)
                    self.assertEqual(summary.read_bytes(), b"original failure evidence\x00\n")
                    self.assertEqual(file.read_bytes(), b"original")
                    self.assertTrue(link.is_symlink())


if __name__ == "__main__":
    unittest.main()
