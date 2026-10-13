"""P1 boundaries: native invalid poses, independent evidence and frame identity."""

import base64
import copy
import json
from types import SimpleNamespace as NS
import threading
import unittest
from urllib.request import urlopen

from mtc_webui.config import RosConfig
from mtc_webui.observations import perception_sample, planner_sample, tool_verification_sample
from mtc_webui.ros_backend import RosBackend
from mtc_webui.server import Server
from mtc_webui.test_mock_backend import Clock


def pose(sec=10):
    return NS(header=NS(stamp=NS(sec=sec, nanosec=123), frame_id="camera_optical"),
              pose=NS(position=NS(x=.1,y=.2,z=.4), orientation=NS(x=0.,y=0.,z=0.,w=1.)))


class ObservationTests(unittest.TestCase):
    def test_perception_nan_and_missing_id_are_not_zero_or_valid(self):
        p=pose();p.pose.position.x=float("nan")
        d=NS(object_id="",color=2,confidence=float("inf"),ring_pose_valid=False,body_pose=p,ring_pose=p)
        result=perception_sample(NS(header=pose().header,detections=[d]))
        json.dumps(result,allow_nan=False)
        row=result["detections"][0]
        self.assertIsNone(row["body_pose"]["position"][0]);self.assertIsNone(row["confidence"])
        self.assertFalse(row["body_pose"]["valid"]);self.assertFalse(row["identity_valid"])
        self.assertFalse(row["ring_pose_valid"])

    def test_old_pose_keeps_own_time_and_duplicate_ids_cannot_target(self):
        d=NS(object_id="battery-a",color=2,confidence=.9,ring_pose_valid=True,body_pose=pose(),ring_pose=pose(1))
        rows=perception_sample(NS(header=pose().header,detections=[d,d]))["detections"]
        self.assertEqual(rows[0]["ring_pose"]["offset_ms"],9000)
        self.assertEqual(rows[0]["ring_pose"]["stamp_ns"],"1000000123")
        self.assertFalse(rows[0]["identity_valid"])

    def test_zero_quaternion_future_pose_and_empty_frame_are_invalid(self):
        p=pose(12);p.pose.orientation.w=0.;p.header.frame_id=""
        d=NS(object_id="a",color=99,confidence=-1.,ring_pose_valid=True,body_pose=p,ring_pose=p)
        row=perception_sample(NS(header=pose().header,detections=[d]))["detections"][0]
        self.assertEqual(len(row["ring_pose"]["issues"]),3)
        self.assertEqual(row["color"],99);self.assertIsNone(row["confidence"])

    def test_planner_error_verbatim_and_success_never_fills_checks(self):
        raw=dict(schema_version=1,stamp_ns=10,plan_id="p1",state="FAILED",reason="EXCEEDS_STATE_WATCHDOG: 8.7 > 5.0",error_code="EXCEEDS_STATE_WATCHDOG",collision_free=False)
        failed=planner_sample(raw)
        self.assertEqual(failed["reason"],raw["reason"]);self.assertFalse(failed["collision_free"])
        raw=dict(schema_version=1,stamp_ns=11,plan_id="p2",state="SUCCEEDED")
        success=planner_sample(raw)
        for k in ("joint_limits_ok","collision_free","start_state_consistent","reason","point_count"):
            self.assertIsNone(success[k])

    def test_invalid_telemetry_rejected(self):
        raw=dict(schema_version=1,stamp_ns=10,plan_id="p",state="PLANNING")
        for bad in [dict(schema_version=True),dict(stamp_ns=True),dict(plan_id=" "),dict(state="IDLE"),dict(point_count=True),dict(point_count=-1),dict(expected_duration_sec=float("nan")),dict(collision_free="true"),dict(error_code=True)]:
            with self.subTest(bad=bad),self.assertRaises(ValueError):planner_sample(raw|bad)

    def test_proof_requires_identity_source_independent_level_and_event_stamp(self):
        raw=dict(schema_version=1,stamp_ns=20,verification_id="grasp-1",task_id="task-1",object_id="b1",tool_type="passive_hook_v1",
                 test_lift=dict(verified=True,evidence_level=2,source="vision_relative_motion",stamp_ns=18),seated=None)
        out=tool_verification_sample(raw)
        self.assertTrue(out["test_lift"]["verified"]);self.assertIsNone(out["seated"])
        for bad in [dict(evidence_level=1),dict(source=""),dict(stamp_ns=0),dict(stamp_ns=21),dict(verified="true")]:
            value=copy.deepcopy(raw);value["test_lift"].update(bad)
            with self.subTest(bad=bad),self.assertRaises(ValueError):tool_verification_sample(value)
        del raw["object_id"]
        with self.assertRaises(ValueError):tool_verification_sample(raw)

    def test_optional_publishers_must_be_explicitly_pinned(self):
        for kind in ("planner","tool_verification"):
            with self.assertRaises(ValueError):RosConfig(**{kind+"_topic":"/test"})
        self.assertEqual(RosConfig().perception_topic,"/mtc/perception/batteries")

    def test_http_image_metadata_belongs_to_returned_bytes_and_ns_is_exact(self):
        clock=Clock();backend=RosBackend(RosConfig(),clock=clock)
        backend.connection_update(connected=True)
        stamp=1791871800123456789
        backend.frame("color",dict(frame_id="camera",topic="/color"),b"old-jpeg","source",stamp,0)
        previous=backend.snapshot()["camera"]["color"]
        backend.frame("color",dict(frame_id="camera",topic="/color"),b"new-jpeg","source",stamp+1,0)
        server=Server(("127.0.0.1",0),backend);thread=threading.Thread(target=server.serve_forever,daemon=True);thread.start()
        try:
            with urlopen(f"http://127.0.0.1:{server.server_port}/api/camera/color") as r:
                metadata=json.loads(base64.b64decode(r.headers["X-MTC-Frame"]))
                self.assertEqual(r.read(),b"new-jpeg")
            self.assertEqual(previous["source_stamp"],str(stamp))
            self.assertEqual(metadata["source_stamp"],str(stamp+1))
            self.assertEqual(metadata["session_id"],backend.session_id)
            clock.now+=4
            from mtc_webui.mock_backend import ApiError
            with self.assertRaises(ApiError):backend.camera_packet("color")
        finally:server.shutdown();server.server_close();thread.join()


if __name__=="__main__":unittest.main()
