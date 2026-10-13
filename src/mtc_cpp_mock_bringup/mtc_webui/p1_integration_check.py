"""P1 native ROS/DDS observation tests, only in a network-isolated container.

Fixture images, detections, planner and tool evidence are synthetic stimuli.
No physical camera, Planner/FSM/Motion kernel or robot is launched.
"""

import argparse
from dataclasses import replace
import json
from pathlib import Path
import signal
import time
import unittest

from mtc_webui.p0_integration_check import ValidationBackend
from mtc_webui.ros_integration_check import Fixture, wait_for
from mtc_webui.config import RosConfig
from mtc_webui.ros_transport import RosTransport


class P1Fixture(Fixture):
    def __init__(self):
        self.emit_p1=True;self.empty=False;self.nan_pose=False;self.old_pose=False
        self.plan_override={};self.proof_override={};self.tool_attached=False;self.tool_state=1
        super().__init__()
        from mtc_interfaces.msg import BatteryDetectionArray, BatteryDetection
        from std_msgs.msg import String
        self.array_type=BatteryDetectionArray;self.detection_type=BatteryDetection;self.string_type=String
        self.perception=self.node.create_publisher(BatteryDetectionArray,"/mtc/perception/batteries",5)
        self.planner=self.node.create_publisher(String,"/p1/planner",10)
        self.proof=self.node.create_publisher(String,"/p1/tool_verification",10)
        self.evidence_stamp=self.node.get_clock().now().nanoseconds

    def publish(self):
        if not self.emit:return
        import cv2
        import numpy as np
        now=self.node.get_clock().now().to_msg()
        for kind,msg_type in self.types.items():
            m=msg_type();m.header.stamp=now
            if kind=="task_state":
                m.task_id=self.task_id;m.state=self.state;m.substate="TEST_LIFT" if self.task_id else ""
                m.total_count=self.total_count;m.detail="ISOLATED P1 FIXTURE"
            elif kind=="robot":
                m.communication_ok=m.robot_ready=m.safety_normal=True
                m.stop_confirmed=self.stop_confirmed;m.motion_active=not self.stop_confirmed
                m.simulation_mode=False;m.controller_mode="ISOLATED_P1_FIXTURE"
            elif kind=="tool":
                m.state=self.tool_state;m.verified=self.tool_attached;m.evidence_level=2 if self.tool_attached else 1
                m.tool_type="passive_hook_v1";m.object_id="fixture-battery-1"
                m.detail="Synthetic Attached estimate; not real attachment evidence"
            elif kind=="joints":
                m.name=[f"joint_{i}" for i in range(1,7)];m.position=[.1,-.2,.3,-.4,.5,-.6];m.velocity=[0.]*6
            else:
                w,h=640,360;m.width=w;m.height=h;m.header.frame_id="isolated_fixture_camera"
                if kind=="color":
                    frame=np.full((h,w,3),(40,55,65),np.uint8)
                    cv2.putText(frame,"SYNTHETIC ROS FIXTURE",(32,40),cv2.FONT_HERSHEY_SIMPLEX,.85,(220,220,220),2)
                    for x,c,label in [(70,(210,110,35),"ID 1 / BLUE"),(265,(50,55,205),"ID 2 / RED"),(465,(40,200,230),"ID 3 / YELLOW")]:
                        cv2.rectangle(frame,(x,115),(x+90,250),c,-1)
                        cv2.ellipse(frame,(x+45,110),(24,16),0,180,360,(210,220,220),3)
                        cv2.putText(frame,label,(x-15,285),cv2.FONT_HERSHEY_SIMPLEX,.45,(220,220,220),1)
                    cv2.putText(frame,"NO PHYSICAL CAMERA OR ROBOT",(65,330),cv2.FONT_HERSHEY_SIMPLEX,.65,(90,210,245),1)
                    m.encoding="bgr8";m.step=w*3
                else:
                    frame=np.tile(np.linspace(200,1800,w,dtype=np.uint16),(h,1));frame[:30]=0
                    m.encoding="16UC1";m.step=w*2
                m.data=frame.tobytes()
            self.publishers[kind].publish(m)
        if not hasattr(self,"perception") or not self.emit_p1:return
        stamp=now.sec*1_000_000_000+now.nanosec
        msg=self.array_type();msg.header.stamp=now;msg.header.frame_id="isolated_fixture_camera"
        if not self.empty:
            for i,color in enumerate((2,1,3)):
                d=self.detection_type();d.object_id=f"fixture-battery-{i+1}";d.color=color;d.confidence=.92-i*.04
                d.ring_pose_valid=i==0
                for p in (d.body_pose,d.ring_pose):
                    p.header.stamp=now;p.header.frame_id="isolated_fixture_camera"
                    p.pose.position.x=.15+i*.12;p.pose.position.y=.04;p.pose.position.z=.55
                    p.pose.orientation.w=1.
                if self.nan_pose:d.ring_pose.pose.position.x=float("nan")
                if self.old_pose:
                    from builtin_interfaces.msg import Time
                    # Assignment of generated ROS messages retains the Time object;
                    # don't mutate the array/image/body stamps through a shared ref.
                    d.ring_pose.header.stamp=Time(sec=now.sec-10,nanosec=now.nanosec)
                msg.detections.append(d)
        self.perception.publish(msg)
        p=dict(schema_version=1,stamp_ns=stamp,plan_id="fixture-plan-1",task_id=self.task_id or None,
               primitive="side_approach",state="FAILED",point_count=42,expected_duration_sec=8.7,
               joint_limits_ok=True,collision_free=True,start_state_consistent=False,
               error_code="EXCEEDS_STATE_WATCHDOG",reason="EXCEEDS_STATE_WATCHDOG: trajectory 8.7 s exceeds state watchdog 5.0 s (injected fixture)")
        p.update(self.plan_override);m=self.string_type();m.data=json.dumps(p);self.planner.publish(m)
        proof=dict(schema_version=1,stamp_ns=stamp,verification_id="fixture-grasp-1",task_id=self.task_id or "fixture-no-current-task",
                   object_id="fixture-battery-1",tool_type="passive_hook_v1",
                   test_lift=dict(verified=True,evidence_level=2,source="fixture_relative_motion",stamp_ns=self.evidence_stamp),seated=None)
        proof.update(self.proof_override);m=self.string_type();m.data=json.dumps(proof);self.proof.publish(m)

    def execute(self, handle):
        from mtc_interfaces.action import ExecuteTask
        self.task_id=handle.request.task_id;self.state="EXECUTE_ITEM";self.total_count=len(handle.request.ordered_colors)
        self.stop_confirmed=False
        self.tool_state=3
        deadline=time.monotonic()+30
        while self.running and time.monotonic()<deadline:
            f=ExecuteTask.Feedback();f.state=self.state;f.substate="TEST_LIFT";f.current_object_id="fixture-battery-1"
            f.current_slot="T0" if handle.request.mode==1 else "P1";f.total_count=self.total_count
            handle.publish_feedback(f)
            if handle.is_cancel_requested:
                handle.abort();self.state="FAULT"
                r=ExecuteTask.Result();r.error_code=520;r.message="Injected ABORTED/520; no stop evidence";return r
            time.sleep(.1)
        handle.succeed();self.state="FINISHED";self.stop_confirmed=True
        r=ExecuteTask.Result();r.success=True;r.completed_count=self.total_count;r.message="isolated fixture only"
        r.completed_slots=["T0"] if handle.request.mode==1 else ["P1","P2","P3"];return r


def configuration():
    return RosConfig(domain_id=94,task_publisher="/webui_isolated_fixture",robot_publisher="/webui_isolated_fixture",
                     perception_publisher="/webui_isolated_fixture",planner_topic="/p1/planner",planner_publisher="/webui_isolated_fixture",
                     tool_verification_topic="/p1/tool_verification",tool_verification_publisher="/webui_isolated_fixture")


class Integration(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.fixture=P1Fixture();cls.backend=ValidationBackend(configuration());RosTransport(cls.backend).start()
        try:wait_for(lambda:cls.backend.snapshot()["perception"].get("fresh"))
        except Exception:cls.backend.close();cls.fixture.close();raise

    @classmethod
    def tearDownClass(cls):cls.backend.close();cls.fixture.close()

    def setUp(self):
        self.fixture.emit=True;self.fixture.emit_p1=True;self.fixture.empty=False
        self.fixture.nan_pose=False;self.fixture.old_pose=False;self.fixture.plan_override={};self.fixture.proof_override={}
        self.fixture.tool_state=1

    def test_01_native_detection_and_rgb_depth_frames(self):
        wait_for(lambda:all(self.backend.snapshot()["camera"][k].get("fresh") for k in ("color","depth")))
        s=self.backend.snapshot();self.assertEqual(len(s["perception"]["data"]["detections"]),3)
        self.assertEqual(s["perception"]["data"]["detections"][0]["object_id"],"fixture-battery-1")
        import cv2,numpy as np
        for kind in ("color","depth"):
            jpeg,meta=self.backend.camera_packet(kind)
            frame=cv2.imdecode(np.frombuffer(jpeg,np.uint8),cv2.IMREAD_COLOR)
            self.assertEqual(frame.shape,(360,640,3));self.assertEqual(meta["data"]["frame_id"],"isolated_fixture_camera")
            self.assertTrue(meta["source_stamp"].isdigit())

    def test_02_nan_pose_old_pose_and_zero_detections(self):
        self.fixture.nan_pose=True
        wait_for(lambda:self.backend.snapshot()["perception"]["data"]["detections"][0]["ring_pose"]["position"][0] is None)
        self.assertTrue(self.backend.snapshot()["perception"]["data"]["detections"][0]["ring_pose_valid"])
        self.fixture.nan_pose=False;self.fixture.old_pose=True
        wait_for(lambda:self.backend.snapshot()["perception"]["data"]["detections"][0]["ring_pose"]["offset_ms"]>9000)
        self.fixture.empty=True
        wait_for(lambda:self.backend.snapshot()["perception"]["data"]["detections"]==[])
        self.assertTrue(self.backend.snapshot()["perception"]["fresh"])

    def test_03_watchdog_raw_reason_and_new_plan_does_not_inherit_checks(self):
        wait_for(lambda:self.backend.snapshot()["planner"].get("fresh"))
        s=self.backend.snapshot()["planner"]["data"]
        self.assertEqual(s["error_code"],"EXCEEDS_STATE_WATCHDOG");self.assertIn("8.7 s",s["reason"])
        self.fixture.plan_override=dict(plan_id="fixture-plan-2",state="PLANNING",joint_limits_ok=None,collision_free=None,start_state_consistent=None,reason=None,error_code=None)
        wait_for(lambda:self.backend.snapshot()["planner"]["data"]["plan_id"]=="fixture-plan-2")
        self.assertIsNone(self.backend.snapshot()["planner"]["data"]["collision_free"])
        self.assertTrue(any("EXCEEDS_STATE_WATCHDOG" in e["message"] for e in self.backend.snapshot()["events"]))

    def test_04_attached_without_independent_mount_and_invalid_evidence_rejected(self):
        self.fixture.tool_state=3
        wait_for(lambda:self.backend.snapshot()["tool"]["data"]["state"]==3)
        wait_for(lambda:self.backend.snapshot()["tool_verification"].get("fresh"))
        s=self.backend.snapshot();self.assertEqual(s["tool"]["data"]["state"],3);self.assertFalse(s["tool"]["data"]["verified"])
        self.assertIsNone(s["tool_verification"]["data"]["seated"])
        self.fixture.proof_override=dict(test_lift=dict(verified=True,evidence_level=1,source="command_ack",stamp_ns=self.fixture.evidence_stamp))
        wait_for(lambda:not self.backend.snapshot()["tool_verification"]["available"])

    def test_05_duplicate_publisher_revokes_attribution(self):
        extra=self.fixture.node.create_publisher(self.fixture.array_type,"/mtc/perception/batteries",5)
        try:wait_for(lambda:not self.backend.snapshot()["perception"]["available"])
        finally:self.fixture.node.destroy_publisher(extra)
        wait_for(lambda:self.backend.snapshot()["perception"].get("fresh"))

    def test_06_stale_data_and_frames_do_not_remain_current(self):
        wait_for(lambda:self.backend.snapshot()["planner"].get("fresh"))
        self.fixture.emit=False
        wait_for(lambda:not self.backend.snapshot()["perception"].get("fresh"),timeout=6)
        self.assertFalse(self.backend.snapshot()["planner"]["fresh"])
        from mtc_webui.mock_backend import ApiError
        with self.assertRaises(ApiError):self.backend.camera_packet("color")


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument("--isolated-fixture",action="store_true",required=True)
    parser.add_argument("--serve",action="store_true");parser.add_argument("--output-dir",type=Path);args=parser.parse_args()
    signal.signal(signal.SIGTERM,lambda *_:(_ for _ in ()).throw(KeyboardInterrupt()))
    if not args.serve:unittest.main(argv=["p1_integration_check"],verbosity=2);return
    from mtc_webui.server import Server
    fixture=P1Fixture();backend=ValidationBackend(configuration(),args.output_dir);RosTransport(backend).start()
    try:
        with Server(("127.0.0.1",8769),backend) as server:server.serve_forever()
    except KeyboardInterrupt:pass
    finally:backend.close();fixture.close()


if __name__=="__main__":main()
