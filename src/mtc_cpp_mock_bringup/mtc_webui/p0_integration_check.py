"""P0 ROS observation fixture; only run in an isolated container, fixed domain 94.

Generated messages exercise the UI client, not the FSM, Phase 2C or real hardware.
The injected ABORTED/520 checks display fidelity; it does not revalidate or fix C1.
"""

import argparse
import json
from pathlib import Path
import signal
import time
import unittest

from mtc_webui.config import RosConfig
from mtc_webui.ros_backend import RosBackend
from mtc_webui.ros_transport import RosTransport
from mtc_webui.ros_integration_check import Fixture, wait_for


class P0Fixture(Fixture):
    def __init__(self):
        self.emit_motion = True
        self.completed = 0
        self.child_state = ""
        self.motion_state = "IDLE"
        self.motion_id = None
        self.elapsed = self.duration = None
        self.motion_override = None
        self.fjt_handle = None
        super().__init__()
        from control_msgs.action import FollowJointTrajectory
        from control_msgs.msg import JointTrajectoryControllerState
        from std_msgs.msg import String
        from rclpy.action import ActionServer, ActionClient, CancelResponse
        self.controller_type = JointTrajectoryControllerState
        self.string_type = String
        self.controller = self.node.create_publisher(JointTrajectoryControllerState, "/p0_controller/controller_state", 10)
        self.motion = self.node.create_publisher(String, "/p0/execution_state", 10)
        self.fjt_server = ActionServer(self.node, FollowJointTrajectory, "/p0_controller/follow_joint_trajectory",
            self.execute_fjt, cancel_callback=lambda _: CancelResponse.ACCEPT, callback_group=self.group)
        # The fixture alone owns FJT Goals. The Web observer creates no send_goal client.
        self.fjt_client = ActionClient(self.node, FollowJointTrajectory, "/p0_controller/follow_joint_trajectory", callback_group=self.group)

    def publish(self):
        if not self.emit:return
        for kind in ("task_state", "robot", "tool", "joints"):
            message = self.types[kind]()
            message.header.stamp = self.node.get_clock().now().to_msg()
            if kind == "task_state":
                message.task_id, message.state, message.substate = self.task_id, self.state, self.child_state
                message.completed_count, message.total_count = self.completed, self.total_count
                message.last_error_code = 520 if self.state == "FAULT" else 0
                message.detail = "隔离 ROS 接口夹具；未连接真实机器人"
            elif kind == "robot":
                message.communication_ok = message.robot_ready = message.safety_normal = True
                message.stop_confirmed = self.stop_confirmed
                message.motion_active = not self.stop_confirmed
                message.simulation_mode = False  # Gate stimulus, not hardware evidence.
                message.controller_mode = "ISOLATED_P0_FIXTURE"
            elif kind == "tool":
                message.state, message.evidence_level, message.verified = 1, 3, True
                message.tool_type = "fixture"
            else:
                message.name = [f"joint_{i}" for i in range(1,7)]
                message.position = [.1,-.2,.3,-.4,.5,-.6]
                message.velocity = [.01,-.02,.03,-.04,.05,-.06]
            self.publishers[kind].publish(message)
        if not hasattr(self, "controller"):return
        state = self.controller_type()
        state.header.stamp = self.node.get_clock().now().to_msg()
        state.joint_names = [f"joint_{i}" for i in range(1,7)]
        state.feedback.positions = [.1,-.2,.3,-.4,.5,-.6]
        state.feedback.velocities = [.01,-.02,.03,-.04,.05,-.06]
        state.reference.positions = [.12,-.21,.31,-.42,.5,-.6]
        state.reference.velocities = [.01,-.02,.03,-.04,.05,-.06]
        state.error.positions = [.02,-.01,.01,-.02,0.,0.]
        self.controller.publish(state)
        if self.emit_motion:
            payload = dict(schema_version=1, stamp_ns=self.node.get_clock().now().nanoseconds,
                command_id="fixture-command-"+(self.motion_id or "idle"),
                trajectory_id="fixture-trajectory-"+(self.motion_id or "idle"), fjt_goal_id=self.motion_id,
                task_id=self.task_id or None, state=self.motion_state, primitive="side_approach" if self.motion_id else None,
                elapsed_sec=self.elapsed, expected_duration_sec=self.duration,
                progress=min(self.elapsed/self.duration,1.) if self.elapsed is not None and self.duration else None,
                fault_locked=self.state=="FAULT", reason="隔离消息夹具；不代表实机或 Phase 2C 验收")
            if self.motion_override:payload.update(self.motion_override)
            message=self.string_type();message.data=json.dumps(payload);self.motion.publish(message)

    def start_motion(self, duration=1.5):
        from control_msgs.action import FollowJointTrajectory
        from trajectory_msgs.msg import JointTrajectoryPoint
        assert self.fjt_client.wait_for_server(timeout_sec=3)
        goal=FollowJointTrajectory.Goal()
        goal.trajectory.joint_names=[f"joint_{i}" for i in range(1,7)]
        point=JointTrajectoryPoint();point.positions=[.2]*6
        point.time_from_start.sec=int(duration);point.time_from_start.nanosec=int(duration%1*1e9)
        goal.trajectory.points=[point]
        future=self.fjt_client.send_goal_async(goal)
        wait_for(future.done)
        self.fjt_handle=future.result()
        assert self.fjt_handle.accepted
        return self.fjt_handle

    def execute_fjt(self, handle):
        from control_msgs.action import FollowJointTrajectory
        self.motion_id=bytes(handle.goal_id.uuid).hex()
        point=handle.request.trajectory.points[-1]
        duration=point.time_from_start.sec+point.time_from_start.nanosec/1e9
        self.duration=duration;self.motion_state="EXECUTING"
        started=time.monotonic()
        while self.running and time.monotonic()-started<duration:
            self.elapsed=time.monotonic()-started
            if handle.is_cancel_requested:
                self.motion_state="CANCELING"
                handle.canceled();self.motion_state="CANCELED"
                result=FollowJointTrajectory.Result();result.error_string="fixture canceled; independent stop remains unknown"
                return result
            feedback=FollowJointTrajectory.Feedback()
            feedback.header.stamp=self.node.get_clock().now().to_msg()
            feedback.joint_names=[f"joint_{i}" for i in range(1,7)]
            feedback.actual.positions=[.1,-.2,.3,-.4,.5,-.6]
            feedback.actual.velocities=[.01,-.02,.03,-.04,.05,-.06]
            feedback.desired.positions=[.12,-.21,.31,-.42,.5,-.6]
            feedback.error.positions=[.02,-.01,.01,-.02,0.,0.]
            feedback.actual.time_from_start.sec=int(self.elapsed)
            feedback.actual.time_from_start.nanosec=int(self.elapsed%1*1e9)
            handle.publish_feedback(feedback);time.sleep(.05)
        handle.succeed();self.motion_state="SUCCEEDED";self.elapsed=duration
        result=FollowJointTrajectory.Result();result.error_string="isolated fixture FJT success"
        return result

    def execute(self, handle):
        from mtc_interfaces.action import ExecuteTask
        self.task_id,self.state,self.stop_confirmed=handle.request.task_id,"EXECUTE_ITEM",False
        self.total_count=len(handle.request.ordered_colors);self.completed=0
        self.start_motion(4*self.total_count)
        stages=["RESOLVE_TARGET","MOVE_PRE_ALIGN","ENGAGE","TEST_LIFT","LIFT","TRANSPORT","SEAT","VERIFY_PLACED"]
        started=time.monotonic()
        while self.running and time.monotonic()-started<4*self.total_count:
            elapsed=time.monotonic()-started
            index=min(int(elapsed/4),self.total_count-1);self.completed=index
            self.child_state=stages[min(int((elapsed % 4) * 2), 7)]
            feedback=ExecuteTask.Feedback();feedback.current_index=index;feedback.total_count=self.total_count
            feedback.state=self.state;feedback.substate=self.child_state
            feedback.current_object_id=f"fixture-battery-{index+1}"
            feedback.current_slot="T0" if handle.request.mode==1 else f"P{index+1}"
            handle.publish_feedback(feedback)
            if handle.is_cancel_requested:
                self.fjt_handle.cancel_goal_async()
                self.state,self.child_state="FAULT","FAULT"
                handle.abort()
                result=ExecuteTask.Result();result.error_code=520;result.completed_count=self.completed
                result.message="Injected ABORTED/520: Cancel ACK is not independent stop confirmation"
                return result
            time.sleep(.05)
        handle.succeed();self.state="FINISHED";self.child_state="SUCCESS";self.completed=self.total_count
        result=ExecuteTask.Result();result.success=True;result.completed_count=self.total_count
        result.completed_slots=["T0"] if handle.request.mode==1 else ["P1","P2","P3"]
        result.message="isolated fixture success, not robot acceptance"
        self.stop_confirmed=True
        return result

    def close(self):
        self.running=False
        time.sleep(.15)
        self.fjt_server.destroy();self.fjt_client.destroy()
        super().close()


class ValidationBackend(RosBackend):
    def snapshot(self):
        value=super().snapshot();value["validation_scope"]="isolated_ros_fixture";return value


def configuration():
    return RosConfig(domain_id=94,task_publisher="/webui_isolated_fixture",robot_publisher="/webui_isolated_fixture",
        controller_topic="/p0_controller/controller_state",fjt_action="/p0_controller/follow_joint_trajectory",
        motion_topic="/p0/execution_state",motion_publisher="/webui_isolated_fixture")


class Integration(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.fixture=P0Fixture();cls.backend=ValidationBackend(configuration())
        cls.transport=RosTransport(cls.backend);cls.transport.start()
        try:wait_for(lambda:cls.backend.snapshot()["operations"]["submit"])
        except Exception:
            cls.transport.close();cls.fixture.close();raise

    @classmethod
    def tearDownClass(cls):
        cls.backend.close();cls.fixture.close()

    def setUp(self):
        self.fixture.emit=True;self.fixture.emit_motion=True;self.fixture.motion_override=None
        self.fixture.state="WAIT_TASK";self.fixture.task_id="";self.fixture.child_state=""
        self.fixture.stop_confirmed=True
        wait_for(lambda:self.backend.snapshot()["robot"].get("fresh"))

    def test_01_controller_and_fjt_feedback_and_real_result(self):
        wait_for(lambda:self.backend.snapshot()["controller"].get("fresh"))
        self.fixture.emit_motion=False;self.backend.invalidate("motion")
        handle=self.fixture.start_motion()
        identity=bytes(handle.goal_id.uuid).hex()
        wait_for(lambda:self.backend.snapshot()["fjt_feedback"].get("fresh"))
        value=self.backend.snapshot()
        self.assertEqual(value["fjt_feedback"]["data"]["goal_id"],identity)
        self.assertEqual(len(value["controller"]["data"]["actual_velocity"]),6)
        self.assertGreater(value["fjt_feedback"]["data"]["elapsed_sec"],0)
        self.assertNotIn("expected_duration_sec",value["fjt_feedback"]["data"])
        wait_for(lambda:self.backend.snapshot()["fjt_result"] is not None)
        self.assertEqual(self.backend.snapshot()["fjt_result"]["status"],"SUCCEEDED")

    def test_02_observer_has_no_send_goal_or_cancel_client(self):
        node=self.transport.node
        names=node.get_client_names_and_types_by_node(node.get_name(),node.get_namespace())
        endpoints={name for name,_ in names if name.startswith("/p0_controller/")}
        self.assertEqual(endpoints,{"/p0_controller/follow_joint_trajectory/_action/get_result"})

    def test_03_cancel_ack_and_injected_520_remain_separate(self):
        wait_for(lambda:self.backend.snapshot()["operations"]["submit"])
        ack=self.backend.submit(dict(task_id="p0-cancel",mode=2,ordered_colors=[2,1,3],timeout_ms=30000,placement_stability_sec=3))
        wait_for(lambda:self.backend.snapshot()["task"]["substate"]=="MOVE_PRE_ALIGN")
        self.backend.cancel(ack["task_id"],ack["goal_id"])
        wait_for(lambda:self.backend.snapshot()["task"]["cancel_ack"] is not None and self.backend.snapshot()["task"]["result"] is not None)
        snapshot=self.backend.snapshot()
        self.assertTrue(snapshot["task"]["cancel_ack"]["request_delivered"])
        self.assertEqual(snapshot["task"]["status"],"ABORTED")
        self.assertEqual(snapshot["task"]["result"]["error_code"],520)
        self.assertFalse(snapshot["stop_evidence"]["confirmed"])

    def test_04_stop_reply_requires_new_independent_evidence(self):
        self.fixture.stop_confirmed=False
        wait_for(lambda:self.backend.snapshot()["robot"]["data"]["stop_confirmed"] is False)
        self.backend.request_stop()
        wait_for(lambda:self.backend.snapshot()["stop_request"]["request_delivered"] is True)
        self.assertFalse(self.backend.snapshot()["stop_evidence"]["confirmed"])
        self.fixture.stop_confirmed=True
        wait_for(lambda:self.backend.snapshot()["stop_evidence"]["confirmed"])

    def test_05_malformed_motion_and_stale_observations(self):
        self.fixture.motion_override=dict(progress=3)
        wait_for(lambda:not self.backend.snapshot()["motion"]["available"])
        self.fixture.motion_override=None
        wait_for(lambda:self.backend.snapshot()["motion"].get("fresh"))
        self.fixture.emit=False
        wait_for(lambda:not self.backend.snapshot()["controller"].get("fresh"),timeout=6)
        self.assertFalse(self.backend.snapshot()["operations"]["submit"])
        self.assertFalse(self.backend.snapshot()["stop_evidence"]["confirmed"])


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--isolated-fixture",action="store_true",required=True)
    parser.add_argument("--serve",action="store_true")
    parser.add_argument("--output-dir",type=Path)
    args=parser.parse_args()
    signal.signal(signal.SIGTERM,lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    if not args.serve:
        unittest.main(argv=["p0_integration_check"],verbosity=2);return
    from mtc_webui.server import Server
    fixture=P0Fixture();backend=ValidationBackend(configuration(),args.output_dir)
    RosTransport(backend).start()
    try:
        with Server(("127.0.0.1",8769),backend) as server:server.serve_forever()
    except KeyboardInterrupt:pass
    finally:backend.close();fixture.close()


if __name__=="__main__":main()
