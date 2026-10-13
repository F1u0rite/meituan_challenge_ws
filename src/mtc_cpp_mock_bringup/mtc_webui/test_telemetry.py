"""P0 regression checks: real observation identity, missing data and stop evidence."""

from dataclasses import replace
from types import SimpleNamespace as N
import unittest

from mtc_webui.config import RosConfig
from mtc_webui.ros_backend import RosBackend
from mtc_webui.telemetry import controller_sample, execution_sample, fjt_sample
from mtc_webui.test_ros_backend import Transport


class Telemetry(unittest.TestCase):
    def setUp(self):
        self.now = 100.
        self.backend = RosBackend(RosConfig(domain_id=94), clock=lambda: self.now, wall_clock=lambda: self.now)
        self.backend.transport = Transport()
        self.backend.connection_update(connected=True, action_ready=True, stop_ready=True)

    def ready(self):
        for key, value in {"robot":dict(communication_ok=True, robot_ready=True, motion_active=False,
                safety_normal=True, stop_confirmed=True, simulation_mode=False),
                "tool":dict(state=1), "task_state":dict(task_id="",state="WAIT_TASK",substate="",
                completed_count=0,total_count=0,last_error_code=0,detail="ready")}.items():
            self.backend.observe(key,value,"/fixture",round(self.now*1e9),0)

    def task(self):
        self.ready()
        value = self.backend.submit(dict(task_id="p0", mode=2, ordered_colors=[2,1,3],timeout_ms=30000,placement_stability_sec=3))
        self.backend.accepted(value["goal_id"],True)
        return value["goal_id"]

    def test_no_feedback_does_not_claim_first_battery(self):
        self.task()
        snapshot=self.backend.snapshot()
        self.assertIsNone(snapshot["task"]["current_index"])
        self.assertFalse(snapshot["task"]["feedback_fresh"])

    def test_current_item_and_completion_count_are_independent(self):
        identity=self.task()
        self.backend.feedback(identity,dict(current_index=1,total_count=3,current_object_id="battery-2",
            current_slot="P2",state="EXECUTE_ITEM",substate="SEAT"))
        task=self.backend.snapshot()["task"]
        self.assertEqual(task["current_index"],1)
        self.assertEqual(task["completed_count"],0)
        self.now+=4
        self.assertFalse(self.backend.snapshot()["task"]["feedback_fresh"])

    def test_cancel_ack_and_520_preserve_unknown_stop(self):
        identity=self.task()
        self.backend.cancel("p0",identity)
        self.backend.cancel_reply(identity,True,"Cancel accepted")
        self.backend.result(identity,6,dict(success=False,completed_count=0,completed_slots=[],error_code=520,message="stop unconfirmed"))
        snapshot=self.backend.snapshot()
        self.assertEqual(snapshot["task"]["status"],"ABORTED")
        self.assertEqual(snapshot["task"]["result"]["error_code"],520)
        self.assertTrue(snapshot["task"]["cancel_ack"]["request_delivered"])
        self.assertFalse(snapshot["stop_evidence"]["confirmed"])
        self.assertFalse(snapshot["operations"]["submit"])

    def test_pre_request_true_and_delayed_old_sample_do_not_confirm_stop(self):
        self.ready()
        self.assertTrue(self.backend.stop_evidence()["confirmed"])
        self.now+=1
        request=self.backend.request_stop()
        self.backend.stop_reply(request["request_id"],True,"delivered")
        self.assertFalse(self.backend.stop_evidence()["confirmed"])
        self.now+=0.1
        self.backend.observe("robot",dict(stop_confirmed=True,motion_active=False,simulation_mode=False),"/fixture",100_500_000_000,600)
        self.assertFalse(self.backend.stop_evidence()["confirmed"])
        self.now+=0.1
        self.backend.observe("robot",dict(stop_confirmed=True,motion_active=False,simulation_mode=False),"/fixture",101_200_000_000,0)
        self.assertTrue(self.backend.stop_evidence()["confirmed"])
        self.now+=4
        self.assertFalse(self.backend.stop_evidence()["confirmed"])

    def test_simultaneous_motion_and_stop_bits_are_not_stop_confirmation(self):
        self.backend.observe("robot",dict(stop_confirmed=True,motion_active=True,simulation_mode=False),"robot",1,0)
        self.assertFalse(self.backend.stop_evidence()["confirmed"])

    def test_simulation_flag_cannot_confirm_real_stop(self):
        self.backend.observe("robot",dict(stop_confirmed=True,motion_active=False,simulation_mode=True),"robot",1,0)
        self.assertFalse(self.backend.stop_evidence()["confirmed"])

    def test_bad_task_timestamp_does_not_overwrite_fresh_action_feedback(self):
        identity=self.task()
        self.backend.feedback(identity,dict(state="EXECUTE_ITEM",substate="TRANSPORT"))
        self.backend.observe("task_state",dict(task_id="p0",state="FAULT",substate="FAULT",completed_count=2,
            total_count=3,last_error_code=520,detail="old"),"other",1,5000)
        self.assertEqual(self.backend.snapshot()["task"]["state"],"EXECUTE_ITEM")

    def status(self, entries):
        self.now+=.01
        return self.backend.observe_fjt_status(entries,"/controller",round(self.now*1e9),0)

    def test_unrelated_historical_terminal_is_not_current_motion(self):
        self.assertIsNone(self.status([dict(goal_id="cccccccccccccccccccccccccccccccc",status=4)]))
        self.assertIsNone(self.backend.fjt_current_id)
        self.assertIsNone(self.backend.snapshot()["fjt_result"])

    def test_fjt_uuid_switch_rejects_old_feedback_and_late_result(self):
        self.status([dict(goal_id="aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa",status=2)])
        self.backend.observe_fjt_feedback(dict(goal_id="aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa",elapsed_sec=1),"/controller",1,0)
        self.status([dict(goal_id="bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb",status=2),dict(goal_id="aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa",status=4)])
        self.backend.observe_fjt_feedback(dict(goal_id="aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa",elapsed_sec=99),"/controller",2,0)
        self.backend.record_fjt_result("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa",4,0,"old success","/controller")
        snapshot=self.backend.snapshot()
        self.assertFalse(snapshot["fjt_feedback"]["available"])
        self.assertIsNone(snapshot["fjt_result"])
        self.assertEqual(snapshot["fjt_status"]["data"]["goal_id"],"bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb")

    def test_multiple_active_goals_and_source_change_invalidate_attribution(self):
        self.status([dict(goal_id="aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa",status=2)])
        self.status([dict(goal_id="aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa",status=2),dict(goal_id="bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb",status=3)])
        self.assertIsNone(self.backend.fjt_current_id)
        self.assertTrue(self.backend.snapshot()["fjt_status"]["data"]["ambiguous"])
        self.status([dict(goal_id="aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa",status=2)])
        self.backend.invalidate("fjt_status")
        self.assertIsNone(self.backend.fjt_current_id)

    def test_fjt_cancel_status_and_result_do_not_synthesize_robot_state(self):
        self.status([dict(goal_id="aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa",status=3)])
        self.assertEqual(self.status([dict(goal_id="aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa",status=5)]),"aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa")
        self.backend.record_fjt_result("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa",5,-4,"cancel with tolerance failure","/controller")
        data=self.backend.snapshot()
        self.assertEqual(data["fjt_result"]["error_code"],-4)
        self.assertFalse(data["robot"]["available"])
        self.assertFalse(data["stop_evidence"]["confirmed"])

    def test_motion_contract_keeps_optional_fields_missing(self):
        d=execution_sample(dict(schema_version=1,stamp_ns=1,command_id="cmd",state="EXECUTING",stop_confirmed=True))
        for key in ("progress","expected_duration_sec","primitive","trajectory_id","elapsed_sec"):
            self.assertIsNone(d[key])
        self.assertNotIn("stop_confirmed",d)

    def test_motion_contract_rejects_fabricated_or_malformed_values(self):
        base=dict(schema_version=1,stamp_ns=1,command_id="cmd",state="EXECUTING")
        for bad in (dict(progress=1.2),dict(progress=float("nan")),dict(elapsed_sec=True),dict(stamp_ns=0),
                    dict(expected_duration_sec=-1),dict(state="MOVE_PRE_ALIGN"),dict(success="yes"),dict(fjt_goal_id="not-uuid")):
            with self.subTest(bad=bad),self.assertRaises(ValueError):execution_sample({**base,**bad})
        with self.assertRaises(ValueError):replace(RosConfig(),motion_topic="/motion")

    def test_joint_conversion_does_not_zero_fill_missing_velocity(self):
        point=lambda p,v: N(positions=p,velocities=v,time_from_start=N(sec=2,nanosec=100000000))
        controller=N(joint_names=["joint_6","joint_1"],feedback=point([1,float("nan")],[]),
                     reference=point([2,3],[]),error=point([1,0],[]))
        sample=controller_sample(controller)
        self.assertEqual(sample["joint_names"],["joint_6","joint_1"])
        self.assertEqual(sample["actual_position"],[1,None])
        self.assertEqual(sample["actual_velocity"],[])
        feedback=N(goal_id=N(uuid=list(range(16))),feedback=N(joint_names=["joint_1"],
            actual=point([1],[.1]),desired=point([2],[.2]),error=point([1],[])))
        converted=fjt_sample(feedback)
        self.assertEqual(converted["elapsed_sec"],2.1)
        self.assertNotIn("progress",converted)
        self.assertNotIn("expected_duration_sec",converted)


if __name__ == "__main__":unittest.main()
