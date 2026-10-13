"""rclpy adapter. All ROS work stays on one executor; HTTP threads only enqueue.

This client neither implements the FSM nor launches drivers or sends joint motion.
"""

import math
import os
import queue
import threading
import time
import uuid
from mtc_webui.telemetry import controller_sample, fjt_sample, execution_sample
from mtc_webui.observations import perception_sample, planner_sample, tool_verification_sample


def fields(message, names):
    result = {}
    for name in names.split():
        value = getattr(message, name)
        result[name] = list(value) if not isinstance(value, (str, int, float, bool)) else value
    return result


def encode_image(message, kind):
    """Respect ROS Image stride, endian and encoding. Depth preview: fixed 0–2 m."""
    import cv2
    import numpy as np
    if kind == "depth" and (hasattr(message, "format") or message.encoding.lower() not in {"16uc1", "32fc1"}):
        raise ValueError("深度预览需要原始 16UC1（mm）或 32FC1（m）图像")
    if hasattr(message, "format"):
        encoded = np.frombuffer(bytes(message.data), dtype=np.uint8)
        frame = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
        if frame is None:
            raise ValueError("CompressedImage 无法解码")
        encoding = message.format
    else:
        encoding = message.encoding.lower()
        formats = {"rgb8": ("u1", 3), "bgr8": ("u1", 3), "rgba8": ("u1", 4),
                   "bgra8": ("u1", 4), "mono8": ("u1", 1), "16uc1": ("u2", 1), "32fc1": ("f4", 1)}
        if encoding not in formats:
            raise ValueError(f"不支持的图像编码 {encoding}")
        dtype, channels = formats[encoding]
        dtype = np.dtype((">" if message.is_bigendian else "<") + dtype)
        width, height, step = message.width, message.height, message.step
        if not (0 < width <= 4096 and 0 < height <= 4096) or step < width * channels * dtype.itemsize or len(message.data) < step * height:
            raise ValueError("Image 尺寸、步长或缓冲区无效")
        frame = np.ndarray((height, width, channels), dtype=dtype, buffer=bytes(message.data),
                           strides=(step, channels * dtype.itemsize, dtype.itemsize)).copy()
        if channels == 1:
            frame = frame[:, :, 0]
        if encoding in ("16uc1", "32fc1"):
            meters = frame.astype(np.float32) * (0.001 if encoding == "16uc1" else 1)
            valid = np.isfinite(meters) & (meters > 0)
            normalized = np.nan_to_num(np.clip(meters / 2.0, 0, 1), nan=0, posinf=0, neginf=0)
            frame = cv2.applyColorMap((normalized * 255).astype(np.uint8), cv2.COLORMAP_TURBO)
            frame[~valid] = 0
        elif encoding == "rgb8":
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        elif encoding == "rgba8":
            frame = cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)
        elif encoding == "bgra8":
            frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
    height, width = frame.shape[:2]
    if width > 960:
        frame = cv2.resize(frame, (960, round(height * 960 / width)))
    ok, encoded = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
    if not ok:
        raise ValueError("JPEG 编码失败")
    return encoded.tobytes(), dict(width=width, height=height, encoding=encoding,
                                  preview="深度伪彩 0–2 m；非标定测量" if kind == "depth" else "RGB 图像")


from mtc_webui.diagnostics import calibration_sample, trace_sample, goal_key


class RosTransport:
    def __init__(self, backend):
        self.backend, self.config = backend, backend.config
        self.commands = queue.Queue()
        self.done = threading.Event()
        self.thread = None
        self.handles, self.subscriptions, self.sources, self.topics = {}, {}, {}, {}
        self.fjt_name = ""
        self.fjt_requested = set()
        self.pick_requested = set()
        self.last_frame, self.last_image_error = {}, {}
        self.deadlines = {}
        self.context = self.node = self.executor = None
        backend.transport = self

    def start(self):
        if self.config.domain_id is None:
            self.backend.connection_update(reason="未配置 ROS_DOMAIN_ID；使用实机环境的值启动")
            return
        if os.environ.get("ROS_DISTRO") != "jazzy":
            self.backend.connection_update(reason="尚未 source ROS 2 Jazzy 与含 mtc_interfaces 的工作空间")
            return
        self.thread = threading.Thread(target=self._run, name="webui-ros", daemon=True)
        self.thread.start()

    def enqueue(self, command, identity, payload):
        self.commands.put((command, identity, payload))

    def _run(self):
        try:
            import rclpy
            from rclpy.context import Context
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.node import Node
            from rclpy.action import ActionClient
            from rclpy.qos import qos_profile_sensor_data
            from mtc_interfaces.action import ExecuteTask
            from mtc_interfaces.msg import TaskState, RobotState, ToolState, BatteryDetectionArray
            from mtc_interfaces.srv import RequestStop
            from sensor_msgs.msg import JointState
            from rcl_interfaces.msg import Log
            self.ExecuteTask, self.RequestStop = ExecuteTask, RequestStop
            self.context = Context()
            rclpy.init(args=[], context=self.context, domain_id=self.config.domain_id)
            self.node = Node("mtc_web_console_" + self.backend.session_id[:8], context=self.context)
            self.executor = SingleThreadedExecutor(context=self.context)
            self.executor.add_node(self.node)
            self.action = ActionClient(self.node, ExecuteTask, self.config.task_action)
            self.stop = self.node.create_client(RequestStop, self.config.stop_service)
            self.qos = qos_profile_sensor_data
            specs = [("task_state", TaskState, self.config.task_topic, self.config.task_publisher),
                     ("robot", RobotState, self.config.robot_topic, self.config.robot_publisher),
                     ("tool", ToolState, self.config.tool_topic, self.config.tool_publisher),
                     ("joints", JointState, self.config.joint_topic, self.config.joint_publisher),
                     ("perception", BatteryDetectionArray, self.config.perception_topic, self.config.perception_publisher)]
            for kind, msg_type, topic, publisher in specs:
                if topic:
                    self._subscribe(kind, msg_type, topic, publisher)
            from std_msgs.msg import String
            for kind in ("motion", "planner", "tool_verification", "calibration", "trace"):
                if getattr(self.config, kind + "_topic"):
                    self._subscribe(kind, String, getattr(self.config, kind + "_topic"),
                                    getattr(self.config, kind + "_publisher"))
            self._subscribe_pick()
            if self.config.tf_pairs:
                from tf2_ros import Buffer, TransformListener
                self.tf_buffer = Buffer(node=self.node)
                self.tf_listener = TransformListener(self.tf_buffer, self.node, spin_thread=False)
                self.node.create_timer(0.5, self._check_tf)
            # Volatile best-effort reader accepts both common rosout publisher profiles.
            self.node.create_subscription(Log, "/rosout", self._log, qos_profile_sensor_data)
            self.node.create_timer(0.05, self._drain)
            self.node.create_timer(0.5, self._graph)
            self.backend.connection_update(connected=True, reason="ROS 已连接；等待有效设备状态")
            while not self.done.is_set():
                self.executor.spin_once(timeout_sec=0.1)
        except Exception as error:
            self.backend.connection_update(connected=False, action_ready=False, stop_ready=False,
                                           reason=f"ROS 接入失败：{type(error).__name__}: {error}")
        finally:
            if self.executor:
                self.executor.shutdown(timeout_sec=1)
            if hasattr(self, "action"):
                self.action.destroy()
            if self.node:
                self.node.destroy_node()
            if self.context and self.context.ok():
                self.context.shutdown()

    def _subscribe(self, kind, msg_type, topic, publisher="", qos=None):
        # Exactly two positional args: rclpy uses the signature to pass MessageInfo.
        def callback(message, info):
            self._receive(kind, message, info)
        self.topics[kind] = (topic, publisher)
        self.subscriptions[kind] = self.node.create_subscription(msg_type, topic, callback, qos or self.qos)

    def _subscribe_pick(self):
        from mtc_interfaces.action import PickPlace
        from action_msgs.msg import GoalStatusArray
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        name = self.config.pick_action
        if not name:
            return
        self.pick_result_type = PickPlace.Impl.GetResultService
        self.pick_get_result = self.node.create_client(self.pick_result_type, name + "/_action/get_result")
        self._subscribe("pick_status", GoalStatusArray, name + "/_action/status", self.config.pick_publisher,
                        QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self._subscribe("pick_feedback", PickPlace.Impl.FeedbackMessage, name + "/_action/feedback", self.config.pick_publisher)

    def _pick_result(self, goal_id):
        if goal_id in self.pick_requested or not self.pick_get_result.service_is_ready():
            return
        self.pick_requested.add(goal_id)
        if len(self.pick_requested) > 500:
            self.pick_requested = {goal_id}
        request = self.pick_result_type.Request()
        request.goal_id.uuid = list(uuid.UUID(goal_id).bytes)
        source = self.sources.get("pick_status")
        future = self.pick_get_result.call_async(request)
        def received(done):
            try:
                self._authority("pick_status")
                if not source or self.sources.get("pick_status") != source:
                    return
                response = done.result()
                self.backend.record_pick_result(goal_id, response.status,
                    fields(response.result, "success placement_verified error_code message"), source[1])
            except Exception as error:
                self.backend.log("webui", 30, f"PickPlace 结果读取失败：{error}")
        future.add_done_callback(received)

    def _check_tf(self):
        from rclpy.time import Time
        now = self.node.get_clock().now().nanoseconds
        pairs = []
        for target, source in self.config.tf_pairs:
            row = dict(target=target, source=source, available=False, reason=None, static_cache=None)
            try:
                transform = self.tf_buffer.lookup_transform(target, source, Time())
                stamp = transform.header.stamp.sec * 1_000_000_000 + transform.header.stamp.nanosec
                age = (now - stamp) / 1e6 if stamp else None
                t, q = transform.transform.translation, transform.transform.rotation
                values = [t.x, t.y, t.z, q.x, q.y, q.z, q.w]
                finite = all(math.isfinite(v) for v in values) and abs(sum(v*v for v in values[3:]) - 1) <= .02
                row.update(available=finite and (stamp == 0 or -500 <= age < self.config.freshness_ms),
                           transform_stamp=str(stamp), age_ms=round(age) if age is not None else None, static_cache=stamp == 0,
                           reason="静态 TF 缓存可查询" if stamp == 0 and finite else "动态 TF 可查询" if finite and -500 <= age < self.config.freshness_ms else "TF 数值或时间戳无效/过期")
            except Exception as error:
                row['reason'] = str(error)[:1000]
            pairs.append(row)
        self.backend.observe("tf", dict(pairs=pairs, available=all(r['available'] for r in pairs)),
                             "本机 tf2 Buffer 只读查询；静态缓存不证明标定有效或发布节点在线", now, 0)

    def _subscribe_fjt(self, name):
        from control_msgs.action import FollowJointTrajectory
        from action_msgs.msg import GoalStatusArray
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        self.fjt_name = name
        # No FJT ActionClient or send_goal/cancel_goal client. Reading an already
        # terminal result cannot take ownership of execution or change a Goal.
        self.fjt_result_type = FollowJointTrajectory.Impl.GetResultService
        self.fjt_get_result = self.node.create_client(self.fjt_result_type, name + "/_action/get_result")
        self._subscribe("fjt_status", GoalStatusArray, name + "/_action/status", self.config.fjt_publisher,
                        QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE,
                                   durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self._subscribe("fjt_feedback", FollowJointTrajectory.Impl.FeedbackMessage,
                        name + "/_action/feedback", self.config.fjt_publisher)

    def _fjt_result(self, goal_id):
        if goal_id in self.fjt_requested or not self.fjt_get_result.service_is_ready():
            return
        self.fjt_requested.add(goal_id)
        # Only one observed terminal Goal is requested; retain a bounded dedupe set.
        if len(self.fjt_requested) > 500:
            self.fjt_requested = {goal_id}
        request = self.fjt_result_type.Request()
        request.goal_id.uuid = list(uuid.UUID(goal_id).bytes)
        source = self.sources.get("fjt_status")
        future = self.fjt_get_result.call_async(request)
        def received(done):
            try:
                self._authority("fjt_status")
                if not source or self.sources.get("fjt_status") != source:
                    return
                response = done.result()
                self.backend.record_fjt_result(goal_id, response.status, response.result.error_code,
                                               response.result.error_string, source[1])
            except Exception as error:
                self.backend.log("webui", 30, f"FJT 结果读取失败：{error}")
        future.add_done_callback(received)

    def _authority(self, kind):
        topic, pin = self.topics[kind]
        entries = []
        for info in self.node.get_publishers_info_by_topic(topic):
            name = info.node_namespace.rstrip("/") + "/" + info.node_name
            entries.append(dict(node=name, gid=bytes(info.endpoint_gid).hex()))
        # Jazzy 7.1.12 MessageInfo does not expose publisher_gid. Never choose one
        # of several writers by name then attribute every message to that writer.
        chosen = None
        if len(entries) == 1 and self.subscriptions[kind].get_publisher_count() == 1:
            entry = entries[0]
            if "_UNKNOWN_" not in entry["node"] and (not pin or entry["node"] == pin):
                chosen = (entry["gid"], f"{entry['node']} · {topic} · {entry['gid'][:8]}")
        if chosen != self.sources.get(kind):
            self.backend.invalidate(kind)
            self.sources[kind] = chosen
        return dict(topic=topic, expected_node=pin, candidates=entries,
                    selected=chosen[1] if chosen else None,
                    attribution="unique topic publisher; graph GID, not per-message GID")

    def _graph(self):
        from rclpy.action.graph import get_action_server_names_and_types_by_node
        from sensor_msgs.msg import Image, CompressedImage
        topics = self.node.get_topic_names_and_types()
        controller_topics = sorted(name for name, types in topics
                                   if types == ["control_msgs/msg/JointTrajectoryControllerState"])
        controller_topic = self.config.controller_topic or (controller_topics[0] if len(controller_topics) == 1 else "")
        if controller_topic and "controller" not in self.subscriptions:
            from control_msgs.msg import JointTrajectoryControllerState
            self._subscribe("controller", JointTrajectoryControllerState, controller_topic,
                            self.config.controller_publisher)
        image_topics = sorted(name for name, types in topics if any(t in ("sensor_msgs/msg/Image", "sensor_msgs/msg/CompressedImage") for t in types))
        for kind in ("color", "depth"):
            if kind in self.subscriptions:
                continue
            topic = getattr(self.config, kind + "_topic")
            if not topic:
                suffixes = ("/color/image_raw", "/color/image_raw/compressed") if kind == "color" else ("/aligned_depth_to_color/image_raw", "/depth/image_rect_raw")
                for suffix in suffixes:
                    matches = [name for name in image_topics if name.endswith(suffix)]
                    if len(matches) == 1:
                        topic = matches[0]
                        break
                    if len(matches) > 1:
                        break
            topic_types = dict(topics).get(topic, [])
            if topic and len(topic_types) == 1 and topic_types[0] in ("sensor_msgs/msg/Image", "sensor_msgs/msg/CompressedImage"):
                self._subscribe(kind, Image if topic_types[0].endswith("/Image") else CompressedImage, topic)
        publishers = {}
        for kind in self.topics:
            publishers[kind] = self._authority(kind)
        action_servers = []
        stop_servers = []
        pick_servers = []
        fjt_servers = {}
        nodes = []
        for name, namespace in self.node.get_node_names_and_namespaces():
            full_name = namespace.rstrip("/") + "/" + name
            if "_UNKNOWN_" not in full_name:
                nodes.append(full_name)
            for action, types in get_action_server_names_and_types_by_node(self.node, name, namespace):
                if action == self.config.task_action and "mtc_interfaces/action/ExecuteTask" in types:
                    action_servers.append(full_name)
                if action == self.config.pick_action and "mtc_interfaces/action/PickPlace" in types:
                    pick_servers.append(full_name)
                if "control_msgs/action/FollowJointTrajectory" in types:
                    fjt_servers.setdefault(action, []).append(full_name)
            for service, types in self.node.get_service_names_and_types_by_node(name, namespace):
                if service == self.config.stop_service and "mtc_interfaces/srv/RequestStop" in types:
                    stop_servers.append(full_name)
        fjt_name = self.config.fjt_action or (next(iter(fjt_servers)) if len(fjt_servers) == 1 else "")
        if fjt_name and not self.fjt_name:
            self._subscribe_fjt(fjt_name)
        self.backend.connection_update(connected=True, reason="ROS 已连接；设备状态按来源与时间戳核验",
                                       action_ready=len(action_servers) == 1 and self.action.server_is_ready(),
                                       stop_ready=len(stop_servers) == 1 and self.stop.service_is_ready(),
                                       action_servers=action_servers, stop_servers=stop_servers,
                                       publishers=publishers, image_topics=image_topics, nodes=sorted(nodes),
                                       pick_servers=pick_servers, controller_topics=controller_topics,
                                       fjt_action=self.fjt_name, fjt_servers=fjt_servers)

    def _receive(self, kind, message, info):
        # Refresh authority on each sample, not only on the periodic graph poll.
        self._authority(kind)
        source = self.sources.get(kind)
        if not source:
            return
        # Newer distributions may add the field; never rely on it being present.
        gid = info.get("publisher_gid")
        if gid is not None and bytes(gid).hex() != source[0]:
            return
        if kind in ("motion", "planner", "tool_verification", "calibration", "trace"):
            try:
                import json
                if len(message.data) > 32768:
                    raise ValueError(kind + " telemetry 超过 32 KiB")
                raw = json.loads(message.data)
                data = {"motion": execution_sample, "planner": planner_sample,
                        "tool_verification": tool_verification_sample, "calibration": calibration_sample, "trace": trace_sample}[kind](raw)
                stamp = raw["stamp_ns"]
                age = (self.node.get_clock().now().nanoseconds - stamp) / 1e6
                self.backend.observe(kind, data, source[1], stamp, age)
            except (ValueError, TypeError, KeyError) as error:
                self.backend.invalidate(kind)
                self.backend.log("webui", 30, str(error))
            return
        if kind in ("pick_status", "pick_feedback"):
            stamp = info.get("source_timestamp", 0)
            age = (time.time_ns() - stamp) / 1e6
            if kind == "pick_status":
                entries = [dict(goal_id=bytes(e.goal_info.goal_id.uuid).hex(), status=e.status) for e in message.status_list]
                try:
                    for entry in entries: goal_key(entry["goal_id"])
                except ValueError:
                    self.backend.invalidate(kind)
                    return
                terminal = self.backend.observe_pick_status(entries, source[1], stamp, age)
                if terminal and stamp > 0 and -500 <= age < self.config.freshness_ms:
                    self._pick_result(terminal)
            else:
                data = fields(message.feedback, "substate progress object_attached_estimated detail")
                if not math.isfinite(data["progress"]) or not 0 <= data["progress"] <= 1:
                    data["progress"] = None
                self.backend.observe_pick_feedback(bytes(message.goal_id.uuid).hex(), data, source[1], stamp, age)
            return
        if kind == "fjt_status":
            # GoalStatusArray has no Header. DDS publication time is freshness
            # evidence; Goal acceptance time is not the message timestamp.
            stamp = info.get("source_timestamp", 0)
            age = (time.time_ns() - stamp) / 1e6
            entries = [dict(goal_id=bytes(e.goal_info.goal_id.uuid).hex(), status=e.status)
                       for e in message.status_list]
            try:
                for entry in entries: goal_key(entry["goal_id"])
            except ValueError:
                self.backend.invalidate(kind)
                return
            terminal = self.backend.observe_fjt_status(entries, source[1], stamp, age)
            if terminal and stamp > 0 and -500 <= age < self.config.freshness_ms:
                self._fjt_result(terminal)
            return
        header = message.feedback.header if kind == "fjt_feedback" else message.header
        stamp = header.stamp.sec * 1_000_000_000 + header.stamp.nanosec
        age = (self.node.get_clock().now().nanoseconds - stamp) / 1_000_000
        if kind == "perception":
            try:
                self.backend.observe(kind, perception_sample(message), source[1], stamp, age)
            except (ValueError, TypeError) as error:
                self.backend.invalidate(kind)
                self.backend.log("webui", 30, str(error))
            return
        if kind == "fjt_feedback":
            self.backend.observe_fjt_feedback(fjt_sample(message), source[1], stamp, age)
            return
        if kind == "controller":
            self.backend.observe(kind, controller_sample(message), source[1], stamp, age)
            return
        if kind in ("color", "depth"):
            if time.monotonic() - self.last_frame.get(kind, 0) < 0.2:
                return
            self.last_frame[kind] = time.monotonic()
            try:
                jpeg, data = encode_image(message, kind)
                data.update(frame_id=message.header.frame_id, topic=self.topics[kind][0])
                self.backend.frame(kind, data, jpeg, source[1], stamp, age)
            except Exception as error:
                self.backend.invalidate(kind)
                text = f"{kind} 图像：{error}"
                if self.last_image_error.get(kind) != text:
                    self.backend.log("webui", 30, text)
                    self.last_image_error[kind] = text
            return
        names = {"task_state": "task_id state substate completed_count total_count last_error_code detail",
                 "robot": "communication_ok robot_ready motion_active safety_normal stop_confirmed simulation_mode controller_mode detail",
                 "tool": "state evidence_level verified tool_type object_id detail",
                 "joints": "name position velocity effort"}
        data = fields(message, names[kind])
        if kind == "joints":
            for key in ("position", "velocity", "effort"):
                data[key] = [v if math.isfinite(v) else None for v in data[key]]
        self.backend.observe(kind, data, source[1], stamp, age)

    def _log(self, message):
        # Avoid mirroring unrelated system logs and the console's own messages.
        name = message.name
        configured = {getattr(self.config, k).strip("/").replace("/", ".") for k in self.config.__dataclass_fields__ if k.endswith("_publisher")}
        if ("mtc" in name or name in configured or name in {"task_executor", "motion_executor", "tool_manager", "aubo_driver"}) and "web_console" not in name:
            self.backend.log(name, message.level, message.msg, str(message.stamp.sec * 1_000_000_000 + message.stamp.nanosec))

    def _drain(self):
        while not self.commands.empty():
            command, identity, payload = self.commands.get_nowait()
            try:
                if command == "submit":
                    self._submit(identity, payload)
                elif command == "cancel":
                    handle = self.handles.get(identity)
                    if handle is None:
                        self.backend.cancel_reply(identity, False, "该 Goal 句柄已不可用")
                        continue
                    future = handle.cancel_goal_async()
                    self.deadlines[("cancel", identity)] = time.monotonic() + 10
                    future.add_done_callback(lambda f, key=identity: self._cancel_done(key, f))
                elif command == "stop":
                    request = self.RequestStop.Request()
                    request.reason = "Web operator requested software stop"
                    future = self.stop.call_async(request)
                    self.deadlines[("stop", identity)] = time.monotonic() + 10
                    future.add_done_callback(lambda f, key=identity: self._stop_done(key, f))
            except Exception as error:
                if command == "stop":
                    self.backend.stop_reply(identity, None, f"停止回执未知：{error}")
                else:
                    self.backend.uncertain(identity, f"{command} 回执未知：{error}")
        for key, deadline in list(self.deadlines.items()):
            if time.monotonic() > deadline:
                del self.deadlines[key]
                command, identity = key
                if command == "stop":
                    self.backend.stop_reply(identity, None, "软件停止回执超时；未确认送达或停稳")
                else:
                    self.backend.uncertain(identity, "Cancel 回执超时；等待最终 Result，不自动重发")

    def _submit(self, goal_id, payload):
        from unique_identifier_msgs.msg import UUID
        self._graph()
        if not self.backend.dispatch_allowed(goal_id):
            self.backend.accepted(goal_id, False, "提交前设备状态变化；Goal 未发送")
            return
        goal = self.ExecuteTask.Goal()
        for name, value in payload.items():
            setattr(goal, name, float(value) if name == "placement_stability_sec" else value)
        future = self.action.send_goal_async(goal, goal_uuid=UUID(uuid=list(uuid.UUID(goal_id).bytes)),
                                             feedback_callback=lambda m: self.backend.feedback(str(uuid.UUID(bytes=bytes(m.goal_id.uuid))),
                                                 fields(m.feedback, "current_index total_count state current_slot current_object_id substate")))
        future.add_done_callback(lambda f: self._accepted(goal_id, f))

    def _accepted(self, goal_id, future):
        try:
            handle = future.result()
            if handle.accepted:
                self.handles[goal_id] = handle
            self.backend.accepted(goal_id, handle.accepted)
            if handle.accepted:
                handle.get_result_async().add_done_callback(lambda f: self._result(goal_id, f))
        except Exception as error:
            self.backend.uncertain(goal_id, f"Goal 回执异常：{error}")

    def _result(self, goal_id, future):
        try:
            response = future.result()
            self.backend.result(goal_id, response.status, fields(response.result, "success completed_count completed_slots error_code message"))
            self.handles.pop(goal_id, None)
        except Exception as error:
            self.backend.uncertain(goal_id, f"Result 读取失败：{error}")

    def _cancel_done(self, goal_id, future):
        self.deadlines.pop(("cancel", goal_id), None)
        try:
            reply = future.result()
            delivered = reply.return_code == 0 and any(bytes(g.goal_id.uuid) == uuid.UUID(goal_id).bytes for g in reply.goals_canceling)
            self.backend.cancel_reply(goal_id, delivered, "ROS 已确认接受取消" if delivered else f"ROS 未接受该 Goal 的取消（code={reply.return_code}）")
        except Exception as error:
            self.backend.uncertain(goal_id, f"Cancel 回执异常：{error}")

    def _stop_done(self, identity, future):
        self.deadlines.pop(("stop", identity), None)
        try:
            reply = future.result()
            self.backend.stop_reply(identity, reply.request_delivered, reply.message)
        except Exception as error:
            self.backend.stop_reply(identity, None, f"停止回执异常：{error}")

    def close(self):
        self.done.set()
        if self.thread:
            self.thread.join(timeout=3)
        self.backend.connection_update(connected=False, action_ready=False, stop_ready=False,
                                       reason="操作台已关闭；未自动取消机器人任务")
        task = self.backend.tasks.get(self.backend.current_id)
        if task:
            self.backend.uncertain(task["goal_id"], "操作台退出，活动 Goal 最终结果尚未确认")
