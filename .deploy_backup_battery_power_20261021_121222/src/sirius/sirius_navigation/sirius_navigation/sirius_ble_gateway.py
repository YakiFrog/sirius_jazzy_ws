#!/usr/bin/env python3
import asyncio
import json
import inspect
import math
import shlex
import subprocess
import sys
import threading
import time
import urllib.request

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, Int32, String

try:
    from .navigation_modes import navigation_mode_controller, normalize_navigation_mode
except ImportError:  # pragma: no cover - direct script execution
    from navigation_modes import navigation_mode_controller, normalize_navigation_mode


SIRIUS_SERVICE_UUID = "A07498CA-AD5B-474E-940D-16F1F1E0A123"
SIRIUS_CHAR_UUID = "A07498CA-AD5B-474E-940D-16F1F1E0A124"


def _append_face_stubs_dir(stubs_dir: str):
    if stubs_dir and stubs_dir not in sys.path:
        sys.path.append(stubs_dir)


class SiriusBleGateway(Node):
    """Own all Sirius BLE traffic from one ROS 2 process."""

    def __init__(self):
        super().__init__("sirius_ble_gateway")

        self.declare_parameter("enable_remote_server", True)
        self.declare_parameter("enable_battery_client", False)
        self.declare_parameter("enable_ear_led_client", True)
        self.declare_parameter("battery_mac", "")
        self.declare_parameter("battery_plaintext", True)
        self.declare_parameter("ear_led_left_mac", "7C:2C:67:64:BD:3A")
        self.declare_parameter("ear_led_right_mac", "7C:2C:67:64:A6:46")
        self.declare_parameter("ear_led_char_uuid", "beb5483e-36e1-4688-b7f5-ea07361b26a8")
        self.declare_parameter("ear_led_period", 0.5)
        self.declare_parameter("battery_poll_interval", 2.0)
        self.declare_parameter("battery_scan_before_connect", False)
        self.declare_parameter("advertise_name", "SiriusBleBridge")
        self.declare_parameter("service_uuid", SIRIUS_SERVICE_UUID)
        self.declare_parameter("characteristic_uuid", SIRIUS_CHAR_UUID)
        self.declare_parameter("nav_http_target", "http://localhost:50060/instruction")
        self.declare_parameter("face_speak_grpc_target", "localhost:50052")
        self.declare_parameter("face_status_grpc_target", "localhost:50051")
        import os
        self.declare_parameter(
            "face_stubs_dir",
            os.path.expanduser("~/sirius_face_anim2/scripts/stubs"),
        )
        self.declare_parameter("publish_face_battery_params", True)
        self.declare_parameter("enable_battery_speech", True)

        self.enable_remote_server = self._bool_param("enable_remote_server")
        self.enable_battery_client = self._bool_param("enable_battery_client")
        self.enable_ear_led_client = self._bool_param("enable_ear_led_client")
        self.battery_mac = self._str_param("battery_mac")
        self.battery_plaintext = self._bool_param("battery_plaintext")
        self.ear_led_left_mac = self._str_param("ear_led_left_mac")
        self.ear_led_right_mac = self._str_param("ear_led_right_mac")
        self.ear_led_char_uuid = self._str_param("ear_led_char_uuid")
        self.ear_led_period = self._float_param("ear_led_period")
        self.battery_poll_interval = self._float_param("battery_poll_interval")
        self.battery_scan_before_connect = self._bool_param("battery_scan_before_connect")
        self.advertise_name = self._str_param("advertise_name")
        self.service_uuid = self._str_param("service_uuid")
        self.characteristic_uuid = self._str_param("characteristic_uuid")
        self.nav_http_target = self._str_param("nav_http_target")
        self.face_speak_grpc_target = self._str_param("face_speak_grpc_target")
        self.face_status_grpc_target = self._str_param("face_status_grpc_target")
        self.face_stubs_dir = self._str_param("face_stubs_dir")
        self.publish_face_battery_params = self._bool_param("publish_face_battery_params")
        self.enable_battery_speech = self._bool_param("enable_battery_speech")

        _append_face_stubs_dir(self.face_stubs_dir)

        self.remote_command_pub = self.create_publisher(String, "/sirius/remote_command", 10)
        self.remote_status_pub = self.create_publisher(String, "/sirius/remote_status", 10)
        self.battery_json_pub = self.create_publisher(String, "/sirius/battery_status", 10)
        self.battery_state_pub = self.create_publisher(BatteryState, "/battery_state", 10)
        self.ear_led_status_pub = self.create_publisher(String, "/sirius/ear_led_status", 10)
        self.stop_pub = self.create_publisher(Bool, "/stop", 10)
        self.instruction_pub = self.create_publisher(String, "/llm_instruction", 10)

        self._blinker_sub = self.create_subscription(
            String,
            "/blinker_led_command",
            self._on_blinker_command,
            10,
        )
        self._remote_command_sub = self.create_subscription(
            String,
            "/sirius/remote_command",
            self._on_remote_command,
            10,
        )
        self._stop_sub = self.create_subscription(Bool, "/stop", self._on_stop_command, 1)
        self._battery_speech_sub = self.create_subscription(
            Bool,
            "/sirius/battery_speech_enable",
            self._on_battery_speech_enable,
            10,
        )
        self._people_count_sub = self.create_subscription(
            Int32,
            "/target_detector/people_count",
            self._on_people_count,
            10,
        )
        self._tracked_people_count_sub = self.create_subscription(
            Int32,
            "/target_detector/tracked_people_count",
            self._on_tracked_people_count,
            10,
        )
        navigation_mode_qos = QoSProfile(depth=1)
        navigation_mode_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        navigation_mode_qos.reliability = ReliabilityPolicy.RELIABLE
        self._navigation_mode_sub = self.create_subscription(
            String,
            "/sirius/navigation_mode",
            self._on_navigation_mode,
            navigation_mode_qos,
        )
        self._controller_selector_sub = self.create_subscription(
            String,
            "/controller_selector",
            self._on_controller_selector,
            navigation_mode_qos,
        )

        self._loop = asyncio.new_event_loop()
        self._thread = threading.Thread(target=self._run_loop, daemon=True)
        self._tasks = []
        self._stopping = threading.Event()
        self._ble_central_lock = asyncio.Lock()

        self._last_face_battery_update = 0.0
        self._last_remote_status = None
        self._last_remote_payload = ""
        self._remote_ble_link = False
        self._remote_last_activity = 0.0
        self._remote_server = None
        self._remote_kernel_advertising = False
        self._battery_device = None
        self._ear_led_left_client = None
        self._ear_led_right_client = None
        self._ear_led_stop = False
        self._emergency_stop_active = False
        self._people_count = None
        self._tracked_people_count = None
        self._people_count_at = 0.0
        self._navigation_mode = "unknown"
        self._controller_selector = ""
        self._estop_heartbeat_timer = self.create_timer(1.0, self._estop_heartbeat_timer_callback)
        self._ear_led_blinking = False
        self._ear_led_blink_on = True
        self._ear_led_left_command = "M:1"
        self._ear_led_right_command = "M:1"
        self._ear_led_signal = "straight"
        self._ear_led_last_left_command = "M:1"
        self._ear_led_last_right_command = "M:1"
        self._ear_led_last_mode = "normal"
        self._last_ear_led_status = None

        # Battery speech trigger state machine variables
        self._has_spoken_40 = False
        self._has_spoken_10 = False
        self._has_spoken_full = False
        self._has_spoken_charging_start = False
        self._was_charging = False
        self._battery_first_run = True
        self._last_critical_time = 0.0
        self._charging_start_time = 0.0

        self._thread.start()
        self._schedule_gateway_tasks()

        self.get_logger().info(
            "Sirius BLE Gateway started "
            f"(remote_server={self.enable_remote_server}, "
            f"battery_client={self.enable_battery_client}, "
            f"ear_led_client={self.enable_ear_led_client})"
        )

    def _bool_param(self, name: str) -> bool:
        value = self.get_parameter(name).value
        if isinstance(value, str):
            return value.lower() in ("1", "true", "yes", "on")
        return bool(value)

    def _str_param(self, name: str) -> str:
        return str(self.get_parameter(name).value)

    def _float_param(self, name: str) -> float:
        return float(self.get_parameter(name).value)

    def _run_loop(self):
        asyncio.set_event_loop(self._loop)
        self._loop.run_forever()

    def _schedule_gateway_tasks(self):
        if self.enable_remote_server:
            self._tasks.append(
                asyncio.run_coroutine_threadsafe(self._run_remote_ble_server(), self._loop)
            )
        if self.enable_battery_client:
            if not self.battery_mac:
                self.get_logger().error(
                    "enable_battery_client=true but battery_mac is empty. "
                    "Battery BLE client will not start."
                )
            else:
                self._tasks.append(
                    asyncio.run_coroutine_threadsafe(self._run_battery_monitor(), self._loop)
                )
        if self.enable_ear_led_client:
            self._tasks.append(
                asyncio.run_coroutine_threadsafe(self._run_ear_led_client(), self._loop)
            )

    def destroy_node(self):
        self._stopping.set()
        if self._remote_kernel_advertising:
            try:
                subprocess.run(
                    self._btmgmt_pty_command("rm-adv", "1"),
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                    timeout=5.0,
                    check=False,
                )
            except Exception as exc:
                self.get_logger().warning(
                    f"Failed to remove kernel-managed BLE advertisement: {exc}"
                )
            self._remote_kernel_advertising = False
        try:
            shutdown = asyncio.run_coroutine_threadsafe(
                self._shutdown_ble_tasks(), self._loop
            )
            shutdown.result(timeout=5.0)
        except Exception as exc:
            self.get_logger().debug(f"BLE task shutdown did not fully complete: {exc}")
        self._loop.call_soon_threadsafe(self._loop.stop)
        self._thread.join(timeout=3.0)
        super().destroy_node()

    async def _shutdown_ble_tasks(self):
        current = asyncio.current_task()
        tasks = [task for task in asyncio.all_tasks() if task is not current]
        for task in tasks:
            task.cancel()
        if tasks:
            await asyncio.gather(*tasks, return_exceptions=True)

    def _on_blinker_command(self, msg: String):
        data = (msg.data or "").strip()
        if not data:
            return
        self._ear_led_signal = data

        if data == "left":
            self._ear_led_left_command = "C:0,255,0"
            self._ear_led_right_command = "C:0,0,0"
            self._ear_led_blinking = True
        elif data == "right":
            self._ear_led_left_command = "C:0,0,0"
            self._ear_led_right_command = "C:0,255,0"
            self._ear_led_blinking = True
        elif data == "hazard":
            self._ear_led_left_command = "C:255,255,0"
            self._ear_led_right_command = "C:255,255,0"
            self._ear_led_blinking = True
        else:
            self._ear_led_left_command = "M:1"
            self._ear_led_right_command = "M:1"
            self._ear_led_blinking = False

        self._ear_led_blink_on = True
        self.get_logger().info(f"Ear LED command: {data}")
        self._publish_ear_led_status(
            "connected" if self._ear_leds_connected() else "disconnected",
            left_command=self._ear_led_left_command,
            right_command=self._ear_led_right_command,
            mode=self._ear_led_mode_text(),
            force=True,
        )

    def _on_stop_command(self, msg: Bool):
        self._ear_led_stop = bool(msg.data)
        state = "emergency" if self._ear_led_stop else "safe"
        self.get_logger().info(f"Ear LED stop state: {state}")
        self._publish_ear_led_status(
            "connected" if self._ear_leds_connected() else "disconnected",
            left_command="C:255,0,0" if self._ear_led_stop else self._ear_led_left_command,
            right_command="C:255,0,0" if self._ear_led_stop else self._ear_led_right_command,
            mode=self._ear_led_mode_text(),
            force=True,
        )

    def _on_battery_speech_enable(self, msg: Bool):
        self.enable_battery_speech = bool(msg.data)
        self.get_logger().info(f"Battery speech output enabled set to: {self.enable_battery_speech}")

    def speak_on_sirius(self, text: str):
        if not self.enable_battery_speech:
            self.get_logger().info(f"Battery speech suppressed (disabled): {text}")
            return
        threading.Thread(target=self._send_to_face_speak, args=(text,), daemon=True).start()

    def _check_battery_speech_triggers(self, data: dict):
        if data.get("status") != "connected":
            return

        percentage = data.get("battery_level", -1.0)
        if percentage < 0:
            return

        is_charging = bool(data.get("is_charging", False))

        SPEECH_CHARGING_START = "[happy]バッテリーの充電を開始しました。"
        SPEECH_BATTERY_LOW_40 = "[sad]バッテリー残量が40パーセント以下になりました。そろそろ充電してください。"
        SPEECH_BATTERY_CRITICAL_10 = "[angry]バッテリー残量が10パーセント以下になりました！大至急、充電してください！"
        SPEECH_BATTERY_FULL = "[happy]バッテリーの充電が１００パーセントになりました！"

        if self._battery_first_run:
            if percentage <= 10:
                self._has_spoken_10 = True
                self._has_spoken_40 = True
                self._last_critical_time = time.time()
            elif percentage <= 40:
                self._has_spoken_40 = True
            elif percentage >= 100:
                self._has_spoken_full = True

            if is_charging:
                self._has_spoken_charging_start = True
            self._was_charging = is_charging
            self._battery_first_run = False
        else:
            # 1. 充電状態の遷移判定
            if is_charging and not self._was_charging:
                self._has_spoken_charging_start = False
                self._charging_start_time = time.time()
            elif not is_charging and self._was_charging:
                self._has_spoken_charging_start = False
                if percentage <= 10:
                    self._last_critical_time = 0.0
                    self._has_spoken_10 = False
            self._was_charging = is_charging

            # 充電開始の報告ロジック
            if is_charging and not self._has_spoken_charging_start:
                time_rem = float(data.get("time_remaining", 0.0))
                if (0.0 < time_rem < 200.0) or (time.time() - self._charging_start_time > 6.0):
                    time_str = self._format_remaining_time(time_rem)
                    if time_str:
                        self.speak_on_sirius(f"{SPEECH_CHARGING_START}満充電までの推定時間は{time_str}です。")
                    else:
                        self.speak_on_sirius(SPEECH_CHARGING_START)
                    self._has_spoken_charging_start = True

            # 2. 満充電の判定
            if percentage >= 100:
                if not self._has_spoken_full:
                    self.speak_on_sirius(SPEECH_BATTERY_FULL)
                    self._has_spoken_full = True
            else:
                if percentage <= 98:
                    self._has_spoken_full = False

            def get_status_report(base_text: str) -> str:
                power_out = float(data.get("total_output", 0.0))
                time_rem = float(data.get("time_remaining", 0.0))
                if power_out > 0 and 0.0 < time_rem < 200.0:
                    time_str = self._format_remaining_time(time_rem)
                    if time_str:
                        insert_text = f"推定残り駆動時間は{time_str}です。"
                        if "以下になりました。" in base_text:
                            return base_text.replace("以下になりました。", f"以下になりました。{insert_text}")
                        elif "以下になりました！" in base_text:
                            return base_text.replace("以下になりました！", f"以下になりました！{insert_text}")
                        return f"{base_text} {insert_text}"
                return base_text

            # 3. バッテリー残量低下の判定
            if percentage <= 10:
                current_time = time.time()
                if not is_charging and (not self._has_spoken_10 or (current_time - self._last_critical_time) >= 120.0):
                    report_text = get_status_report(SPEECH_BATTERY_CRITICAL_10)
                    self.speak_on_sirius(report_text)
                    self._has_spoken_10 = True
                    self._has_spoken_40 = True
                    self._last_critical_time = current_time
            elif percentage <= 40:
                if not self._has_spoken_40 and not is_charging:
                    report_text = get_status_report(SPEECH_BATTERY_LOW_40)
                    self.speak_on_sirius(report_text)
                    self._has_spoken_40 = True
                if percentage >= 13:
                    self._has_spoken_10 = False
                    self._last_critical_time = 0.0
            else:
                if percentage >= 45:
                    self._has_spoken_40 = False
                    self._has_spoken_10 = False
                    self._last_critical_time = 0.0

    def _on_remote_command(self, msg: String):
        text = (msg.data or "").strip()
        if not text:
            return
        self.get_logger().info(f"Remote command topic received: {text}")
        if text.startswith("[nav]"):
            instruction = text[len("[nav]"):].strip()
            if instruction:
                self._process_nav_command(instruction)
            return
        threading.Thread(target=self._send_to_face_speak, args=(text,), daemon=True).start()

    def _estop_heartbeat_timer_callback(self):
        if getattr(self, '_emergency_stop_active', False):
            stop_msg = Bool()
            stop_msg.data = True
            self.stop_pub.publish(stop_msg)

    def _process_nav_command(self, instruction: str):
        if not instruction:
            return

        # 1. 電子緊急停止コマンドであれば直接 /stop にパブリッシュ（即時停止保障）
        try:
            if instruction.startswith("{") and instruction.endswith("}"):
                payload = json.loads(instruction)
                p_type = payload.get("type")
                if p_type in ["emergency_stop", "estop"]:
                    state = bool(payload.get("state", True))
                    self._emergency_stop_active = state
                    self._ear_led_stop = state
                    stop_msg = Bool()
                    stop_msg.data = state
                    self.stop_pub.publish(stop_msg)
                    self.get_logger().warning(f"🚨 [BLE Gateway Direct] Emergency Stop published directly to /stop: state={state}")
            elif any(kw in instruction.lower() for kw in ["緊急停止解除", "電子緊急停止解除", "estop off", "estop_release"]):
                self._emergency_stop_active = False
                self._ear_led_stop = False
                stop_msg = Bool()
                stop_msg.data = False
                self.stop_pub.publish(stop_msg)
                self.get_logger().info("🟢 [BLE Gateway Direct] Emergency Stop RELEASED directly to /stop: state=False")
            elif any(kw in instruction.lower() for kw in ["緊急停止", "電子緊急停止", "estop"]):
                self._emergency_stop_active = True
                self._ear_led_stop = True
                stop_msg = Bool()
                stop_msg.data = True
                self.stop_pub.publish(stop_msg)
                self.get_logger().warning("🚨 [BLE Gateway Direct] Emergency Stop keyword published directly to /stop: state=True")
        except Exception as e:
            self.get_logger().error(f"Error processing nav command in BLE Gateway: {e}")

        # 2. ナビゲーション処理へは必ず一つの経路だけで配送する。
        # 通常は低遅延なROS 2トピックを使い、購読者がいない場合だけHTTPへ
        # フォールバックする。両方へ同時配送すると同じ指令が二重実行される。
        self._forward_nav_instruction(instruction)

    def _forward_nav_instruction(self, instruction: str) -> str:
        """Select one downstream route for a navigation instruction."""
        subscriber_count = self.instruction_pub.get_subscription_count()
        if subscriber_count > 0:
            instr_msg = String()
            instr_msg.data = instruction
            self.instruction_pub.publish(instr_msg)
            return 'ros_topic'

        self.get_logger().warning(
            'No /llm_instruction subscribers; falling back '
            'to navigation HTTP endpoint'
        )
        threading.Thread(
            target=self._send_to_nav_http,
            args=(instruction,),
            daemon=True,
        ).start()
        return 'http'

    async def _stop_remote_server(self):
        try:
            await self._remote_server.stop()
        except Exception as exc:
            self.get_logger().warning(f"Failed to stop BLE remote server: {exc}")

    async def _disconnect_battery(self):
        try:
            await self._battery_device.disconnect()
        except Exception:
            pass

    async def _run_remote_ble_server(self):
        try:
            from bless import (
                BlessServer,
                GATTAttributePermissions,
                GATTCharacteristicProperties,
            )
        except Exception as exc:
            self.get_logger().error(
                "BLE remote server requires 'bless'. "
                f"Install scripts/requirements_ble.txt dependencies. Error: {exc}"
            )
            return

        server = BlessServer(name=self.advertise_name)
        self._remote_server = server
        server.read_request_func = lambda characteristic, **kwargs: characteristic.value
        server.write_request_func = self._handle_remote_ble_write

        await server.add_new_service(self.service_uuid)
        char_flags = (
            GATTCharacteristicProperties.write
            | GATTCharacteristicProperties.read
            | GATTCharacteristicProperties.write_without_response
        )
        permissions = GATTAttributePermissions.writeable | GATTAttributePermissions.readable
        await server.add_new_characteristic(
            self.service_uuid,
            self.characteristic_uuid,
            char_flags,
            None,
            permissions,
        )

        # Ubuntu's BlueZ 5.72 D-Bus advertisement path is rejected by Linux
        # 7.0 with HCI status 0x0d.  Keep Bless for the GATT application, but
        # register the advertising payload through the kernel management API.
        # The narrowly-scoped sudo permission is installed by
        # config/sirius-ble-btmgmt.sudoers.
        async def start_kernel_advertising(adapter):
            await server.app.set_name(adapter, self.advertise_name)
            await self._run_btmgmt("rm-adv", "1", allow_failure=True)
            await self._run_btmgmt(
                "add-adv",
                "-c",
                "-g",
                "-n",
                "-u",
                self.service_uuid,
                "1",
            )
            self._remote_kernel_advertising = True

        async def stop_kernel_advertising(adapter):
            if self._remote_kernel_advertising:
                await self._run_btmgmt("rm-adv", "1", allow_failure=True)
            self._remote_kernel_advertising = False
            await server.app.set_name(adapter, "")

        server.app.start_advertising = start_kernel_advertising
        server.app.stop_advertising = stop_kernel_advertising

        self.get_logger().info(
            f"Remote BLE server advertising as '{self.advertise_name}' "
            f"service={self.service_uuid}"
        )
        await server.start()
        self._publish_remote_status("advertising", ble_link=False, active=False)

        try:
            while not self._stopping.is_set():
                is_connected_value = server.is_connected()
                if inspect.isawaitable(is_connected_value):
                    is_connected_value = await is_connected_value
                is_connected = bool(is_connected_value)
                self._remote_ble_link = is_connected
                active = (time.time() - self._remote_last_activity) < 4.0
                status = "connected" if active else "advertising"
                self._publish_remote_status(status, ble_link=is_connected, active=active)
                await asyncio.sleep(1.0)
        finally:
            await server.stop()
            self._publish_remote_status("stopped", ble_link=False, active=False)

    async def _run_btmgmt(self, *arguments, allow_failure=False):
        command = self._btmgmt_pty_command(*arguments)
        process = await asyncio.create_subprocess_exec(
            *command,
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.STDOUT,
        )
        output_bytes, _ = await process.communicate()
        output = output_bytes.decode("utf-8", errors="replace").strip()
        if process.returncode != 0 and not allow_failure:
            raise RuntimeError(
                "Kernel-managed BLE advertising failed: "
                f"exit={process.returncode}, output={output or '(none)'}"
            )
        if output and (process.returncode == 0 or not allow_failure):
            self.get_logger().debug(f"btmgmt: {output}")
        return process.returncode == 0

    @staticmethod
    def _btmgmt_pty_command(*arguments):
        btmgmt_command = (
            "sudo",
            "-n",
            "/usr/bin/btmgmt",
            "-i",
            "hci0",
            *arguments,
        )
        # btmgmt's non-interactive mode waits indefinitely when stdin is a
        # pipe on this BlueZ/kernel combination. util-linux script(1) gives it
        # a short-lived PTY while preserving its exit status.
        return (
            "/usr/bin/script",
            "--quiet",
            "--return",
            "--command",
            shlex.join(btmgmt_command),
            "/dev/null",
        )

    def _handle_remote_ble_write(self, characteristic, value: bytes, **kwargs):
        try:
            raw_bytes = bytes(value)
            text = raw_bytes.decode("utf-8").strip()
        except Exception as exc:
            self.get_logger().warning(f"Failed to decode BLE remote payload: {exc}")
            text = ""

        status_payload = {
            "type": "status",
            "status": "connected",
            "emergency_stop": getattr(self, '_emergency_stop_active', False),
            "battery": getattr(self, '_last_battery_data', None),
            **self._mode_status_fields(),
            **self._people_status_fields(),
        }
        characteristic.value = json.dumps(status_payload, ensure_ascii=False).encode("utf-8")

        if not text:
            return
        if text == "[ping]":
            self.get_logger().debug("Remote BLE ping received")
            self._remote_last_activity = time.time()
            self._publish_remote_status("connected", ble_link=True, active=True, last_payload="[ping]")
            return

        # 診断用: 受信した生データをINFOレベルでログ出力
        self.get_logger().info(f"🔵 [BLE RAW] len={len(raw_bytes)} hex={raw_bytes.hex()} text={repr(text)}")

        self._remote_last_activity = time.time()
        self._publish_remote_status("connected", ble_link=True, active=True, last_payload=text)

        msg = String()
        msg.data = text
        self.remote_command_pub.publish(msg)

    def _send_to_nav_http(self, instruction: str):
        if not instruction:
            return
        try:
            data = json.dumps({"instruction": instruction}).encode("utf-8")
            req = urllib.request.Request(
                self.nav_http_target,
                data=data,
                headers={"Content-Type": "application/json"},
                method="POST",
            )
            with urllib.request.urlopen(req, timeout=3.0) as response:
                self.get_logger().info(
                    f"Forwarded nav command via HTTP status={response.status}: {instruction}"
                )
        except Exception as exc:
            self.get_logger().error(f"Failed to forward nav command: {exc}")

    def _send_to_face_speak(self, text: str):
        try:
            import grpc
            import face_control_pb2
            import face_control_pb2_grpc

            with grpc.insecure_channel(self.face_speak_grpc_target) as channel:
                stub = face_control_pb2_grpc.PythonControlServiceStub(channel)
                req = face_control_pb2.SpeakRequest(text=text)
                stub.Speak(req, timeout=3.0)
        except Exception as exc:
            self.get_logger().error(f"Failed to forward speak command to face: {exc}")

    def _publish_remote_status(
        self,
        status: str,
        ble_link: bool,
        active: bool,
        last_payload: str = "",
    ):
        if last_payload:
            self._last_remote_payload = last_payload[:80]

        data = {
            "status": status,
            "ble_link": ble_link,
            "active": active,
            "emergency_stop": getattr(self, '_emergency_stop_active', False),
            "battery": getattr(self, '_last_battery_data', None),
            **self._mode_status_fields(),
            **self._people_status_fields(),
            "advertise_name": self.advertise_name,
            "service_uuid": self.service_uuid,
            "stamp": time.time(),
        }
        if self._last_remote_payload:
            data["last_payload"] = self._last_remote_payload

        cache_key = (
            data["status"],
            data["ble_link"],
            data["active"],
            data.get("people_detection_active", False),
            data.get("people_count"),
            data.get("tracked_people_count"),
            data.get("navigation_mode", "unknown"),
            data.get("navigation_mode_effective", ""),
            data.get("controller_selector", ""),
            data.get("last_payload", ""),
        )
        if cache_key == self._last_remote_status and not last_payload:
            return
        self._last_remote_status = cache_key

        msg = String()
        msg.data = json.dumps(data, ensure_ascii=False)
        self.remote_status_pub.publish(msg)

    def _on_people_count(self, msg: Int32):
        self._people_count = max(0, int(msg.data))
        self._people_count_at = time.monotonic()

    def _on_tracked_people_count(self, msg: Int32):
        self._tracked_people_count = max(0, int(msg.data))
        self._people_count_at = time.monotonic()

    def _on_navigation_mode(self, msg: String):
        mode = str(msg.data or "").strip()
        if mode:
            self._navigation_mode = mode

    def _on_controller_selector(self, msg: String):
        controller = str(msg.data or "").strip()
        if controller:
            self._controller_selector = controller

    @staticmethod
    def _effective_navigation_mode(requested, controller_selector):
        """Return the mode implied by the currently active controller.

        The behavior tree resets its controller selection to the default
        (FollowPath) whenever it is rebuilt, while /sirius/navigation_mode
        keeps the last requested value. Deriving the effective mode from the
        live /controller_selector keeps the remote display consistent with the
        robot's actual behaviour.
        """
        requested_mode = normalize_navigation_mode(requested) or "normal"
        if not controller_selector:
            return requested_mode
        if controller_selector == navigation_mode_controller(requested_mode):
            return requested_mode
        return "wait_normal" if controller_selector == "WaitPath" else "normal"

    def _mode_status_fields(self):
        requested = getattr(self, '_navigation_mode', "unknown")
        selector = getattr(self, '_controller_selector', "")
        return {
            "navigation_mode": requested,
            "navigation_mode_effective": self._effective_navigation_mode(
                requested, selector
            ),
            "controller_selector": selector,
        }

    def _people_status_fields(self):
        active = (
            self._people_count_at > 0.0
            and time.monotonic() - self._people_count_at <= 3.0
        )
        return {
            "people_detection_active": active,
            "people_count": self._people_count if active else None,
            "tracked_people_count": (
                self._tracked_people_count if active else None
            ),
        }

    async def _run_ear_led_client(self):
        try:
            from bleak import BleakClient
        except Exception as exc:
            self.get_logger().error(
                "Ear LED BLE client requires 'bleak'. "
                f"Install scripts/requirements_ble.txt dependencies. Error: {exc}"
            )
            return

        self._publish_ear_led_status("connecting")
        while not self._stopping.is_set():
            try:
                await self._ensure_ear_led_connected(BleakClient)
                if not self._ear_leds_connected():
                    await asyncio.sleep(3.0)
                    continue

                if self._ear_led_stop:
                    left_command = right_command = "C:255,0,0"
                    mode = "stop"
                elif not self._ear_led_blinking or self._ear_led_blink_on:
                    left_command = self._ear_led_left_command
                    right_command = self._ear_led_right_command
                    mode = "blink_on" if self._ear_led_blinking else "normal"
                else:
                    left_command = right_command = "C:0,0,0"
                    mode = "blink_off"

                await self._write_ear_leds(left_command, right_command)
                self._ear_led_last_left_command = left_command
                self._ear_led_last_right_command = right_command
                self._ear_led_last_mode = mode
                if self._ear_led_blinking and not self._ear_led_stop:
                    self._ear_led_blink_on = not self._ear_led_blink_on

                self._publish_ear_led_status(
                    "connected",
                    left_command=left_command,
                    right_command=right_command,
                    mode=mode,
                )
            except asyncio.CancelledError:
                break
            except Exception as exc:
                self.get_logger().warning(f"Ear LED BLE error; reconnecting: {exc}")
                await self._disconnect_ear_leds()
                self._publish_ear_led_status("reconnecting", error=str(exc))
                await asyncio.sleep(3.0)

            await asyncio.sleep(max(0.1, self.ear_led_period))

        await self._disconnect_ear_leds()
        self._publish_ear_led_status("disconnected")

    async def _ensure_ear_led_connected(self, BleakClient):
        async with self._ble_central_lock:
            if not self._ear_led_left_client or not self._ear_led_left_client.is_connected:
                self.get_logger().info(f"Connecting left ear LED BLE: {self.ear_led_left_mac}")
                self._ear_led_left_client = BleakClient(self.ear_led_left_mac)
                try:
                    await asyncio.wait_for(self._ear_led_left_client.connect(), timeout=4.0)
                except Exception as exc:
                    self.get_logger().warning(f"Left Ear LED connection timed out / failed: {exc}")
                    self._ear_led_left_client = None

            if not self._ear_led_right_client or not self._ear_led_right_client.is_connected:
                self.get_logger().info(f"Connecting right ear LED BLE: {self.ear_led_right_mac}")
                self._ear_led_right_client = BleakClient(self.ear_led_right_mac)
                try:
                    await asyncio.wait_for(self._ear_led_right_client.connect(), timeout=4.0)
                except Exception as exc:
                    self.get_logger().warning(f"Right Ear LED connection timed out / failed: {exc}")
                    self._ear_led_right_client = None

    def _ear_leds_connected(self) -> bool:
        return bool(
            self._ear_led_left_client
            and self._ear_led_left_client.is_connected
            and self._ear_led_right_client
            and self._ear_led_right_client.is_connected
        )

    async def _write_ear_leds(self, left_command: str, right_command: str):
        if not self._ear_leds_connected():
            return
        await asyncio.gather(
            self._ear_led_left_client.write_gatt_char(
                self.ear_led_char_uuid,
                left_command.encode("utf-8"),
            ),
            self._ear_led_right_client.write_gatt_char(
                self.ear_led_char_uuid,
                right_command.encode("utf-8"),
            ),
        )

    async def _disconnect_ear_leds(self):
        for client in (self._ear_led_left_client, self._ear_led_right_client):
            if not client:
                continue
            try:
                if client.is_connected:
                    await client.disconnect()
            except Exception:
                pass
        self._ear_led_left_client = None
        self._ear_led_right_client = None

    def _publish_ear_led_status(
        self,
        status: str,
        error: str = "",
        left_command: str = "",
        right_command: str = "",
        mode: str = "",
        force: bool = False,
    ):
        left_command = left_command or self._ear_led_last_left_command or self._ear_led_left_command
        right_command = right_command or self._ear_led_last_right_command or self._ear_led_right_command
        mode = mode or self._ear_led_last_mode or self._ear_led_mode_text()
        data = {
            "status": status,
            "left_mac": self.ear_led_left_mac,
            "right_mac": self.ear_led_right_mac,
            "left_connected": bool(
                self._ear_led_left_client and self._ear_led_left_client.is_connected
            ),
            "right_connected": bool(
                self._ear_led_right_client and self._ear_led_right_client.is_connected
            ),
            "signal": "stop" if self._ear_led_stop else self._ear_led_signal,
            "stop": self._ear_led_stop,
            "blinking": self._ear_led_blinking,
            "blink_on": self._ear_led_blink_on,
            "mode": mode,
            "left_command": left_command,
            "right_command": right_command,
            "stamp": time.time(),
        }
        if error:
            data["error"] = error[:160]

        cache_key = (
            data["status"],
            data["left_connected"],
            data["right_connected"],
            data["signal"],
            data["stop"],
            data["blinking"],
            data["blink_on"],
            data["mode"],
            data["left_command"],
            data["right_command"],
            data.get("error", ""),
        )
        if cache_key == self._last_ear_led_status and not force:
            return
        self._last_ear_led_status = cache_key

        msg = String()
        msg.data = json.dumps(data, ensure_ascii=False)
        self.ear_led_status_pub.publish(msg)

    def _ear_led_mode_text(self) -> str:
        if self._ear_led_stop:
            return "stop"
        if self._ear_led_blinking:
            return "blink_on" if self._ear_led_blink_on else "blink_off"
        return "normal"

    async def _run_battery_monitor(self):
        solix = self._load_solix_support()
        if not solix:
            return

        C300, ChargingStatus, BleakScanner, BLEDevice = solix
        self._publish_battery_json({"status": "connecting"})

        while not self._stopping.is_set():
            try:
                if not self._battery_device or not self._battery_device.connected:
                    self._publish_battery_json({"status": "connecting"})
                    async with self._ble_central_lock:
                        if self._battery_device:
                            await self._disconnect_battery()
                        self._battery_device = await self._resolve_and_create_station(
                            C300,
                            BleakScanner,
                            BLEDevice,
                        )
                        self.get_logger().info(f"Connecting to battery BLE: {self.battery_mac}")
                        try:
                            connected = await asyncio.wait_for(self._battery_device.connect(), timeout=8.0)
                        except Exception as exc:
                            self.get_logger().warning(f"Battery BLE connect timed out/failed: {exc}")
                            connected = False
                        if not connected:
                            self.get_logger().warning("Battery BLE connection failed; retrying")
                            await asyncio.sleep(3.0)
                            continue

                try:
                    await self._battery_device.get_status_update()
                except Exception as exc:
                    self.get_logger().warning(f"Battery polling failed; reconnecting: {exc}")
                    await self._disconnect_battery()
                    self._battery_device = None
                    await asyncio.sleep(2.0)
                    continue

                data = self._build_battery_status(self._battery_device, ChargingStatus)
                self._publish_battery_json(data)
                self._publish_battery_state(data)
                self._update_face_battery_params(data)
                self._check_battery_speech_triggers(data)
            except asyncio.CancelledError:
                break
            except Exception as exc:
                self.get_logger().error(f"Battery monitor error: {exc}")
                await asyncio.sleep(5.0)

            await asyncio.sleep(max(0.5, self.battery_poll_interval))

        self._publish_battery_json({"status": "disconnected"})
        await self._disconnect_battery()

    def _load_solix_support(self):
        try:
            from bleak import BleakScanner
            from bleak.backends.device import BLEDevice
            from SolixBLE import C300, ChargingStatus
            import SolixBLE.const
            from SolixBLE.device import SolixBLEDevice
            from SolixBLE.devices.c300 import C300 as OriginalC300
        except Exception as exc:
            self.get_logger().error(
                "Battery BLE client requires 'bleak' and 'SolixBLE'. "
                f"Error: {exc}"
            )
            return None

        SolixBLE.const.RECONNECT_ATTEMPTS_MAX = 0
        self._patch_bleak_bluez_manager()
        if self.battery_plaintext:
            self._patch_solix_c300(SolixBLE, SolixBLEDevice, OriginalC300)
        return C300, ChargingStatus, BleakScanner, BLEDevice

    def _patch_bleak_bluez_manager(self):
        try:
            from bleak.backends.bluezdbus.manager import BlueZManager
        except Exception as exc:
            self.get_logger().debug(f"Bleak BlueZ manager patch skipped: {exc}")
            return

        if getattr(BlueZManager, "_sirius_missing_device_patch_applied", False):
            return

        original_parse_msg = BlueZManager._parse_msg

        def patched_parse_msg(manager_self, message):
            try:
                return original_parse_msg(manager_self, message)
            except KeyError as exc:
                if exc.args == ("Device",):
                    return None
                raise

        BlueZManager._parse_msg = patched_parse_msg
        BlueZManager._sirius_missing_device_patch_applied = True
        self.get_logger().debug("Applied Bleak BlueZ missing Device compatibility patch")

    def _patch_solix_c300(self, SolixBLE, SolixBLEDevice, OriginalC300):
        import hashlib

        if getattr(SolixBLEDevice, "_sirius_c300_patch_applied", False):
            return

        original_reset_session = SolixBLEDevice._reset_session

        def patched_reset_session(device_self, reset_data=True):
            original_reset_session(device_self, reset_data)
            device_self._c300_negotiated = False

        original_encrypt_payload = SolixBLEDevice._encrypt_payload

        def patched_encrypt_payload(device_self, payload: bytes) -> bytes:
            if device_self._shared_secret is None:
                return payload
            return original_encrypt_payload(device_self, payload)

        async def patched_send_encrypted_packet(device_self, cmd: bytes, payload: bytes) -> None:
            packet = device_self._build_packet(bytes.fromhex("03000f"), cmd, payload)
            await device_self._client.write_gatt_char(SolixBLE.const.UUID_COMMAND, packet)

        async def patched_process_telemetry_packet(device_self, payload: bytes, cmd: bytes = None):
            fragment_index = (payload[0] >> 4) & 0x0F
            fragment_total = payload[0] & 0x0F

            if fragment_total > 1:
                fragment_data = payload[1:]
                cmd_key = bytes(cmd)
                if fragment_index > fragment_total:
                    swapped_index = payload[0] & 0x0F
                    swapped_total = (payload[0] >> 4) & 0x0F
                    if swapped_index <= swapped_total and swapped_total > 0:
                        fragment_index = swapped_index
                        fragment_total = swapped_total
                        fragment_data = payload[1:]
                    else:
                        payload = payload[1:]
                        fragment_total = 1

                if cmd_key not in device_self._fragment_buffers or fragment_index == 1:
                    device_self._fragment_buffers[cmd_key] = {}
                    device_self._fragment_totals[cmd_key] = fragment_total

                device_self._fragment_buffers[cmd_key][fragment_index] = fragment_data
                if fragment_total > 1 and len(device_self._fragment_buffers[cmd_key]) < fragment_total:
                    return

                payload = b"".join(
                    device_self._fragment_buffers[cmd_key][i]
                    for i in sorted(device_self._fragment_buffers[cmd_key])
                )
                del device_self._fragment_buffers[cmd_key]
                del device_self._fragment_totals[cmd_key]
            else:
                payload = payload[1:]

            if cmd and cmd.hex() != "8402":
                return
            parameters = device_self._parse_payload(payload)
            return await device_self._process_telemetry(parameters)

        original_process_negotiation = SolixBLEDevice._process_negotiation

        async def patched_process_negotiation(device_self, cmd: bytes, payload: bytes) -> None:
            if cmd.hex() == "0821":
                device_self._c300_negotiated = True
                device_self._shared_secret = hashlib.sha256(b"Solix").digest()
                await device_self._client.write_gatt_char(
                    SolixBLE.const.UUID_COMMAND,
                    bytes.fromhex(SolixBLE.const.NEGOTIATION_COMMAND_5),
                )
                return
            return await original_process_negotiation(device_self, cmd, payload)

        original_process_notification = SolixBLEDevice._process_notification

        async def patched_process_notification(device_self, client, handle, data) -> None:
            if device_self._client is not client:
                return
            try:
                pattern, cmd, payload = device_self._split_packet(data)
                pattern_hex = pattern.hex()
                cmd_hex = cmd.hex()
                if pattern_hex == "030001" and getattr(device_self, "_c300_negotiated", False):
                    return
                if cmd_hex == "8402" or pattern_hex in ("03000f", "030002", "03000c"):
                    return await device_self._process_telemetry_packet(payload, cmd)
                if cmd_hex == "4840":
                    return
            except Exception as exc:
                self.get_logger().debug(f"C300 notification patch fallback: {exc}")
            return await original_process_notification(device_self, client, handle, data)

        async def patched_get_status_update(device_self):
            await device_self._send_command(
                cmd=bytes.fromhex("4040"),
                payload=bytes.fromhex("a10121"),
            )
            return device_self._data or {}

        SolixBLEDevice._reset_session = patched_reset_session
        SolixBLEDevice._encrypt_payload = patched_encrypt_payload
        SolixBLEDevice._send_encrypted_packet = patched_send_encrypted_packet
        SolixBLEDevice._process_telemetry_packet = patched_process_telemetry_packet
        SolixBLEDevice._process_negotiation = patched_process_negotiation
        SolixBLEDevice._process_notification = patched_process_notification
        OriginalC300.get_status_update = patched_get_status_update
        SolixBLEDevice._sirius_c300_patch_applied = True

    async def _resolve_and_create_station(self, C300, BleakScanner, BLEDevice):
        self.get_logger().info(f"Resolving battery BLE device {self.battery_mac}")
        if self.battery_scan_before_connect:
            device = await BleakScanner.find_device_by_address(self.battery_mac, timeout=10.0)
            if device is None:
                raise RuntimeError(f"Battery not found during BLE scan: {self.battery_mac}")
            return C300(device)
        details = {
            "path": f"/org/bluez/hci0/dev_{self.battery_mac.replace(':', '_')}",
            "props": {
                "Address": self.battery_mac,
                "AddressType": "public",
                "Name": "Anker SOLIX C300",
                "Alias": "Anker SOLIX C300",
                "Paired": False,
                "Bonded": False,
                "Trusted": False,
                "Blocked": False,
                "LegacyPairing": False,
                "RSSI": -99,
                "Connected": False,
                "UUIDs": [],
                "Adapter": "/org/bluez/hci0",
                "ServicesResolved": False,
            },
        }
        ble_device = BLEDevice(
            address=self.battery_mac,
            name="Anker SOLIX C300",
            details=details,
            rssi=-99,
        )
        return C300(ble_device)

    def _build_battery_status(self, power_station, ChargingStatus) -> dict:
        percentage = getattr(power_station, "battery_percentage", -1)
        charging_status = getattr(power_station, "charging_status", None)
        status_str = str(charging_status)
        if "." in status_str:
            status_str = status_str.split(".")[-1]

        is_charging = False
        if charging_status == ChargingStatus.CHARGING:
            is_charging = True
        elif hasattr(ChargingStatus, "BOTH") and charging_status == ChargingStatus.BOTH:
            is_charging = True
        elif type(charging_status).__name__ == "ChargingStatusF3800":
            is_charging = getattr(charging_status, "name", "") == "BOTH"

        time_rem = float(getattr(power_station, "time_remaining", 0.0) or 0.0)
        return {
            "status": "connected" if percentage >= 0 else "connecting",
            "battery_level": float(percentage),
            "charging_status": status_str,
            "is_charging": is_charging,
            "total_input": float(getattr(power_station, "power_in", 0.0) or 0.0),
            "total_output": float(getattr(power_station, "power_out", 0.0) or 0.0),
            "temperature": float(getattr(power_station, "temperature", 0.0) or 0.0),
            "time_remaining": time_rem,
            "time_remaining_str": self._format_remaining_time(time_rem),
            "stamp": time.time(),
        }

    def _format_remaining_time(self, hours_float: float) -> str:
        if hours_float <= 0 or hours_float >= 200:
            return ""
        hours = int(hours_float)
        minutes = int(round((hours_float - hours) * 60))
        if hours > 0 and minutes > 0:
            return f"{hours}時間{minutes}分"
        if hours > 0:
            return f"{hours}時間"
        return f"{minutes}分"

    def _publish_battery_json(self, data: dict):
        self._last_battery_data = data
        msg = String()
        msg.data = json.dumps(data, ensure_ascii=False)
        self.battery_json_pub.publish(msg)

        if hasattr(self, "_remote_server") and self._remote_server:
            try:
                char = self._remote_server.get_characteristic(self.characteristic_uuid)
                if char:
                    status_payload = {
                        "type": "status",
                        "status": "connected",
                        "emergency_stop": getattr(self, '_emergency_stop_active', False),
                        "battery": data,
                        **self._mode_status_fields(),
                        **self._people_status_fields(),
                    }
                    char.value = json.dumps(status_payload, ensure_ascii=False).encode("utf-8")
            except Exception:
                pass

    def _publish_battery_state(self, data: dict):
        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_UNKNOWN
        if data.get("is_charging"):
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_CHARGING
        elif data.get("status") == "connected":
            msg.power_supply_status = BatteryState.POWER_SUPPLY_STATUS_DISCHARGING
        msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
        level = float(data.get("battery_level", -1.0))
        msg.percentage = level / 100.0 if level >= 0 else math.nan
        msg.voltage = math.nan
        msg.current = math.nan
        msg.temperature = float(data.get("temperature", math.nan))
        msg.present = data.get("status") == "connected"
        self.battery_state_pub.publish(msg)

    def _update_face_battery_params(self, data: dict):
        if not self.publish_face_battery_params or data.get("status") != "connected":
            return
        now = time.time()
        if now - self._last_face_battery_update < 2.0:
            return
        self._last_face_battery_update = now

        def worker():
            try:
                import grpc
                import face_control_pb2
                import face_control_pb2_grpc

                charging_value = 1.0 if data.get("is_charging") else 2.0
                with grpc.insecure_channel(self.face_status_grpc_target) as channel:
                    stub = face_control_pb2_grpc.FaceServiceStub(channel)
                    stub.UpdateParameters(
                        face_control_pb2.ParameterRequest(
                            values={
                                "batteryStatus": 1.0,
                                "batteryLevel": float(data.get("battery_level", -1.0)),
                                "batteryCharging": charging_value,
                                "batteryInput": float(data.get("total_input", 0.0)),
                                "batteryOutput": float(data.get("total_output", 0.0)),
                                "batteryTemperature": float(data.get("temperature", 0.0)),
                                "batteryTimeRemaining": float(data.get("time_remaining", 0.0)),
                            }
                        ),
                        timeout=1.0,
                    )
            except Exception as exc:
                self.get_logger().debug(f"Failed to update face battery params: {exc}")

        threading.Thread(target=worker, daemon=True).start()


def main(args=None):
    rclpy.init(args=args)
    node = SiriusBleGateway()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
