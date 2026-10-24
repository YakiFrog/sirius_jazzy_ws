"""Run the real driver against a PTY motor controller, on an isolated ROS domain."""
import json
import os
from pathlib import Path
import pty
import re
import select
import signal
import subprocess
import threading
import time
import tty

import rclpy
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from roboteq_ros2_driver.msg import RoboteqDiagnostics


def main():
    assert os.environ.get('ROS_DOMAIN_ID') not in (None, '', '0', '53'), 'Use an isolated domain'
    package = Path(__file__).resolve().parents[1]
    executable = subprocess.check_output(
        ['ros2', 'pkg', 'prefix', 'roboteq_ros2_driver'], text=True).strip()
    master, slave = pty.openpty()
    tty.setraw(slave)
    state = {'voltage': 13.2, 'fault': 0, 'ex': False, 'running': True, 'extended': False}
    commands = []

    def serial_peer():
        pending = b''
        stream = set()
        while state['running']:
            if select.select([master], [], [], .01)[0]:
                pending += os.read(master, 65536)
                while b'\r' in pending:
                    raw, pending = pending.split(b'\r', 1)
                    command = raw.decode(errors='ignore').strip()
                    commands.append((time.monotonic(), command))
                    if command.startswith('# C_'):
                        stream = set(re.findall(r'\?([A-Z]+)', command))
                    elif command.startswith('?'):
                        # Real hardware cancels its periodic stream on a direct query.
                        stream.clear()
                    if command.startswith('!EX'):
                        state['ex'] = True
                    elif command == '!MG':
                        state['ex'] = False
            voltage = round(state['voltage'] * 10)
            fault = state['fault'] | (16 if state['ex'] else 0)
            replies = {'V': f'138:{voltage}:5000', 'FF': str(fault),
                       'CB': '0:0', 'A': '0:0', 'BS': '0:0'}
            if state['extended']:
                replies.update(FS='9', FM='3:128', P='123:-234')
            packet = ''.join(f'{key}={value}\r' for key, value in replies.items() if key in stream)
            if packet:
                os.write(master, packet.encode())

    thread = threading.Thread(target=serial_peer, daemon=True)
    thread.start()
    log = Path('/tmp/sirius-protection-serial.log').open('w')
    process = subprocess.Popen([
        executable + '/lib/roboteq_ros2_driver/roboteq_ros2_driver_node', '--ros-args',
        '--params-file', str(package / 'config/roboteq.yaml'),
        '-p', 'port:=' + os.ttyname(slave),
        '-p', 'pub_odom_tf:=false',
        '-p', 'estop_latch_hardware:=' + os.environ.get('TEST_LATCH', 'true'),
        '-p', 'publish_status:=' + os.environ.get('TEST_PUBLISH_STATUS', 'true')], stdout=log, stderr=log)
    rclpy.init()
    node = rclpy.create_node('protection_serial_test')
    velocity = node.create_publisher(Twist, '/cmd_vel', 10)
    stop = node.create_publisher(Bool, '/stop', 10)
    diagnostics = []
    diagnostic_sub = node.create_subscription(RoboteqDiagnostics, '/roboteq/diagnostics', diagnostics.append, 100)
    command = Twist()
    command.linear.x = .6

    def run(seconds):
        until = time.monotonic() + seconds
        while time.monotonic() < until:
            velocity.publish(command)
            rclpy.spin_once(node, timeout_sec=.02)
            assert process.poll() is None, 'driver exited'

    def nonzero(since):
        return [c for t, c in commands if t >= since and re.match(r'!G [12] -?[1-9][0-9]*$', c)]

    try:
        started = time.monotonic()
        run(2)
        assert velocity.get_subscription_count(), 'driver not discovered'
        assert not nonzero(started), 'startup must stay stopped'
        assert diagnostics and not diagnostics[-1].status_flags_valid, 'absent MD responses are unknown'
        assert not diagnostics[-1].motor_flags_valid_ch2
        state['extended'] = True; run(.3)
        md = diagnostics[-1]
        assert md.status_flags_valid and md.status_flags == 9
        assert md.motor_flags_valid_ch1 and md.motor_flags_valid_ch2
        assert (md.motor_flags_ch1, md.motor_flags_ch2) == (3, 128)
        assert md.motor_flag_labels_ch1 == ['current_limit', 'stall']
        assert md.motor_flag_labels_ch2 == ['fets_off']
        assert md.md_output_valid_ch1 and md.md_output_valid_ch2
        assert (md.md_output_ch1, md.md_output_ch2) == (123, -234)
        assert any({'CB', 'V', 'FF', 'FS', 'FM', 'P'} <= set(re.findall(r'\?([A-Z]+)', c))
                   for _, c in commands if c.startswith('# C_')), 'all queries share one stream'
        assert not any(c.startswith('?') for _, c in commands), 'direct queries cancel streaming'
        stop.publish(Bool(data=False))
        acknowledged = time.monotonic()
        run(.5)
        assert nonzero(acknowledged), 'healthy manual release must work'
        pi = diagnostics[-1]
        assert pi.control_sample_valid
        assert abs(pi.target_speed_right-.6) < 1e-5 and abs(pi.target_speed_left-.6) < 1e-5
        assert abs(pi.p_term_right-120*pi.error_right) < 1e-6
        assert abs(pi.i_term_right-250*pi.integral_right) < 1e-6
        assert abs(pi.output_unlimited_right-pi.feedforward_right-pi.p_term_right-pi.i_term_right) < 1e-6
        assert pi.command_write_ok_ch1 and pi.command_write_ok_ch2
        assert abs(pi.sent_command_ch1-round(pi.output_limited_right)) <= 2
        # A stale response cannot masquerade as a healthy zero.
        state['extended'] = False; run(.6)
        assert not diagnostics[-1].status_flags_valid and not diagnostics[-1].md_output_valid_ch1
        state['extended'] = True
        state.update(voltage=5.0, fault=4)
        run(.4)
        if os.environ.get('TEST_LATCH') == 'false':
            assert diagnostics[-1].hardware_estop and not diagnostics[-1].software_estop
            assert diagnostics[-1].estop_active and not nonzero(time.monotonic() - .1)
            state.update(voltage=13.2, fault=0); run(.4)
            assert not diagnostics[-1].hardware_estop and not diagnostics[-1].software_estop
            stop.publish(Bool(data=True)); run(.2)
            state.update(voltage=5.0, fault=4); run(.2)
            state.update(voltage=13.2, fault=0); run(.3)
            assert diagnostics[-1].software_estop, 'manual stop must survive voltage recovery'
            stop.publish(Bool(data=False)); run(.4)
            assert not diagnostics[-1].software_estop
            stop.publish(Bool(data=True)); run(.2)
            print(json.dumps({'physical_stop_not_latched': True, 'physical_recovery': True,
                              'manual_stop_preserved': True, 'manual_release': True}))
            return
        assert state['ex'], 'voltage/fault must stop controller'
        fault = diagnostics[-1]
        assert fault.last_stop_reason_mask & RoboteqDiagnostics.STOP_LOW_VOLTAGE
        assert fault.last_stop_reason_mask & RoboteqDiagnostics.STOP_MD_FAULT
        assert fault.last_stop_voltage == 5.0 and fault.last_stop_fault_flags == 4
        assert fault.latched_stop_reason_mask & RoboteqDiagnostics.STOP_LOW_VOLTAGE
        assert not fault.control_sample_valid and fault.integral_right == 0
        assert fault.sent_command_ch1 == 0 and fault.sent_command_ch2 == 0
        stopped = time.monotonic()
        for _ in range(3):
            state.update(voltage=13.2, fault=0); run(.3)
            state.update(voltage=5.0, fault=4); run(.3)
        assert not nonzero(stopped), 'voltage recovery must never restart automatically'
        stop.publish(Bool(data=False)); run(.2)
        assert state['ex'] and not nonzero(stopped), 'release with low voltage must be rejected'
        state.update(voltage=13.2, fault=0); run(.3)
        stop.publish(Bool(data=False)); released = time.monotonic(); run(.5)
        assert nonzero(released), 'release after recovery must work'
        stop.publish(Bool(data=True)); run(.2)
        print(json.dumps({'startup_stopped': True, 'fault_latched': True,
            'automatic_restarts': 0, 'low_voltage_release_rejected': True,
            'healthy_manual_release': True, 'extended_md_fields': True, 'pi_terms': True,
            'stop_reasons': True, 'unknown_and_stale_fields': True, 'log': '/tmp/sirius-protection-serial.log'}))
    finally:
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=4)
        except subprocess.TimeoutExpired:
            process.kill(); process.wait()
        state['running'] = False
        thread.join(timeout=1)
        log.close(); os.close(master); os.close(slave)
        node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
