"""Shared, bounded host sampling. HTTP requests only read the cached snapshot."""
from collections import deque
import os
import platform
import socket
import threading
import time

import psutil


RELATED = ('/opt/ros/', 'ros2', 'sirius', 'fastlio', 'fast_lio', 'laser_mapping',
           'ekf_node', 'witmotion', 'velodyne', 'theta', 'foxglove', 'rviz',
           'roboteq', 'nav2', 'slam_toolbox', 'gazebo', 'rosbridge')


class ResourceMonitor:
    def __init__(self, interval=1.0, history_size=120):
        self.interval = interval
        self._history = deque(maxlen=history_size)
        self._lock = threading.Lock()
        self._latest = None
        self._thread = None
        self._processes = {}
        self._network = {}
        self._network_at = None
        self._stop = threading.Event()

    def start(self):
        with self._lock:
            if self._thread is None:
                self._thread = threading.Thread(target=self._run, daemon=True,
                                                name='sirius-resource-monitor')
                self._thread.start()

    def close(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2)

    def snapshot(self):
        self.start()
        with self._lock:
            if self._latest is None:
                return {'available': False, 'warming_up': True, 'hostname': socket.gethostname()}
            return {**self._latest, 'available': True, 'history': list(self._history)}

    def _run(self):
        psutil.cpu_percent(interval=None, percpu=True)
        while not self._stop.wait(self.interval):
            try:
                sample = self._collect()
                with self._lock:
                    self._history.append({key: sample[key] for key in
                                          ('timestamp', 'cpu_percent', 'memory_percent')})
                    self._latest = sample
            except (OSError, psutil.Error):
                # Preserve the last reading; its timestamp makes a stale sample visible.
                continue

    def _collect(self):
        now, mono = time.time(), time.monotonic()
        cores = psutil.cpu_percent(interval=None, percpu=True)
        memory, swap = psutil.virtual_memory(), psutil.swap_memory()
        disk = psutil.disk_usage(os.path.expanduser('~'))
        rows, live = [], {}
        for proc in psutil.process_iter(['pid', 'name', 'cmdline', 'create_time',
                                         'memory_info', 'num_threads', 'status']):
            try:
                info = proc.info
                key = (info['pid'], info['create_time'])
                cached = self._processes.get(key)
                cpu = cached.cpu_percent(None) if cached else None
                if cached is None:
                    proc.cpu_percent(None)
                live[key] = cached or proc
                command = ' '.join(info['cmdline'] or [])
                rss = info['memory_info'].rss if info['memory_info'] else 0
                rows.append({'pid': info['pid'], 'name': info['name'] or 'unknown',
                             'command': command[:500], 'cpu_percent': cpu,
                             'memory_bytes': rss, 'memory_percent': rss/memory.total*100,
                             'threads': info['num_threads'], 'status': info['status'],
                             'age_seconds': max(0, now-(info['create_time'] or now)),
                             'related': any(word in command.lower() for word in RELATED)})
            except (psutil.Error, OSError):
                continue
        self._processes = live
        rows.sort(key=lambda row: (row['cpu_percent'] or 0, row['memory_bytes']), reverse=True)
        network = psutil.net_io_counters(pernic=True)
        elapsed = None if self._network_at is None else mono-self._network_at
        interfaces = []
        for name, current in network.items():
            if name == 'lo':
                continue
            previous = self._network.get(name)
            interfaces.append({'name': name,
                'receive_bytes_per_second': None if not previous or not elapsed else
                    max(0, current.bytes_recv-previous.bytes_recv)/elapsed,
                'send_bytes_per_second': None if not previous or not elapsed else
                    max(0, current.bytes_sent-previous.bytes_sent)/elapsed})
        self._network, self._network_at = network, mono
        temperatures = []
        try:
            for name, values in psutil.sensors_temperatures().items():
                for value in values:
                    if value.current is not None:
                        temperatures.append({'name': value.label or name, 'sensor': name, 'celsius': value.current})
        except (AttributeError, OSError, psutil.Error):
            pass
        return {'timestamp': now, 'hostname': socket.gethostname(), 'platform': platform.system(),
                'uptime_seconds': now-psutil.boot_time(), 'logical_cpus': len(cores),
                'cpu_percent': sum(cores)/max(1, len(cores)), 'cpu_cores': cores,
                'load_average': list(os.getloadavg()), 'memory_percent': memory.percent,
                'memory': {'total': memory.total, 'used': memory.total-memory.available,
                           'available': memory.available},
                'swap': {'total': swap.total, 'used': swap.used, 'percent': swap.percent},
                'disk': {'total': disk.total, 'used': disk.used, 'free': disk.free, 'percent': disk.percent},
                'interfaces': interfaces, 'temperatures': temperatures, 'processes': rows,
                'process_cpu_definition': '100% = one logical CPU'}
