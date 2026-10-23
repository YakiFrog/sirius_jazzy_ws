"""Observe foreground terminal reads without consuming input or tracing processes."""
import os
import platform
import re
import time
from pathlib import Path

import psutil

READ_CALLS = {'x86_64': {0, 19}, 'aarch64': {63, 65}}.get(platform.machine(), set())
POLL_CALLS = {'x86_64': {7, 23, 270, 271}, 'aarch64': {72, 73}}.get(platform.machine(), set())
PROMPT = re.compile(r'入力|選択|番号|続行|地図名|保存.*[？?]|\b(?:input|enter|choose|select|password)\b|y/n|n/y', re.IGNORECASE)
ANSI = re.compile(r'\x1b\[[0-?]*[ -/]*[@-~]')


def prompt_tail(path):
    try:
        with open(path, 'rb') as stream:
            stream.seek(0, os.SEEK_END)
            stream.seek(max(0, stream.tell()-2048))
            text = ANSI.sub('', stream.read().decode('utf-8', 'replace'))
        return next((line.strip()[:180] for line in reversed(text.replace('\r', '\n').splitlines())
                     if line.strip()), '')
    except OSError:
        return ''


def inspect_terminal_reader(pid, prompt=''):
    base = Path('/proc')/str(pid)
    try:
        stdin = os.readlink(base/'fd/0')
        if not (stdin.startswith('/dev/pts/') or re.fullmatch(r'/dev/tty\d*', stdin)):
            return None
        fields = (base/'stat').read_text().rsplit(')', 1)[1].split()
        if int(fields[2]) != int(fields[5]) or int(fields[5]) <= 0:
            return None  # Only the foreground job can accept terminal input.
        channel = (base/'wchan').read_text().strip()
        if channel in {'0', 'do_wait', 'futex_wait_queue', 'futex_do_wait', 'hrtimer_nanosleep'}:
            return None
        try:
            call = (base/'syscall').read_text().split()
        except (PermissionError, FileNotFoundError):
            call = []  # Reattached sessions may be outside the server's ptrace scope.
        if call and call[0].lstrip('-').isdigit():
            number = int(call[0])
            if number in READ_CALLS and len(call) > 1:
                fd = int(call[1], 0)
                if os.readlink(base/f'fd/{fd}') == stdin:
                    return dict(state='waiting', confirmed=True, pid=pid, reason='terminal_read', prompt=prompt)
                return None  # Reading a pipe, file or sensor is not operator input.
            if number not in POLL_CALLS:
                return None
        if channel in {'n_tty_read', 'tty_read'}:
            return dict(state='waiting', confirmed=True, pid=pid, reason='terminal_read', prompt=prompt)
        if PROMPT.search(prompt) and (channel == 'wait_woken' or (channel.startswith('poll_schedule_timeout')
                and (base/'comm').read_text().strip() in {'bash', 'sh', 'dash'})):
            return dict(state='possible', confirmed=False, pid=pid, reason='terminal_wait', prompt=prompt)
    except (OSError, ValueError, IndexError, ProcessLookupError):
        return None
    return None


class InputWaitMonitor:
    def __init__(self):
        self.cache = {}

    def snapshot(self, pid, log_path, interactive=False):
        key = (pid, str(log_path), interactive)
        cached = self.cache.get(key)
        if cached and time.monotonic()-cached[0] < .5:
            return dict(cached[1])
        result = dict(state='none', confirmed=False, prompt='', interactive=interactive)
        try:
            root = psutil.Process(int(pid))
            prompt = prompt_tail(log_path)
            for process in [root]+root.children(recursive=True):
                found = inspect_terminal_reader(process.pid, prompt)
                if found and (result['state'] == 'none' or found['confirmed']):
                    result.update(found)
                    if found['confirmed']:
                        break
        except (psutil.Error, OSError, TypeError, ValueError):
            result['state'] = 'unknown'
        if len(self.cache) > 256:
            self.cache.clear()
        self.cache[key] = time.monotonic(), dict(result)
        return result
