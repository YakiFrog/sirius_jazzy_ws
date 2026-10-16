#!/usr/bin/env python3
"""Persistent PTY owner for Web-launched commands; reconnect through a Unix socket."""
import argparse
import fcntl
import json
import os
from pathlib import Path
import pty
import select
import signal
import socket
import struct
import termios
import time


def socket_path(name):
    return f'/tmp/sirius_launcher_{name}.sock'


def session_request(name, data=None):
    """None probes the session; a string writes to its terminal, including control keys."""
    try:
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as connection:
            connection.settimeout(1)
            connection.connect(socket_path(name))
            connection.sendall((json.dumps({'data': data}) + '\n').encode())
            response = b''
            while b'\n' not in response and len(response) < 4096:
                chunk = connection.recv(4096)
                if not chunk:
                    break
                response += chunk
            return json.loads(response)
    except (OSError, ValueError):
        return {'ok': False, 'error': '入力端末がありません。Webからコマンドを再起動してください。'}


def run(name, script):
    path = socket_path(name)
    pid_file = Path(f'/tmp/sirius_launcher_{name}.pid')
    listener = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    # Never replace a live session's endpoint.
    if os.path.exists(path):
        if session_request(name).get('ok'):
            raise RuntimeError('command session already exists')
        os.unlink(path)
    listener.bind(path)
    os.chmod(path, 0o600)
    listener.listen(4)
    pid, master = pty.fork()
    if pid == 0:
        listener.close()
        env = os.environ.copy()
        env['TERM'] = 'xterm-256color'
        try:
            os.execvpe('bash', ['bash', '--noprofile', '--rcfile', script, '-i'], env)
        finally:
            os._exit(127)
    fcntl.ioctl(master, termios.TIOCSWINSZ, struct.pack('HHHH', 30, 120, 0, 0))
    os.set_blocking(master, False)
    pid_file.write_text(str(os.getpid()))
    stopping = False

    def stop(_signum, _frame):
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    pending = bytearray()
    try:
        with open(f'/tmp/sirius_launcher_{name}.log', 'ab', buffering=0) as log:
            while not stopping:
                readable, writable, _ = select.select([master, listener], [master] if pending else [], [], .2)
                if master in readable:
                    try:
                        chunk = os.read(master, 65536)
                    except BlockingIOError:
                        chunk = None
                    except OSError:
                        break
                    if chunk == b'':
                        break
                    if chunk:
                        log.write(chunk)
                if master in writable:
                    try:
                        written = os.write(master, pending)
                        del pending[:written]
                    except BlockingIOError:
                        pass
                    except OSError:
                        break
                if listener in readable:
                    connection, _ = listener.accept()
                    with connection:
                        connection.settimeout(.5)
                        try:
                            raw = b''
                            while b'\n' not in raw and len(raw) <= 65536:
                                part = connection.recv(8192)
                                if not part:
                                    break
                                raw += part
                            body = json.loads(raw)
                            data = body.get('data')
                            if data is not None:
                                if not isinstance(data, str) or len(data.encode('utf-8')) > 4096:
                                    raise ValueError('入力は4096バイトまでです')
                                encoded = data.encode('utf-8')
                                if len(pending) + len(encoded) > 16384:
                                    raise ValueError('入力の処理待ちです')
                                pending.extend(encoded)
                            reply = {'ok': True, 'pid': os.getpid()}
                        except (OSError, ValueError, AttributeError) as error:
                            reply = {'ok': False, 'error': str(error)}
                        try:
                            connection.sendall((json.dumps(reply) + '\n').encode())
                        except OSError:
                            pass
    finally:
        listener.close()
        os.close(master)
        # The PTY child owns a separate session. Also terminate its foreground job.
        try:
            os.killpg(pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
        deadline = time.monotonic() + 2
        while True:
            waited, _ = os.waitpid(pid, os.WNOHANG)
            if waited:
                break
            if time.monotonic() >= deadline:
                try:
                    os.killpg(pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                os.waitpid(pid, 0)
                break
            time.sleep(.02)
        Path(path).unlink(missing_ok=True)
        if pid_file.exists() and pid_file.read_text().strip() == str(os.getpid()):
            pid_file.unlink()
        Path(script).unlink(missing_ok=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--name', required=True)
    parser.add_argument('--script', required=True)
    args = parser.parse_args()
    run(args.name, args.script)
