"""SCN velocity keys and a renewable browser control session; no ROS imports."""
import math


class VelocityControl:
    LINEAR_STEP, ANGULAR_STEP = .1, .157
    LINEAR_MAX, ANGULAR_MAX = 2., 3.14
    LINEAR_ACCEL, ANGULAR_ACCEL = .8, 1.
    TIMEOUT = 1.
    KEYS = {65, 66, 67, 68, 44, 46}

    def __init__(self):
        self.owner = None
        self.last_heartbeat = 0.
        self.target_linear = self.target_angular = 0.
        self.linear = self.angular = 0.
        self.reason = '操作は無効です'
        self.sequences = {}

    def stop(self, reason='停止しました'):
        self.owner = None
        self.target_linear = self.target_angular = 0.
        self.linear = self.angular = 0.
        self.reason = reason

    def command(self, action, session, sequence, now, key=None, blocked=''):
        if (not isinstance(session, str) or not 16 <= len(session) <= 80
                or type(sequence) is not int or sequence < 1):
            return 'invalid session or sequence'
        if action not in {'arm', 'heartbeat', 'key', 'disable', 'stop'}:
            return 'invalid action'
        if action == 'key' and (type(key) is not int or key not in self.KEYS):
            return 'invalid key'
        if sequence <= self.sequences.get(session, 0):
            return '古い操作を破棄しました'
        self.sequences[session] = sequence
        if len(self.sequences) > 128:
            oldest = next(k for k in self.sequences if k != self.owner)
            self.sequences.pop(oldest)
        if action == 'stop':
            if self.owner == session and not blocked:
                # A normal stop zeros both axes while retaining the operator.
                self.target_linear = self.target_angular = self.linear = self.angular = 0.
                self.last_heartbeat, self.reason = now, '停止中・操作は有効です'
            else:
                self.stop(blocked or '停止しました')
            return ''
        if action == 'disable':
            if self.owner == session:
                self.stop('操作を無効にしました')
            return ''
        if blocked:
            if self.owner == session:
                self.stop(blocked)
            return blocked
        if action == 'arm':
            if self.owner and self.owner != session:
                return '別の画面で操作中です'
            self.stop()
            self.owner, self.last_heartbeat, self.reason = session, now, '操作中'
            return ''
        if self.owner != session:
            return '先に操作を有効にしてください'
        self.last_heartbeat = now
        if action == 'key':
            if key == 65:
                self.target_linear = min(self.LINEAR_MAX, self.target_linear+self.LINEAR_STEP)
            elif key == 66:
                self.target_linear = max(-self.LINEAR_MAX, self.target_linear-self.LINEAR_STEP)
            elif key == 68:
                self.target_angular = min(self.ANGULAR_MAX, self.target_angular+self.ANGULAR_STEP)
            elif key == 67:
                self.target_angular = max(-self.ANGULAR_MAX, self.target_angular-self.ANGULAR_STEP)
            elif key == 44:
                self.target_linear = 0.
            elif key == 46:
                self.target_angular = 0.
            self.target_linear = round(self.target_linear, 9)
            self.target_angular = round(self.target_angular, 9)
        return ''

    def tick(self, now, dt, blocked=''):
        if self.owner and (blocked or now-self.last_heartbeat >= self.TIMEOUT):
            self.stop(blocked or '通信が途切れたため停止しました')
        if not self.owner:
            return
        dt = min(.1, max(0., dt))
        def approach(current, target, change):
            delta = target-current
            return target if abs(delta) <= change else current+math.copysign(change, delta)
        self.linear = approach(self.linear, self.target_linear, self.LINEAR_ACCEL*dt)
        self.angular = approach(self.angular, self.target_angular, self.ANGULAR_ACCEL*dt)

    def snapshot(self):
        return {'enabled': self.owner is not None, 'target_linear': self.target_linear,
                'target_angular': self.target_angular, 'linear': self.linear,
                'angular': self.angular, 'reason': self.reason,
                'linear_step': self.LINEAR_STEP, 'angular_step': self.ANGULAR_STEP,
                'linear_max': self.LINEAR_MAX, 'angular_max': self.ANGULAR_MAX,
                'timeout_seconds': self.TIMEOUT}
