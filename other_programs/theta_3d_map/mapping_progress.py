"""Periodic, line-based progress suitable for both terminals and Web logs."""
import threading
import time


def elapsed_text(seconds):
    seconds = int(seconds)
    minutes, seconds = divmod(seconds, 60)
    hours, minutes = divmod(minutes, 60)
    return f'{hours:02d}:{minutes:02d}:{seconds:02d}'


class MappingProgress:
    def __init__(self, interval=3.):
        self.interval = interval
        self.lock = threading.Lock()
        self.stop = threading.Event()
        self.started = time.monotonic()
        self.stage_started = self.started
        self.stage = 0
        self.title = '準備中'
        self.total = None
        self.current = self.evaluations = 0
        self.detail = ''
        self.stage_done = False
        self.thread = None

    def __enter__(self):
        self.thread = threading.Thread(target=self._heartbeat, daemon=True)
        self.thread.start()
        return self

    def __exit__(self, kind, error, traceback):
        self.stop.set()
        if self.thread:
            self.thread.join()
        if kind is not None:
            label = '中断' if issubclass(kind, KeyboardInterrupt) else '失敗'
            print(f'【{label}】{self.title}: {error}', flush=True)

    def _heartbeat(self):
        while not self.stop.wait(self.interval):
            with self.lock:
                if self.stage and not self.stage_done:
                    self._print()

    def begin(self, stage, title, total=None, detail=''):
        with self.lock:
            self.stage, self.title, self.total = stage, title, total
            self.current = self.evaluations = 0
            self.detail, self.stage_done = detail, False
            self.stage_started = time.monotonic()
            self._print()

    def update(self, current, detail=''):
        with self.lock:
            self.current = current
            self.detail = detail

    def evaluated(self):
        with self.lock:
            self.evaluations += 1

    def finish_stage(self, detail=''):
        with self.lock:
            self.stage_done = True
            if self.total is not None:
                self.current = self.total
            self.detail = detail or self.detail
            self._print()

    def complete(self, output, points, warning=False):
        self.stop.set()
        with self.lock:
            self.stage_done = True
            label = '保存完了・要確認' if warning else '完了'
            print(f'\n【{label}】3D地図・色付き3D地図を保存しました（{points:,}点）\n'
                  f'総経過時間: {elapsed_text(time.monotonic()-self.started)}\n'
                  f'保存先: {output}\n'
                  '次の操作: view_3dmap で今回のBAGの地図を選択してください。', flush=True)
            if warning:
                print('要確認: 最適化が収束する前に停止しました。report.jsonと地図を確認してください。', flush=True)

    def _print(self):
        # Called with the lock held: a heartbeat cannot mix different stages.
        if self.total is not None and self.total > 0:
            fraction = min(1., max(0., self.current / self.total))
            filled = int(fraction * 20)
            status = f'[{"#"*filled}{"-"*(20-filled)}] {fraction:5.1%} ({self.current}/{self.total})'
        else:
            status = '工程完了' if self.stage_done else '処理中…'
        if self.evaluations:
            status += f' | 最適化の残差評価 {self.evaluations:,}回'
        timing = elapsed_text(time.monotonic()-self.stage_started)
        print(f'[{self.stage}/8] {self.title} {status} | 経過 {timing}'
              + (f' | {self.detail}' if self.detail else ''), flush=True)
