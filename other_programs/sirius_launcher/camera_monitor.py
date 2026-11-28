"""Read-only health monitoring of the compressed THETA and ZED streams."""
from io import BytesIO
import time
from zlib import crc32

from PIL import Image


CAMERA_TOPICS = {
    'theta': '/theta/dual_fisheye/image_raw/compressed',
    'zed': '/camera/stereo_sbs/compressed',
}
FRAME_TIMEOUT = 4
SAMPLE_PERIOD = 1
FROZEN_AFTER = 5

# Thresholded 48×16 mask of the lower "No Signal" text observed on the
# UGREEN 25854. Matching the text avoids treating a dark room as signal loss.
UGREEN_MASK = bytes.fromhex("""
    000000000000
    000000000000
    000000000000
    000000000000
    011006000040
    019008000040
    01d3081ee640
    01548ef6a140
    017482f6b740
    013483f6bd40
    01378edebf40
    000000060000
    0000001c0000
    000000000000
    000000000000
    000000000000
""")
UGREEN_MASK_SIZE = (48, 16)
UGREEN_MASK_PIXELS = tuple(bool(byte & (128 >> bit)) for byte in UGREEN_MASK for bit in range(8))


def ugreen_no_signal(data):
    with Image.open(BytesIO(data)) as image:
        gray = image.convert('L')
        w, h = gray.size
        crop = gray.crop(tuple(round(x) for x in (w*.40, h*.58, w*.60, h*.69)))
        detected = [value >= 85 for value in crop.resize(UGREEN_MASK_SIZE, Image.Resampling.BOX).getdata()]
        intersection = sum(a and b for a, b in zip(UGREEN_MASK_PIXELS, detected))
        union = sum(a or b for a, b in zip(UGREEN_MASK_PIXELS, detected))
        if intersection < .80 * union:
            return False
        thumb = gray.resize((48, 27), Image.Resampling.BOX)
        return sum(thumb.histogram()[:22]) >= .88 * thumb.width * thumb.height


class CameraFeed:
    def __init__(self, topic, check_signal=False):
        self.topic = topic
        self.check_signal = check_signal
        self.reset()

    def reset(self):
        self.connected_at = self.received_at = self.sampled_at = self.changed_at = None
        self.signature = None
        self.no_signal_samples = 0
        self.invalid = False

    def ingest(self, data, now):
        self.received_at = now
        if self.sampled_at is not None and now - self.sampled_at < SAMPLE_PERIOD:
            return
        self.sampled_at = now
        if not data:
            self.invalid = True
            self.signature = None
            self.no_signal_samples = 0
            return
        signature = crc32(data)
        if signature != self.signature:
            self.signature = signature
            self.changed_at = now
            self.invalid = False
            try:
                self.no_signal_samples = self.no_signal_samples + 1 if self.check_signal and ugreen_no_signal(data) else 0
            except (OSError, ValueError, Image.DecompressionBombError):
                self.invalid = True
                self.no_signal_samples = 0
                return
        elif self.no_signal_samples:
            self.no_signal_samples += 1

    def snapshot(self, now, publishers):
        age = None if self.received_at is None else now - self.received_at
        if not publishers:
            self.reset()
            state = 'off'
        else:
            if self.connected_at is None:
                self.connected_at = now
            if age is None:
                state = 'waiting' if now - self.connected_at < FRAME_TIMEOUT else 'stalled'
            elif age >= FRAME_TIMEOUT:
                state = 'stalled'
            elif self.invalid:
                state = 'invalid'
            elif self.no_signal_samples >= 2:
                state = 'no_signal'
            elif self.no_signal_samples:
                state = 'waiting'
            elif self.changed_at is not None and now - self.changed_at >= FROZEN_AFTER:
                state = 'frozen'
            else:
                state = 'live'
        return {'state': state, 'topic': self.topic, 'age': age}


class CameraReceiver:
    def __init__(self, node):
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import CompressedImage

        self.node = node
        self.feeds = {name: CameraFeed(topic, check_signal=name == 'theta')
                      for name, topic in CAMERA_TOPICS.items()}

        def callback(feed):
            return lambda message: feed.ingest(message.data, time.monotonic())

        self.subscriptions = {
            name: node.create_subscription(CompressedImage, feed.topic, callback(feed), qos_profile_sensor_data)
            for name, feed in self.feeds.items()
        }

    def snapshot(self, now):
        return {name: feed.snapshot(now, self.subscriptions[name].get_publisher_count())
                for name, feed in self.feeds.items()}
