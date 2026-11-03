"""Interface to the PARC 2026 referee (see "PARC 2026 - Referee Topics").

Use one RefereeClient per solution, inside the node that drives the robot:

    self.referee = RefereeClient(self, on_order=..., on_status=...)
    if self.referee.running: ...           # only move while RUNNING
    self.referee.report_tag(7, image_msg)  # stopped in the visit pose
    self.referee.finish()                  # only if the last tag cannot be reported

Rules this class enforces:
  * the five /parc/ topics and their QoS are created here and nowhere else;
  * StartSignal is sent at most once;
  * each tag is reported at most once, and only while the run is RUNNING.
Holding still for 3 s after report_tag() is the caller's job (see visit_hold_s).
"""

import cv2
from cv_bridge import CvBridge
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image

from parc_msgs.msg import Finish, RunStatus, StartSignal, TagOrder, TagVisit

# /parc/tag_order and /parc/run_status
LATCHED = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL)
# /parc/start, /parc/tag_visit and /parc/finish
EVENTS = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

TOPIC_TAG_ORDER = '/parc/tag_order'
TOPIC_START = '/parc/start'
TOPIC_TAG_VISIT = '/parc/tag_visit'
TOPIC_FINISH = '/parc/finish'
TOPIC_RUN_STATUS = '/parc/run_status'

START_WORDS = ('go', 'allez', 'vai')
END_STATES = (RunStatus.FINISHED, RunStatus.TIMED_OUT, RunStatus.TERMINATED)
STATE_NAMES = {
    RunStatus.IDLE: 'IDLE',
    RunStatus.ARMED: 'ARMED',
    RunStatus.RUNNING: 'RUNNING',
    RunStatus.FINISHED: 'FINISHED',
    RunStatus.TIMED_OUT: 'TIMED_OUT',
    RunStatus.TERMINATED: 'TERMINATED',
}

# Minimum time to hold still in the visit pose after publishing a TagVisit.
VISIT_HOLD_S = 3.0


class RefereeClient:

    def __init__(self, node, on_order=None, on_status=None, jpeg_quality=90):
        """on_order(list[int]) runs when a new tag order arrives.
        on_status(RunStatus, old_state) runs on every status message."""
        self.node = node
        self.log = node.get_logger()
        self.on_order = on_order
        self.on_status = on_status
        self.jpeg_quality = int(jpeg_quality)
        self.bridge = CvBridge()

        self.tag_order = []          # list[int], in the order to visit
        self.status = None           # last RunStatus, None until the referee is heard
        self.start_sent = False
        self.reported = set()        # tag ids we have published a TagVisit for

        node.create_subscription(TagOrder, TOPIC_TAG_ORDER, self._order_cb, LATCHED)
        node.create_subscription(RunStatus, TOPIC_RUN_STATUS, self._status_cb, LATCHED)
        self.start_pub = node.create_publisher(StartSignal, TOPIC_START, EVENTS)
        self.visit_pub = node.create_publisher(TagVisit, TOPIC_TAG_VISIT, EVENTS)
        self.finish_pub = node.create_publisher(Finish, TOPIC_FINISH, EVENTS)

    # ---------------------------------------------------------------- state
    @property
    def state(self):
        return None if self.status is None else self.status.state

    @property
    def running(self):
        """True only while the clock runs; the robot must not move otherwise."""
        return self.state == RunStatus.RUNNING

    @property
    def ended(self):
        return self.state in END_STATES

    @property
    def remaining_tags(self):
        """Tags in the order that have not been reported yet."""
        return [t for t in self.tag_order if t not in self.reported]

    @property
    def time_left_s(self):
        if self.status is None:
            return None
        return float(self.status.time_limit_s) - float(self.status.elapsed_s)

    # ------------------------------------------------------------ callbacks
    def _order_cb(self, msg):
        order = list(msg.tag_ids)
        if order != self.tag_order:
            self.log.info(f'Tag order received: {order}')
            # A new order means a new run: forget earlier reports.
            self.reported.clear()
            self.start_sent = False
        self.tag_order = order
        if self.on_order is not None:
            self.on_order(order)

    def _status_cb(self, msg):
        old = self.state
        self.status = msg
        if old != msg.state:
            self.log.info(
                f'Run status: {STATE_NAMES.get(old, old)} -> '
                f'{STATE_NAMES.get(msg.state, msg.state)}')
            if msg.state == RunStatus.IDLE:
                self.reported.clear()
                self.start_sent = False
        if msg.false_start:
            self.log.error('Referee flagged a FALSE START.',
                           throttle_duration_sec=5.0)
        # The order is also in the status; use it if /parc/tag_order was missed.
        if not self.tag_order and len(msg.tag_order) > 0:
            self._order_cb(TagOrder(stamp=msg.stamp, tag_ids=list(msg.tag_order)))
        if self.on_status is not None:
            self.on_status(msg, old)

    # --------------------------------------------------------------- output
    def send_start(self, heard=None):
        """Publish the single StartSignal. heard='go'/'allez'/'vai' marks a voice start."""
        if self.start_sent:
            self.log.warn('StartSignal already sent; ignoring.')
            return False
        if self.state not in (RunStatus.ARMED, None):
            self.log.warn(
                f'Sending StartSignal while referee is '
                f'{STATE_NAMES.get(self.state, self.state)} (expected ARMED).')
        self.start_pub.publish(make_start(self.node, heard))
        self.start_sent = True
        self.log.info('StartSignal sent' + (f' (voice: "{heard}")' if heard else '.'))
        return True

    def report_tag(self, tag_id, image):
        """Publish one TagVisit. Call while stopped in the visit pose, then hold
        still for VISIT_HOLD_S. image: sensor_msgs Image or CompressedImage (JPEG).
        Returns True if the visit was published."""
        tag_id = int(tag_id)
        if not self.running:
            self.log.warn(f'Not reporting tag {tag_id}: run is not RUNNING.')
            return False
        if tag_id in self.reported:
            self.log.warn(f'Tag {tag_id} already reported; not sending again.')
            return False
        if tag_id not in self.tag_order:
            self.log.warn(f'Tag {tag_id} is not in the order {self.tag_order}; not reporting.')
            return False

        jpeg = self.to_jpeg(image)
        msg = TagVisit()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = jpeg.header.frame_id
        msg.tag_id = tag_id
        msg.image = jpeg
        self.visit_pub.publish(msg)
        self.reported.add(tag_id)
        self.log.info(
            f'Reported tag {tag_id} ({len(self.reported)}/{len(self.tag_order)}). '
            f'Hold still for {VISIT_HOLD_S:.0f} s.')
        return True

    def finish(self, completed=False):
        """End the run early. Not needed if the last tag in the order was reported:
        the referee then ends the run by itself."""
        if self.ended:
            return False
        msg = Finish()
        msg.stamp = self.node.get_clock().now().to_msg()
        msg.reason = Finish.REASON_COMPLETED if completed else Finish.REASON_GAVE_UP
        self.finish_pub.publish(msg)
        self.log.info('Finish sent (' + ('completed' if completed else 'gave up') + ').')
        return True

    def to_jpeg(self, image):
        frame_id = ''  # stays empty for a raw OpenCV array
        if isinstance(image, CompressedImage):
            if 'jp' in image.format.lower():
                return image
            frame_id = image.header.frame_id
            image = self.bridge.compressed_imgmsg_to_cv2(image, 'bgr8')
        elif isinstance(image, Image):
            frame_id = image.header.frame_id
            image = self.bridge.imgmsg_to_cv2(image, 'bgr8')
        ok, buf = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
        if not ok:
            raise RuntimeError('JPEG encoding failed')
        out = CompressedImage()
        out.header.stamp = self.node.get_clock().now().to_msg()
        out.header.frame_id = frame_id
        out.format = 'jpeg'
        out.data = buf.tobytes()
        return out


def make_start(node, heard=None):
    msg = StartSignal()
    msg.stamp = node.get_clock().now().to_msg()
    if heard:
        msg.method = StartSignal.METHOD_VOICE
        msg.heard = heard
    else:
        msg.method = StartSignal.METHOD_COMMAND
    return msg
