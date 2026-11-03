"""Send the single StartSignal to the referee, then exit.

    ros2 run task_solution start_run               # typed command
    ros2 run task_solution start_run --voice go    # voice start (go / allez / vai)

The referee's clock starts when it receives this, and /parc/run_status goes
to RUNNING. The solution nodes only move once they see RUNNING.
"""

import argparse
import sys

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.utilities import remove_ros_args

from parc_msgs.msg import RunStatus, StartSignal

from task_solution.referee import (EVENTS, LATCHED, START_WORDS, STATE_NAMES,
                                   TOPIC_RUN_STATUS, TOPIC_START, make_start)


def main(args=None):
    parser = argparse.ArgumentParser(description='Send the PARC StartSignal once.')
    parser.add_argument('--voice', choices=START_WORDS,
                        help='mark as a voice start with this word')
    parser.add_argument('--timeout', type=float, default=10.0,
                        help='seconds to wait for the referee')
    parser.add_argument('--force', action='store_true',
                        help='send even if the referee is not ARMED')
    opts = parser.parse_args(remove_ros_args(args if args is not None else sys.argv)[1:])

    rclpy.init(args=args)
    node = Node('start_run')
    log = node.get_logger()
    status = {'state': None}
    node.create_subscription(
        RunStatus, TOPIC_RUN_STATUS,
        lambda m: status.update(state=m.state), LATCHED)
    pub = node.create_publisher(StartSignal, TOPIC_START, EVENTS)

    code = 1
    try:
        deadline = node.get_clock().now() + Duration(seconds=opts.timeout)
        while rclpy.ok() and node.get_clock().now() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if pub.get_subscription_count() > 0 and status['state'] is not None:
                break

        state = status['state']
        if pub.get_subscription_count() == 0:
            log.error('No referee is listening on /parc/start. Not sending.')
        elif state != RunStatus.ARMED and not opts.force:
            log.error(f'Referee is {STATE_NAMES.get(state, state)}, not ARMED. '
                      'Not sending (use --force to override).')
        else:
            pub.publish(make_start(node, opts.voice))
            pub.wait_for_all_acked(Duration(seconds=2.0))
            log.info('StartSignal sent' + (f' (voice: "{opts.voice}")' if opts.voice else '.'))
            code = 0
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
