from math import floor
import threading
import time
import blessed

import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Empty


class TimeSimulator(Node):
    def __init__(self):
        super().__init__('time_simulator')
        self.term = blessed.Terminal()
        self.publisher = self.create_publisher(Clock, '/clock', 1)
        self.running = True
        self.interval = 0.01
        self.t = 0.0
        self.thread = threading.Thread(target=self.wall_thread)

    def should_publish(self):
        return True

    def wall_thread(self):
        while self.running:
            if self.should_publish():
                clock = Clock()
                print(self.term.move_x(0) + f'Current Time: {self.t:.1f}', end='', flush=True)
                clock.clock.sec = int(floor(self.t))
                clock.clock.nanosec = int(self.t % 1.0 * 1e9)
                self.publisher.publish(clock)
                self.t += self.interval
            time.sleep(self.interval)
        print()


class WaitingTimeSimulator(TimeSimulator):
    def __init__(self):
        super().__init__()
        self.expected_interval = 1.0
        self.first = True
        self.max_t = self.expected_interval
        self.sub = self.create_subscription(Empty, '/computation', self.callback, 1)

    def callback(self, msg):
        if self.first:
            self.max_t = self.t
            self.first = False

        self.max_t += self.expected_interval

    def should_publish(self):
        if self.first:
            return True

        return self.t < self.max_t


def generic_main(node, args):
    rclpy.init(args=args)
    node.thread.start()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.running = False
    node.thread.join()


def main(args=None):
    generic_main(TimeSimulator(), args)


def waiting_main(args=None):
    generic_main(WaitingTimeSimulator(), args)
