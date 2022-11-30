import threading
from rclpy.node import Node
from toy_threads.timeline import Timeline


class MultiTimer(Node):
    def __init__(self, threads, override_cb_group=None):
        super().__init__('multitimer')
        self.timeline = Timeline(list(threads.keys()))

        self.t0 = self.get_clock().now()
        self.cvs = {}
        for name, (period, duration) in threads.items():
            if override_cb_group is None:
                cb_group = None
            elif isinstance(override_cb_group, dict):
                cb_group = override_cb_group.get(name)
            else:
                cb_group = override_cb_group

            self.cvs[name] = threading.Condition()

            self.create_timer(period,
                              lambda name=name, duration=duration:
                              self.timer_cb(name, duration),
                              callback_group=cb_group)

    def get_t(self):
        return (self.get_clock().now() - self.t0).nanoseconds / 1e9

    def timer_cb(self, name, duration):
        # Mark the beginning
        self.timeline.start(name, self.get_t())

        # Wait for duration (real) seconds
        with self.cvs[name]:
            threading.Timer(duration, self.end_cb, args=(name,)).start()
            self.cvs[name].wait()

        # Mark the end
        self.timeline.end(name, self.get_t())

    def end_cb(self, name):
        with self.cvs[name]:
            self.cvs[name].notify()
