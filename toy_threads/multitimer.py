from rclpy.node import Node
from toy_threads.timeline import Timeline


class MultiTimer(Node):
    def __init__(self, threads, override_cb_group=None):
        super().__init__('multitimer')
        self.timeline = Timeline(list(threads.keys()))

        self.t0 = self.get_clock().now()
        for name, (period, duration) in threads.items():
            if override_cb_group is None:
                cb_group = None
            elif isinstance(override_cb_group, dict):
                cb_group = override_cb_group.get(name)
            else:
                cb_group = override_cb_group

            self.create_timer(period,
                              lambda name=name, duration=duration:
                              self.timer_cb(name, duration),
                              callback_group=cb_group)

    def get_t(self):
        return (self.get_clock().now() - self.t0).nanoseconds / 1e9

    def timer_cb(self, name, duration):
        t = self.get_t()
        self.timeline.start(name, t)

        stop_t = t + duration
        while self.get_t() < stop_t:
            pass

        t = self.get_t()
        self.timeline.end(name, t)
