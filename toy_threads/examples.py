import rclpy
from toy_threads.multitimer import MultiTimer


def single_thread():
    rclpy.init()
    node = MultiTimer({'computation': (1.0, 0.25)})
    rclpy.spin(node)


def two_competiting_threads():
    rclpy.init()
    node = MultiTimer({'computation': (1.0, 0.25),
                       'publisher': (0.1, 0.01)})
    rclpy.spin(node)


def multithread():
    rclpy.init()
    node = MultiTimer({'computation': (1.0, 0.25),
                       'publisher': (0.1, 0.01)})
    executor = rclpy.executors.MultiThreadedExecutor(4)
    executor.add_node(node)
    executor.spin()


def multithread2():
    rclpy.init()
    node = MultiTimer({'computation': (1.0, 0.25),
                       'publisher': (0.1, 0.01)},
                      rclpy.callback_groups.ReentrantCallbackGroup())
    executor = rclpy.executors.MultiThreadedExecutor(4)
    executor.add_node(node)
    executor.spin()


def two_competiting_threads2():
    rclpy.init()
    node = MultiTimer({'computation': (1.0, 0.25),
                       'publisher': (0.1, 0.01)},
                      rclpy.callback_groups.ReentrantCallbackGroup())
    rclpy.spin(node)
