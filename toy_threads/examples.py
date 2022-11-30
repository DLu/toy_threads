import rclpy
from toy_threads.multitimer import MultiTimer


def single_thread():
    rclpy.init()
    node = MultiTimer({'computation': (1.0, 0.25)})
    rclpy.spin(node)


def two_competing_tasks():
    rclpy.init()
    node = MultiTimer({'computation': (1.0, 0.25),
                       'publisher': (0.1, 0.01)})
    rclpy.spin(node)


def multithread():
    rclpy.init()
    node = MultiTimer({'computation': (1.0, 0.25),
                       'publisher': (0.1, 0.01)})
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()


def multithread_with_reentrant():
    rclpy.init()
    node = MultiTimer({'computation': (1.0, 0.25),
                       'publisher': (0.1, 0.01)},
                      rclpy.callback_groups.ReentrantCallbackGroup())
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()


def multithread_with_mux():
    rclpy.init()
    node = MultiTimer({'computation': (1.0, 0.25),
                       'publisher': (0.1, 0.01)},
                      {'computation': rclpy.callback_groups.MutuallyExclusiveCallbackGroup(),
                       'publisher': rclpy.callback_groups.MutuallyExclusiveCallbackGroup(),
                       })
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()


def single_thread_with_reentrant():
    rclpy.init()
    node = MultiTimer({'computation': (1.0, 0.25),
                       'publisher': (0.1, 0.01)},
                      rclpy.callback_groups.ReentrantCallbackGroup())
    rclpy.spin(node)
