# toy_threads

Some toy examples to demonstrate threading in `rclpy`.

# tl;dr: To achieve parallel tasks using `rclpy` you must use BOTH a `MultiThreadedExecutor` AND custom Callback Groups!!

### Installation note:
The interactive demos require the [`blessed`](https://github.com/jquast/blessed) library, which can be installed via `pip`.


## Standard Configuration
Using `rclpy.spin(node)`, all of your callbacks (subscriptions, timers, etc) are (by default) handled in a single thread.

If you run `ros2 run toy_threads single_thread` you will see something like this:

```
computation   0.97 Hz | --xxxxx---------------xxxxx---------------xxxxx-------------
```

This a single task (called `computation`) that aims to run at 1Hz and takes 0.25 seconds to run. The `x` characters indicate where the thread is active. The actual frequency is also displayed.

However, running `ros2 run toy_threads two_competing_tasks` shows this:

```
computation   0.95 Hz | --xxxxx---------------xxxxx---------------xxxxx---------------
publisher     8.05 Hz | -------xx-x-x-x-x-x-x------xx-x-x-x-x-x-x------xx-x-x-x-x-x-x-

```
We've now added a `publisher` task that runs at 10Hz and takes 0.01 seconds to run. Without doing anything fancy, the result is still a single thread, and as a result the publisher cannot run while the computation is active.

As a result, the `publisher` does not reach its desired rate of 10Hz.


## MultiThread Executor
Instead of using the standard `spin` method, we can use a `MultiThreadedExecutor` like so:

```python
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
```

You can demonstrate this with `ros2 run toy_threads multithread`

```
computation   0.92 Hz | --xxxxx---------------xxxxx---------------xxxxx------------
publisher     8.09 Hz | -------xx-x-x-x-x-x-x------xx-x-x-x-x-x-x------xx-x-x-x-x-x
```

And we get the same result?! That's not what we really wanted.

The problem is that unless you specify otherwise, the callbacks are added to the default callback group which has the type `MutuallyExclusiveCallbackGroup`. As a result, the callbacks remain mutually exclusive.

### Solution 1: ReentrantCallbackGroup

If we instead construct all our timers with a new `ReentrantCallbackGroup`, the callbacks will run concurrently, demonstrated with `ros2 run toy_threads multithread_with_reentrant`

```
computation   0.95 Hz | --xxxxx---------------xxxxx---------------xxxxx------------
publisher     9.94 Hz | --x-x-x-x-x-x-x-x-x-x-x-x-x-x-x-x-x-x-x-x-x-x-x-x-x-x-x-x-x
```

Now the `publisher` approaches its desired rate of 10Hz.

### Solution 2: Multiple Callback Groups
Instead of having all the callbacks in the same callback group, you can also put them in individual callback groups and then the `MultiThreadedExecutor` will execute them separately. `ros2 run toy_threads multithread_with_mux`

```
computation   0.94 Hz | --xxxxx---------------xxxxx---------------xxxxx-----------
publisher     9.94 Hz | --x-x-x-x-x-x-x-x-x-x-x-x-x-x-x-x-x-x-x-x-x-x-x-x-x-x-x-x-
```

## SingleThread + Reentrant?
You can also (if you really want) use a single thread (`rclpy.spin`) and a `ReentrantCallbackGroup`, but the result is still only one task executing at a time: `ros2 run toy_threads single_thread_with_reentrant`.

```
computation   0.99 Hz | --xxxxx---------------xxxxx---------------xxxxx-----------
publisher     8.07 Hz | -------xx-x-x-x-x-x-x------xx-x-x-x-x-x-x------xx-x-x-x-x-
```
