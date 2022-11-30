from setuptools import setup

package_name = 'toy_threads'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='David V. Lu!!',
    maintainer_email='davidvlu@gmail.com',
    description='Some toy examples to demonstrate threading in rclpy',
    license='BSD',
    entry_points={
        'console_scripts': [
            'single_thread = toy_threads.examples:single_thread',
            'two_competing_tasks = toy_threads.examples:two_competing_tasks',
            'multithread = toy_threads.examples:multithread',
            'multithread_with_reentrant = toy_threads.examples:multithread_with_reentrant',
            'multithread_with_mux = toy_threads.examples:multithread_with_mux',
            'single_thread_with_reentrant = toy_threads.examples:single_thread_with_reentrant',
        ]
    },
)
