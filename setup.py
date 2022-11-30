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
            'two_competiting_threads = toy_threads.examples:two_competiting_threads',
            'multithread = toy_threads.examples:multithread',
            'two_competiting_threads2 = toy_threads.examples:two_competiting_threads2',
            'multithread2 = toy_threads.examples:multithread2',
        ]
    },
)
