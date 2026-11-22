from setuptools import find_packages, setup

package_name = 'my_robot_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Svetlana Shevchenko',
    maintainer_email='svshevchenko@edu.aau.at',
    description='GPS sensor-processing nodes (topics, services, actions)',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        'publisher_node = my_robot_package.publisher_node:main',
        'subscriber_node = my_robot_package.subscriber_node:main',
        'service_node = my_robot_package.service_node:main',
        'client_node = my_robot_package.client_node:main',
        'action_server_node = my_robot_package.action_server_node:main',
        'action_client_node = my_robot_package.action_client_node:main',
        ],
    },
)
