from setuptools import find_packages, setup

package_name = 'hello_robot'

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
    maintainer='h9',
    maintainer_email='2102337576@qq.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'hello_node = hello_robot.hello_node:main',
            'publisher_node = hello_robot.publisher_node:main',
            'subscriber_node = hello_robot.subscriber_node:main',
            'velocity_publisher = hello_robot.velocity_publisher:main',
        ],
    },
)
