from setuptools import setup

package_name = 'supermarket_capture'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/capture.yaml']),
        ('share/' + package_name + '/launch', ['launch/capture.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'scheduler_node = supermarket_capture.scheduler_node:main',
        ],
    },
)
