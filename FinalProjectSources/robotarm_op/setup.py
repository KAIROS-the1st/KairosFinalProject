from setuptools import setup

package_name = 'robotarm_op'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera = robotarm_op.camera_squad:main',
            'video = robotarm_op.image_processing:main',
            'yolo = robotarm_op.robot_control:main',
            # 'talker = robotarm_op.object_tracking_node:main',
            'talker = robotarm_op.tracking_test:main',
            # 'listener = robotarm_op.moving_robot_node:main',
            'listener = robotarm_op.tracking_test3:main',
        ],
    },
)
