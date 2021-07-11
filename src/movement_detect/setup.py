from setuptools import setup

package_name = 'movement_detect'

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
    maintainer='rockey',
    maintainer_email='rockey@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['movement_detector_mpo = movement_detect.movement_detector_mpo:main',
                            'store_json = movement_detect.store_json:main',
                            'store_goal_waypoint = movement_detect.store_goal_waypoint:main',
        ],
    },
)
