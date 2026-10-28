from setuptools import find_packages, setup

package_name = 'isaac_eff_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    package_data={
        package_name: [
            'camera_metadata.json',
            'sorting_scene.json',
        ]
    },
    data_files=[
        (
            'share/' + package_name + '/model',
            [
                '../../../models/robots/s40_sharpa/assembly/s40_sharpa_two_cameras.xml'
            ],
        ),
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        (
            'share/' + package_name,
            ['package.xml'],
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lab',
    maintainer_email='lab@example.com',
    description='Isaac Sim effector and arm control nodes',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'eff_control_node = isaac_eff_control.eff_control_node:main',
            'arm_control_node = isaac_eff_control.arm_control_node:main',
            'sim_recorder_node = isaac_eff_control.sim_recorder:main',
            'bfm_action_gateway_node = isaac_eff_control.bfm_action_gateway:main',
        ],
    },
)