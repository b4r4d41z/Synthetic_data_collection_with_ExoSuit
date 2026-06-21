from setuptools import find_packages, setup

package_name = 'isaac_eff_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    data_files=[
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
        ],
    },
)