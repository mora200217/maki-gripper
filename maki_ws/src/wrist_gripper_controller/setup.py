from setuptools import setup

package_name = 'wrist_gripper_controller'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='andres',
    maintainer_email='email@example.com',
    description='Wrist + Gripper controller with Tkinter GUI',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'controller = wrist_gripper_controller.controller_node:main',
            'gui = wrist_gripper_controller.gui:main',
        ],
    },
)
