from setuptools import setup
from glob import glob

package_name = 'pose_uwb'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ("share/" + package_name + "/launch", glob("launch/*.launch*")),
        ("share/" + package_name + "/map", glob("map/*.yaml*")),
        ("share/" + package_name + "/scripts", glob("scripts/*.py*")),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Andrea Gaiani',
    maintainer_email='andrea.gaiani@usi.ch',
    description='Publisher to publish coordinate (x,y) of wrist given by UWB',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
    scripts=["scripts/wrist_pose_uwb_node.py",
             "scripts/ray_uwb.py"],
)
