import os
from glob import glob
from setuptools import setup

package_name = 'tetra_2dnav'

setup(
  name=package_name,
  version='0.0.0',
  packages=[package_name],
  data_files=[
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.py'))),
    (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
    (os.path.join('share', package_name, 'odom_nav_config'), glob(os.path.join('odom_nav_config', '*.yaml'))),
    (os.path.join('share', package_name, 'maps'), glob(os.path.join('maps', '*.yaml'))),
    (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*.rviz'))),
  ],
  install_requires=['setuptools'],
  zip_safe=True,
  maintainer='tetra',
  maintainer_email='jjy0607@hanyang.ac.kr',
  description='TODO: Package description',
  license='TODO: License declaration',
  tests_require=['pytest'],
  entry_points={
    'console_scripts': [
      'tf_listener_py = tetra_2dnav.tf_listener:main',
      'rob_glob_Pose = tetra_2dnav.robot_global_pose:main',
      'landmark_sampler = tetra_2dnav.publish_fake_random_landmarks:main',
      'detection_publisher = tetra_2dnav.detection_publisher:main',
      'detection_collector = tetra_2dnav.detection_collector:main',
      'detection_publisher = tetra_2dnav.detection_publisher:main',
    ],
  },
)
