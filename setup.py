from setuptools import find_packages, setup
import os
from glob import glob # globをインポート

package_name = 'crane_aruco_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 【修正箇所】: launchディレクトリ内のファイルをインストール
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.py'))), 
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rsdlab',
    maintainer_email='rsdlab@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'camera_publisher = crane_aruco_control.camera_publisher_node:main',
        'aruco_controller = crane_aruco_control.aruco_controller_node:main',
        ],
    },
)
