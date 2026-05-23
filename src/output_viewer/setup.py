from setuptools import find_packages, setup

package_name = 'output_viewer'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/rviz', ['rviz/nav.rviz']),
        ('share/' + package_name + '/launch', ['launch/viewer.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dev',
    maintainer_email='dev@todo.todo',
    description='Visualizador heatmap para /depth_grid',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'depth_grid_heatmap = output_viewer.output_viewer_node:main',
            'grid_signal_monitor = output_viewer.grid_signal_monitor:main',
            'odometry_path = output_viewer.odometry_path_node:main',
        ],
    },
)
