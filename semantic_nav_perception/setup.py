from setuptools import find_packages, setup

package_name = 'semantic_nav_perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='medhap',
    maintainer_email='medhap@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'scene_perception_node = semantic_nav_perception.scene_perception_node:main',
            'clicked_point_target_node = semantic_nav_perception.clicked_point_target_node:main',
            'save_named_place_node = semantic_nav_perception.save_named_place_node:main',
            'go_to_named_place_node = semantic_nav_perception.go_to_named_place_node:main',
        ],
    },
)
