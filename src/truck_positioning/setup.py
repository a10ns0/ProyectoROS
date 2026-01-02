from setuptools import find_packages, setup

package_name = 'truck_positioning'

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
    maintainer='sick',
    maintainer_email='sick@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'logic_node = truck_positioning.logic_node:main',
            'lector = truck_positioning.ejemplo:main',
            'cliente_API = truck_positioning.cliente_API:main',
            'sub_BD = truck_positioning.sub_BD:main',
            'truck_perception = truck_positioning.truck_perception:main',
            'truck_control = truck_positioning.truck_control:main',
            'simple_truck_sim = truck_positioning.simple_truck_sim:main',
            'integrado = truck_positioning.sistema_integrado.py:main',
            
        ],
    },
)
