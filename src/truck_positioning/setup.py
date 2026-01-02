import os
from glob import glob
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
	(os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
	(os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
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
        'calibrador_sensores = truck_positioning.calibrador_sensores:main',
        'visualizador = truck_positioning.visualizador_perfil:main',
        'visualizador_amarillo = truck_positioning.visualizador_amarillo:main',
        'monitor_dual = truck_positioning.monitor_dual:main',
	    'calculador_distancia = truck_positioning.calculador_distancia:main',
	    'interfaz_camion = truck_positioning.interfaz_camion:main',
	    'cliente_API = truck_positioning.cliente_API:main',
	    'sub_BD = truck_positioning.sub_BD:main',
	    'camion_3D = truck_positioning.open3d:main',
	    'simple_truck_sim = truck_positioning.simple_truck_sim:main',
	    'sistema_integrado = truck_positioning.sistema_integrado:main',
        'vision_PC = truck_positioning.vision_PC:main',

        ],
    },
)
