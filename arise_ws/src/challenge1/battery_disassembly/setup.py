import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'battery_disassembly'
config = "parameters.json"

setup(
    name=package_name,                                      # Nombre del paquete
    version='0.0.0',
    packages=find_packages(exclude=['test']),               # Para encontrar automáticamente todos los paquetes dentro del directorio del proyecto, excluyendo el directorio test.
    data_files=[                                            # Archivos de datos
        ('share/ament_index/resource_index/packages',
            ['resource/' + config]),
        ('share/' + package_name, ['package.xml']),
        ('share/ament_index/resource_index/arise_system_module',
            ['module/' + package_name]),
        ('share/' + package_name + '/launch', ['launch/battery_disassembly.launch.py']),
    ],
    install_requires=['setuptools'],                        # Dependencias del paquete
    zip_safe=True,                                          # Indica si el paquete puede ser distribuido en un archivo ZIP
    maintainer='raucal',
    maintainer_email='raucal@cartif.es',
    description='Disassembly of electric car battery screws',
    license='ARISE - CARTIF',
    tests_require=['pytest'],                               # Dependencias requeridas para ejecutar las pruebas del paquete.

    # Scripts que se ejecutan desde el CMD: Autocompletados ros2 run
    entry_points={
        'console_scripts': [
            'hri_communication = battery_disassembly.hri_communication:main',
            'detect_screws = battery_disassembly.detect_screws:main',
            'ur10e_screw = battery_disassembly.ur10e_screw:main',
            'audio_interface = battery_disassembly.audio_interface:main',
        ],
    },
    
    

)
