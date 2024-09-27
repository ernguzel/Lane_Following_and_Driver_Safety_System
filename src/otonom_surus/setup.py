from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'otonom_surus'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch dosyalarını da ekle
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='eren',
    maintainer_email='eren@todo.todo',
    description='Otonom sürüş ve çizgi takibi paketi',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'line_detector_node = otonom_surus.line_detector:main',
            'sleep_detection = otonom_surus.sleep_detection:main',
            'karar_verici = otonom_surus.karar_verici:main',
        ],
    },
)

