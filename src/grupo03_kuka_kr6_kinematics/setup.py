from glob import glob

from setuptools import find_packages, setup

package_name = 'grupo03_kuka_kr6_kinematics'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='titi',
    maintainer_email='montanocmmc@gmail.com',
    description='Cinemática directa e inversa de posición del KUKA KR 6 R900 sixx',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'fk_node = grupo03_kuka_kr6_kinematics.fk:main',
            'ik_node = grupo03_kuka_kr6_kinematics.ik:main',
        ],
    },
)
