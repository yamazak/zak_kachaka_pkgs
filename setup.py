from setuptools import find_packages, setup

package_name = 'zak_kachaka_move'

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
    maintainer='yamazaki',
    maintainer_email='kimitoshi.yamazaki.c5@tohoku.ac.jp',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'move = zak_kachaka_move.test_move:main',
            'move2 = zak_kachaka_move.simple_move:main',
            'move3 = zak_kachaka_move.map_move:main',
        ],
    },
)
