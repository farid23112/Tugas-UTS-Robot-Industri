from setuptools import find_packages, setup

package_name = 'robin_uts'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='NAMA_KAMU',
    maintainer_email='email@kamu.com',
    description='Node penggerak robot Robin untuk UTS (track kanan)',
    license='MIT',
    entry_points={
        'console_scripts': [
            'uts_node = robin_uts.uts_node:main',
        ],
    },
)
