from setuptools import find_packages, setup

package_name = 'prime_factors_pkg'

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
    maintainer='starkojik',
    maintainer_email='starkojik@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'number_publisher = prime_factors_pkg.number_publisher:main',
            'prime_factor_subscriber = prime_factors_pkg.prime_factor_subscriber:main',
        ],
    },
)

