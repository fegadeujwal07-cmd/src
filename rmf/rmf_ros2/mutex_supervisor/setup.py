from setuptools import setup, find_packages

package_name = 'mutex_supervisor'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name + '/launch', ['launch/mutex_supervisor.launch.xml']),
        ('share/' + package_name + '/config', ['config/mutex_supervisor_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your-email@example.com',
    description='External mutex supervisor for Nav2-based fleet management with RMF',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mutex_supervisor=mutex_supervisor.main:main',
        ],
    },
)

