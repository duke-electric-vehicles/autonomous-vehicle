from setuptools import setup

package_name = 'driver_ui'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'driver_ui = driver_ui.main:main',
            'driver_ui_rtk = driver_ui.mainRTK:main',
            'fonts_utils = driver_ui.fonts_utils',
            'dashboard = driver_ui.dashboard:main',
            'jackson_dashboard = driver_ui.jackson_dashboard:main',
            'rtkdash = driver_ui.rtk_dashboard:main'
        ],
    },
)
