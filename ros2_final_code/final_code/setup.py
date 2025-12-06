from setuptools import setup, find_packages

package_name = 'dsr_rokey2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rokey',
    maintainer_email='88tkeks@naver.com',
    description='Doosan Robot Custom Package',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'tool_grip = dsr_rokey2.tool_grip:main',
            'wipe_floor = dsr_rokey2.wipe_floor:main',
            'water_floor = dsr_rokey2.water_floor:main',
            'move_basic = dsr_rokey2.move_basic:main',
            'move_periodic = dsr_rokey2.move_periodic:main',
            'force_test = dsr_rokey2.force_test:main',
            'mini_jog = dsr_rokey2.mini_jog:main',
            'single_thread_test = dsr_rokey2.single_thread_test:main',
            'cal_angle = dsr_rokey2.cal_angle:main',
            'cal_length = dsr_rokey2.cal_length:main',
            'code_test = dsr_rokey2.code_test:main',
            'main_controller = dsr_rokey2.main_controller:main',
            'check_map = dsr_rokey2.check_map:main',
        ],
    },
)

