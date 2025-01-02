from setuptools import setup

package_name = 'offboard_tests'

setup(
    name=package_name,
    version='0.0.0',
    packages=['offboard_tests'],
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
            'hypotrochoid_test_node = offboard_tests.hypotrochoid_node:main',
            'guided_test_node = offboard_tests.guided_test_node:main',
            'estimator_test_node = offboard_tests.offboard_estimator_test:main',
        ],
    },
)
