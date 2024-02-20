from setuptools import setup

package_name = 'my_package'
#sudo apt install libhidapi-hidraw0
#pip install opencv-python opencv-python-headless
setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    package_data={'my_package': ['models/yolov3-tiny.cfg', 'models/yolov3-tiny.weights', 'models/coco.names']},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/detection_launch.py']),
        ('lib/my_package/models', ['my_package/models/yolov3-tiny.weights']),
        ('lib/my_package/models', ['my_package/models/yolov3-tiny.cfg']),
        ('lib/my_package/models', ['my_package/models/coco.names']),
        ('lib/my_package', ['my_package/controller.py']),
    ],
    install_requires=['setuptools', 'opencv-python', 'easyhid', 'hidapi'],
    zip_safe=True,
    maintainer='jmazon',
    maintainer_email='jmazon@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detection = my_package.detection:main',
            'display = my_package.display:main',
            'controller = my_package.controller:main',
        ],
    },
)
