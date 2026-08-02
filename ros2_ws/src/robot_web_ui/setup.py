from glob import glob

from setuptools import find_packages, setup

package_name = 'robot_web_ui'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    # static/ (index.html, app.js, deck_schematic.js) ships INSIDE the
    # installed package dir (not share/) so web_ui_node.py can find it with
    # a plain __file__-relative path at runtime, same as any other package
    # resource shipped alongside code.
    include_package_data=True,
    package_data={package_name: ['static/*']},
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='doriantodd',
    maintainer_email='doriantodd@todo.todo',
    description='Phase 8: FastAPI/WebSocket web control + telemetry UI, Steam Deck as display and controller',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'web_ui_node = robot_web_ui.web_ui_node:main',
        ],
    },
)
