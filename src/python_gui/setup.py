from setuptools import setup

package_name = 'python_gui'

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
    maintainer='agilex',
    maintainer_email='agilex@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['pythongui = python_gui.pythongui:main',
'pythongui_2 = python_gui.pythongui_2:main',
'pythongui_3 = python_gui.pythongui_3:main'
        ],
    },
)
