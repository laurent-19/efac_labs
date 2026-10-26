from glob import glob

from setuptools import setup

package_name = 'coderun_challenge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/worlds', glob('worlds/*.world')),
        ('share/' + package_name + '/worlds/include', glob('worlds/include/*.inc')),
        ('share/' + package_name + '/bitmaps', glob('bitmaps/*.png')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ioan Laurentiu Popa',
    maintainer_email='ioan.popa@campus.utcluj.ro',
    description='CodeRun 2026 Robotics.AI challenge: worlds, launch and scorer.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'scorer = coderun_challenge.scorer:main',
        ],
    },
)
