from setuptools import setup

package_name = 'coderun_student'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ioan Laurentiu Popa',
    maintainer_email='ioan.popa@campus.utcluj.ro',
    description='CodeRun 2026 starter template.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'solution = coderun_student.solution:main',
        ],
    },
)
