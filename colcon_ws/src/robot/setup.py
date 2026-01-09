import os 
from glob import glob
from setuptools import find_packages, setup

import fnmatch


def get_mesh_files():
    mesh_files = []
    for root, dirnames, filenames in os.walk('meshes'):
        for filename in fnmatch.filter(filenames, '*.*'):
            mesh_files.append(os.path.join(root, filename))
    return mesh_files

package_name = 'robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
        data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Archivos específicos, no carpetas
        (os.path.join('share', package_name, 'launch'), 
         [f for f in glob('launch/*') if os.path.isfile(f)]),
        (os.path.join('share', package_name, 'urdf'), 
         [f for f in glob('urdf/*') if os.path.isfile(f)]),
        (os.path.join('share', package_name, 'rviz'), 
         [f for f in glob('rviz/*') if os.path.isfile(f)]),
        # Para partes, solo archivos

        (os.path.join('share', package_name, 'urdf/parts'), 
         [f for f in glob('urdf/parts/*') if os.path.isfile(f)]),
        (os.path.join('share', package_name, 'urdf/parts/base'), 
         [f for f in glob('urdf/parts/base/*') if os.path.isfile(f)]),
          (os.path.join('share', package_name, 'urdf/parts/left_thigh'), 
         [f for f in glob('urdf/parts/left_thigh/*') if os.path.isfile(f)]),
          (os.path.join('share', package_name, 'urdf/parts/leg_joint'), 
         [f for f in glob('urdf/parts/leg_joint/*') if os.path.isfile(f)]),
         (os.path.join('share', package_name, 'urdf/parts/right_thigh'), 
         [f for f in glob('urdf/parts/right_thigh/*') if os.path.isfile(f)]),
         (os.path.join('share', package_name, 'urdf/parts/shin'), 
         [f for f in glob('urdf/parts/shin/*') if os.path.isfile(f)]),
        (os.path.join('share', package_name, 'meshes'), get_mesh_files()),
         
         
         
        # ... repetir para otras carpetas ...
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='alex',
    maintainer_email='alex@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
