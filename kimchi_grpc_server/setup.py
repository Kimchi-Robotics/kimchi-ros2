import os
from glob import glob
from setuptools import find_packages, setup
from setuptools.command.develop import develop
from setuptools.command.install import install
import subprocess

package_name = 'kimchi_grpc_server'


class GenerateProtobuf:
    """Helper class to generate protobuf files during build."""
    
    def run_protoc(self):
        proto_dir = os.path.join(os.path.dirname(__file__), 'kimchi_grpc_server/proto')
        output_dir = os.path.join(os.path.dirname(__file__), f'{package_name}/proto')
        
        # Create output directory if it doesn't exist
        os.makedirs(output_dir, exist_ok=True)

        # Find all .proto files and compile them
        for proto_file in os.listdir(proto_dir):
            if proto_file.endswith('.proto'):
                proto_path = os.path.join(proto_dir, proto_file)
                subprocess.check_call([
                    'python3', '-m', 'grpc_tools.protoc',
                    f'-I{proto_dir}',
                    f'--python_out={output_dir}',
                    f'-I{proto_dir}',
                    f'--grpc_python_out={output_dir}',
                    proto_path
                ])
                print(f"Generated Python protobuf for {proto_file}")


class CustomDevelopCommand(develop, GenerateProtobuf):
    def run(self):
        self.run_protoc()
        develop.run(self)


class CustomInstallCommand(install, GenerateProtobuf):
    def run(self):
        self.run_protoc()
        install.run(self)

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']), 
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='arilow',
    maintainer_email='arilow@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    cmdclass={
        'develop': CustomDevelopCommand,
        'install': CustomInstallCommand,
    },
    entry_points={
        'console_scripts': [
            'kimchi_grpc_server = kimchi_grpc_server.grpc_bridge_node:main',
        ],
    },
)
