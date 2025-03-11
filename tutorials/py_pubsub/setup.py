from setuptools import setup

package_name = "py_pubsub"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Ekumen Repository Management",
    maintainer_email="repos@ekumenlabs.com",
    keywords=["ROS"],
    classifiers=[
        "Intended Audience :: Developers",
        "License :: OSI Approved :: Apache Software License",
        "Programming Language :: Python",
        "Topic :: Software Development",
    ],
    description="Example Python ROS package",
    license="Apache License, Version 2.0",
    tests_require=["pytest"],
    entry_points={
        "pytest11": ["launch_ros = launch_testing_ros_pytest_entrypoint"],
        "console_scripts": [
            "talker = py_pubsub.talker:main",
            "listener = py_pubsub.listener:main",
        ],
    },
)
