from setuptools import find_packages, setup

setup(
    name="rospy_x402",
    version="0.0.0",
    packages=find_packages("src"),
    package_dir={"": "src"},
    install_requires=[
        "pynacl",
    ],
)

