import setuptools

setuptools.setup(
    name="rosviz",
    version="1.0",
    author="Mitchell Cohen",
    author_email="mitchell.cohen3@mail.mcgill.ca",
    description="A Python package containing useful visualization primitives for use in RViz.",
    install_requires=[
        "numpy",
        "matplotlib",
        "rospkg",
        "catkin_pkg",
    ],
    packages=setuptools.find_packages(),
    python_requires=">=3.6",
)
