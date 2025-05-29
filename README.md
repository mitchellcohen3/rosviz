# ROSViz Visualization Library
A Python package for visualizing basic primitives in RViz. Currently, the following basic visualizations are implemented.
    - Poses (odometry),
    - Ellipsoids,
    - Bounding boxes,
    - Camera poses,
    - Point clouds, 
    - Lines

## Requirements
 - Python 3.6 or higher
 - ROS1 (This code was tested on ROS Noetic on Ubuntu 20.04)
 - `numpy`

## Installation 
It is recommended to install this package in a virtual environment. Create the virtual environment and install `rosviz` using
```bash
python3 -m venv .venv
source .venv/bin/activate
cd /path/to/rosviz
pip install -e .
```

## Usage 
See the `examples` folder for some basic examples on how to publish the visualization primitives contained in `rosviz.types`. Each visualization primitive contains an `update()` function that updates the data within the primitive (attitude, position), and publishes it over ROS, allowing it to be visualized in RViz.
