# 3D Dempster-Shafer SLAM

This repository contains an C++ implementation of the 3D Dempster-Shafer SLAM algorithm. It extends the [vinySLAM](https://github.com/OSLL/slam-constructor) approach to three-dimensional space.

## Installation

#### Clone repository

 ```bash
git clone https://github.com/mishok2503/3D-Dempster-Shafer-SLAM.git
cd 3D-Dempster-Shafer-SLAM
```

#### Build

 ```bash
mkdir -p build && cd build
cmake ..
make
cp ../draw.py .
```

#### Prepare input data

You can generate input data using this TODO generator.

#### Run

```bash
./DSSLAM <input-file> [map-file] [> robot-positions-file]
```
Default `map-file` name is `"map.txt"`.

#### Draw result map

Firstly install `open3d` library.
```bash
pip install open3d
```

Then run script.
```bash
python draw.py <map-file>
```
