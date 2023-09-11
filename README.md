# 3D Dempster-Shafer SLAM

This repository contains an C++ implementation of the 3D Dempster-Shafer SLAM algorithm. It extends the [vinySLAM](https://github.com/OSLL/slam-constructor) approach to three-dimensional space.

## Usage

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

You can generate input data using [this generator](https://github.com/mishok2503/slam-3d-datasets-generator).

#### Run

```bash
./DSSLAM <input-file> [map-file] [> robot-trajectory-file]
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

#### Draw robot trajectory

## How it works

### Map updating

At the beginning, the robot is placed in the center of `mapBuildSteps` empty map (the robot is considered a particle point), and the first n scans are applied to the map,
without correcting the robot's position, since at least some part of the map is needed for position correction.

The map is updated as follows: for each cell *P* from the input lidar scan we mark all cells on the path from the robot to this point empty,
and all cells in a cube of size `2 * holeSize + 1` centered at *P* we mark occupied, the closer to the center the more occupied.
Marking a cell empty/occupied means to update the cell with some probability of being occupied.
Also, each point has a quality, which is some measure of how much confidence the lidar has in the accuracy of the measurement of that point.

### Cell models

##### TCountingCell

This type of cell counts the weighted average of all its updates, where weights are the quality at update. Initially the state (probability that it is occupied) of a cell is 0.5 with weight 1.
Example: a cell was updated to be empty (probability `0`) with quality `1`. And then updated that it is occupied (probability `0.7`) with quality `0.1`.
Then the final probability that the cell is occupied is: `(0.5 * 1 + 0 * 1 + 0.7 * 0.1) / (1 + 1 + 0.1) = 0.2714`.

##### TDSCell

This type of cell uses the [Dempster Shafer theory](https://en.wikipedia.org/wiki/Dempster–Shafer_theory).
Initial cell state:
* unknown: `0`
* empty: `0.5 * (1 - c)`
* occupied: `0.5 * (1 - c)`
* conflict: `c = TDSCell.DEFAULT_CONFLICT`.

When cell updating with probability `p` and quality `q`, a new cell is created:
* unknown: `0`
* empty: `(1 - p) * (1 - c)`
* occupied: `p * (1 - c)`
* conflict: `c = max(0.999, TDSCell.DEFAULT_CONFLICT / q)`

And then this cell is combined with the current cell by [Dempster's rule of combination](https://en.wikipedia.org/wiki/Dempster–Shafer_theory#Dempster's_rule_of_combination).

Example: a cell (where `TDSCell.DEFAULT_CONFLICT = 0.1`) was updated to be empty (probability `0`) with quality `1`. And then updated that it is occupied (probability `0.7`) with quality `0.1`.
Then the final state is:
* unknown: `0`
* empty: `0.90756`
* occupied: `0.0756341`
* conflict: `0.0168062`.

Probability that a cell is occupied is calculated by the formula: `occupied + conflict / 2`. In example this is `0.0840372`.

### Error correction

After the first few steps, the robot begins to correct its position at each step using so-called scan matching.
Before updating the map, a `samplesCount` of copies of the robot is created, but slightly shifted and rotated relative to the original by a random value with a normal distribution.
Then a score is calculated for each of them, which indicates how well the new lidar scan fits into the existing map. Score is calculated as follows:
From the point of the new lidar scan, select those that indicate occupancy (those not farther than the maximum measurement).
Then for each of them we take the extent to which the cell in which this point falls is occupied and multiply it by the quality of the point.
And then summarize all the values. After that we select the robot with the maximum score and consider that this is the actual position of the robot and perform all further steps in relation to it.