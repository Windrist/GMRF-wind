# gmrf_wind_mapping

Real-time 2D wind field estimation using Gaussian Markov Random Fields (GMRF).

## Overview

This package implements the GMRF-W algorithm for estimating coherent 2D wind vector fields from sparse anemometer measurements. The algorithm respects physical constraints including mass conservation (divergence-free flow) and obstacle boundaries.

This is based on the research paper:

> Monroy, J., Jaimez, M., & Gonzalez-Jimenez, J. (2017). **Online Estimation of 2D Wind Maps for Olfactory Robots**. ISOEN 2017. [DOI: 10.1109/ISOEN.2017.7968883](https://ieeexplore.ieee.org/document/7968883)

## Algorithm

The GMRF framework estimates wind fields by minimizing an energy function:

```
E(W,Z) = Ez(W,Z) + Em(W) + Eo(W) + Er(W)
```

| Term | Weight     | Description                                         |
| ---- | ---------- | --------------------------------------------------- |
| Ez   | λ_obs      | Observation consistency — match sensor measurements |
| Em   | λ_div      | Mass conservation — encourage divergence-free flow  |
| Eo   | λ_boundary | Obstacle boundary — zero normal flow at walls       |
| Er   | λ_reg      | Regularization — smooth field variations            |

## Package Structure

```
gmrf_wind_mapping/
├── src/
│   ├── gmrf_node.cpp        # ROS 2 node wrapper
│   ├── gmrf_node.h          # Node interface
│   ├── gmrf_map.cpp         # Core GMRF algorithm (57KB)
│   ├── gmrf_map.h           # Algorithm interface
│   └── Utils.h              # Utility functions
├── scripts/
│   └── ...                  # Visualization scripts
├── CMakeLists.txt
└── package.xml
```

## Installation

### Prerequisites

```bash
# Eigen3 for linear algebra
sudo apt install libeigen3-dev

# OpenCV for visualization
sudo apt install libopencv-dev
```

### Building

```bash
cd ~/Bio_ws
colcon build --packages-select gmrf_msgs gmrf_wind_mapping
source install/setup.bash
```

## Usage

### Launch with GSExploration

The GMRF node is launched automatically with the main system:

```bash
ros2 launch main_decision demo_sim.py
```

### Standalone Execution

```bash
ros2 run gmrf_wind_mapping gmrf_wind_mapping_node \
    --ros-args \
    -p cell_size:=0.5 \
    -p lambda_obs:=1.0 \
    -p lambda_div:=0.1 \
    -p lambda_boundary:=10.0
```

## Parameters

| Parameter         | Default | Description                                |
| ----------------- | ------- | ------------------------------------------ |
| `cell_size`       | 0.5     | Grid cell size (meters)                    |
| `lambda_obs`      | 1.0     | Observation term weight                    |
| `lambda_div`      | 0.1     | Divergence-free (mass conservation) weight |
| `lambda_boundary` | 10.0    | Obstacle boundary condition weight         |
| `lambda_reg`      | 0.01    | Regularization (smoothness) weight         |
| `map_frame`       | "map"   | TF frame for wind field                    |
| `update_rate`     | 5.0     | Wind field update rate (Hz)                |

## Topics

### Subscribed

| Topic         | Type                        | Description                 |
| ------------- | --------------------------- | --------------------------- |
| `/anemometer` | `olfaction_msgs/Anemometer` | Wind sensor readings        |
| `/map`        | `nav_msgs/OccupancyGrid`    | Occupancy map for obstacles |

### Published

| Topic              | Type                             | Description               |
| ------------------ | -------------------------------- | ------------------------- |
| `/wind_array_pub`  | `visualization_msgs/MarkerArray` | Wind vector visualization |
| `/gmrf/wind_image` | `sensor_msgs/Image`              | Wind field as image       |

### Services

| Service               | Type                       | Description                   |
| --------------------- | -------------------------- | ----------------------------- |
| `/gmrf/estimate_wind` | `gmrf_msgs/WindEstimation` | Query wind at specific points |

## Visualization

The wind field is visualized as arrows in RViz:

1. Add `MarkerArray` display
2. Set topic to `/wind_array_pub`
3. Arrows indicate wind direction and magnitude

## Key Files

| File            | Size | Description                                   |
| --------------- | ---- | --------------------------------------------- |
| `gmrf_map.cpp`  | 57KB | Core GMRF algorithm, sparse matrix operations |
| `gmrf_node.cpp` | 14KB | ROS 2 integration, subscribers, publishers    |
| `gmrf_map.h`    | 7KB  | Class interface, energy function definitions  |

## Dependencies

### ROS 2 Packages

- `rclcpp` — ROS 2 C++ client library
- `gmrf_msgs` — Service definitions
- `olfaction_msgs` — Anemometer message type
- `visualization_msgs` — Marker visualization
- `tf2_geometry_msgs` — Coordinate transforms

### External Libraries

- `Eigen3` — Sparse linear algebra
- `OpenCV` — Image visualization

## Citation

```bibtex
@INPROCEEDINGS{jmonroy_isoen_2017,
     author = {Monroy, Javier and Jaimez, Mariano and Gonzalez-Jimenez, Javier},
      title = {Online Estimation of 2D Wind Maps for Olfactory Robots},
  booktitle = {International Symposium on Olfaction and Electronic Nose (ISOEN)},
       year = {2017},
        doi = {10.1109/ISOEN.2017.7968883},
}
```

## License

GPLv3

## Authors

- Javier G. Monroy (jgmonroy@uma.es) — Original author
- MAPIRlab — [GitHub](https://github.com/MAPIRlab)
