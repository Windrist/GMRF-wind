# GMRF-wind

Gaussian Markov Random Field (GMRF) based real-time 2D wind field estimation for robotic olfaction systems. Part of the [GSExploration](../documents/knowledge-overview.md) gas source localization system.

## Overview

A Gaussian Markov Random Field (GMRF) is a probabilistic graphical model where random variables follow a multivariate Gaussian distribution with Markov properties (local dependencies). This package applies GMRF to estimate 2D wind vector fields from sparse anemometer observations while respecting physical constraints.

This is a ROS 2 implementation of the algorithm presented in:

> Monroy, J., Jaimez, M., & Gonzalez-Jimenez, J. (2017). **Online Estimation of 2D Wind Maps for Olfactory Robots**. International Symposium on Olfaction and Electronic Nose (ISOEN). [DOI: 10.1109/ISOEN.2017.7968883](https://ieeexplore.ieee.org/document/7968883)

## Energy Function

The GMRF-W framework estimates wind maps (W) from observations (Z) by minimizing an energy function:

```
E(W,Z) = Ez(W,Z) + Em(W) + Eo(W) + Er(W)
```

| Energy Term | Description             | Physical Meaning                         |
| ----------- | ----------------------- | ---------------------------------------- |
| **Ez**      | Observation consistency | Wind estimates match sensor measurements |
| **Em**      | Mass conservation       | Incompressible flow (divergence-free)    |
| **Eo**      | Obstacle boundary       | Zero normal flow at walls                |
| **Er**      | Regularization          | Smooth transitions between cells         |

This allows real-time 2D approximation of Computational Fluid Dynamics (CFD) suitable for mobile robot applications.

## Package Structure

```
GMRF-wind/
├── gmrf_msgs/                  # Message definitions
│   └── msg/
│       └── WindEstimate.msg
└── gmrf_wind_mapping/          # Core algorithm
    ├── src/
    │   └── gmrf_wind_mapping_node.cpp
    └── scripts/
```

## Installation

### Prerequisites

```bash
# ROS 2 Humble
sudo apt install ros-humble-desktop

# Eigen3
sudo apt install libeigen3-dev
```

### Building

```bash
cd ~/Bio_ws
colcon build --packages-select gmrf_msgs gmrf_wind_mapping
source install/setup.bash
```

## Usage

### Launch with GSExploration System

The GMRF wind mapping node is launched as part of the main system:

```bash
ros2 launch main_decision demo_sim.py
```

### Standalone Launch

```bash
ros2 run gmrf_wind_mapping gmrf_wind_mapping_node \
    --ros-args \
    -p cell_size:=0.5 \
    -p lambda_obs:=1.0 \
    -p lambda_div:=0.1
```

## Parameters

| Parameter         | Default | Description                            |
| ----------------- | ------- | -------------------------------------- |
| `cell_size`       | 0.5     | Grid resolution in meters              |
| `lambda_obs`      | 1.0     | Weight for observation term (Ez)       |
| `lambda_div`      | 0.1     | Weight for divergence-free term (Em)   |
| `lambda_boundary` | 10.0    | Weight for obstacle boundary term (Eo) |
| `lambda_reg`      | 0.01    | Regularization weight (Er)             |

## Topics

### Subscribed

| Topic         | Type                        | Description                 |
| ------------- | --------------------------- | --------------------------- |
| `/anemometer` | `olfaction_msgs/Anemometer` | Wind sensor measurements    |
| `/map`        | `nav_msgs/OccupancyGrid`    | Occupancy map for obstacles |

### Published

| Topic            | Type                             | Description               |
| ---------------- | -------------------------------- | ------------------------- |
| `/wind_field`    | `visualization_msgs/MarkerArray` | Wind vector visualization |
| `/gmrf/wind_map` | `gmrf_msgs/WindEstimate`         | Wind field estimate       |

## Integration

Used by the following GSExploration components:

- `scoring_node` — Wind probability calculation for region scoring
- `gsl_local_search` — Upwind direction for Surge-Cast algorithm

## Dependencies

### ROS 2 Packages

- `rclcpp` — ROS 2 C++ client library
- `olfaction_msgs` — Anemometer message types
- `nav_msgs` — Occupancy grid
- `visualization_msgs` — RViz markers

### External Libraries

- `Eigen3` — Linear algebra operations

## Citation

If you use this package in your research, please cite:

```bibtex
@INPROCEEDINGS{jmonroy_isoen_2017,
     author = {Monroy, Javier and Jaimez, Mariano and Gonzalez-Jimenez, Javier},
      title = {Online Estimation of 2D Wind Maps for Olfactory Robots},
  booktitle = {International Symposium on Olfaction and Electronic Nose (ISOEN)},
       year = {2017},
   location = {Montreal (Canada)},
        doi = {10.1109/ISOEN.2017.7968883},
      pages = {1--3}
}
```

## License

GPL-3.0 (See LICENSE file)

## Related Documentation

- [System Architecture](../documents/knowledge-architecture.md)
- [Domain Glossary](../documents/business/business-glossary.md)
- [Feature Specifications](../documents/business/business-features.md)
