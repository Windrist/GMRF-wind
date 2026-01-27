# gmrf_msgs

ROS 2 message and service definitions for the GMRF (Gaussian Markov Random Field) wind estimation system.

## Overview

This package provides the interface definitions for the GMRF wind mapping algorithm. It enables other ROS 2 nodes to query wind field estimates and receive structured wind data.

## Services

### WindEstimation.srv

Queries the GMRF wind field for wind vector estimates at specified locations.

```yaml
# Request
geometry_msgs/Point[] query_points   # Points to query wind at
---
# Response
geometry_msgs/Vector3[] wind_vectors # Estimated wind vectors (u, v, 0)
float64[] confidence                 # Estimation confidence per point
bool success
```

## Installation

This package is built as part of the GMRF-wind module:

```bash
cd ~/Bio_ws
colcon build --packages-select gmrf_msgs
source install/setup.bash
```

## Dependencies

- `std_msgs` — Standard ROS 2 messages
- `geometry_msgs` — Point, Vector3 types
- `rosidl_default_generators` — Message generation

## Integration

Used by:

- `gmrf_wind_mapping` — Wind estimation node
- `main_decision` — Scoring node for wind probability
- `gsl_local_search` — Upwind direction for Surge-Cast

## License

GPLv3

## Authors

- Pepe Ojeda (ojedamorala@uma.es) — Original author
- MAPIRlab — [GitHub](https://github.com/MAPIRlab)
