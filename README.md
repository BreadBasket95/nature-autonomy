# NATURE-stack
The NATURE (Navigating All Terrains Using Robotic Exploration) autonomy stack is a full stack for off-road navigation. It includes modules for perception, path planning, and vehicle control, with options for Ackermann and skid-steered vehicles.

For more information about the modules and their capabilities, see the upstream wiki.

## Status
This fork removes middleware-specific dependencies. Messaging uses placeholder structs in `include/nature/messaging` and a stub runtime in `include/nature/node` so the core algorithms can be exercised while the ASPN ICD and ZeroMQ transport are integrated. All message fields are manually extracted and populated.

## Build
This repo now builds with a standalone CMake flow.

```bash
cmake -S . -B build
cmake --build build
```

Notes:
- If X11 is available, visualization helpers will link against it. Otherwise image display is disabled.
- If OpenMP is available, FTTE acceleration will link against it.

## Running
Executables are produced in the build directory (for example, `build/nature_local_planner_node`). Integration and orchestration are now external to this repo and should be handled by the new transport layer and your system launcher.

## Funding Acknowledgement
This project is made possible by technical and financial support of the Mississippi State University Center for Advanced Vehicular Systems as well as the Automotive Research Center (ARC) in accordance with Cooperative Agreement W56HZV 14 2 0001 U.S. Army CCDC Ground Vehicle Systems Center (GVSC) Warren, MI.
