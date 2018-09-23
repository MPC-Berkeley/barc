# Learning Model Predictive Control (LMPC) for Multi-Agent Competitive Autonomous Car Racing

This repository implements learning model predictive controllers for racing multiple agents competitively on a given race track in simulation or on the Berkeley Autonomus Race Car platform (BARC). 

## Installing dependencies
After installing python and ROS, go ahead an install julia version 0.4. This package has only been tested using ROS kinetic and julia version 0.4.7. Older julia versions can be found [here](https://julialang.org/downloads/oldreleases.html). This package requires a couple of packages, which can be installed with the following command: 
```
julia <path_to_barc>/dependencies.jl
```

### Troubleshooting
Build Errors in the Gtk-package while installing the ProfileView-package can be resolved by running the following line in a separate terminal
```
sudo apt-get install libgtk-3-dev
```
followed by 
```
Pkg.build("Gtk")
```
in the julia shell.

