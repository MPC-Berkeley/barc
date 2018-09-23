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

### Adding get_name() from RobotOS
This package requires access to the function get_name() from RobotOS, which is not supported by default (pull request is pending). In order to be able to access this function follow the next steps. Change into the source folder of your RobotOS package in the julia directory where you install all your packages. this should be at the following location: 
```
cd ~/.julia/v0.4/RobotOS/src
```
Using your favorite editor add "get_name" to
```
export init_node, is_shutdown, spin,
       get_param, has_param, set_param, delete_param,
       logdebug, loginfo, logwarn, logerr, logfatal
```
so you get
```
export init_node, is_shutdown, spin,
       get_param, has_param, set_param, delete_param,
       logdebug, loginfo, logwarn, logerr, logfatal, get_name
```
Rebuild the package in a julia console with
```
Pkg.build("RobotOS")
```