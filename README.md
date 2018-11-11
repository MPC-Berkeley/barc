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

## Configuration
### Add HOME_DIR environment variable
Create the environment variable HOME_DIR, which is a string which points to your home directory/the directory which includes the barc folder. Create a julia configuration file, if you do not have one already:
```
cd
vim .juliarc.jl
```
and add the following line: 
```
ENV["HOME_DIR"] = <path_to_your_home_folder>
```

### Config file
The config.jl file allows for multiple different settings. 
* MODE: One of ["path_following", "learning", "racing"]
* INITIALIZATION_TYPE: One of ["inner", "center", "outer"]
* TRACK_NAME: One of ["oval", "l_shape"]

### Launch file
The parameters delay_df and delay_a specifiy the delay for the steering and the acceleration in the controller and in the estimator. For the controller they are given in time steps (value="1") and for the estimator in seconds (value="0.1"). The parameters time_offset specify how long after the initialization the agent should wait before applying control inputs. This allows the two agents to start at different times. 

## Running the code
### Simulation Agent
To run agent 1 in simulation execute the following command:
```
roslaunch barc sim_agent_1.launch
```
For path following MPC set MODE to "path_following" and for learning set MODE to "learning". This runs the agent on the track and records the data. Notice that you have to run the command first in the path following mode to collect data for LMPC. To run agent 2 in simulation run the following command: 
```
roslaunch barc sim_agent_2.launch
```
