### Register with the cloud
In your home directory, edit a file named `team_name.sh` to define a user name, then in four separate terminals, execute the following
```
source ~/barc/scripts/reset_database.sh
roscore
rosrun data_service service.py
source ~/barc/scripts/register_cloud.sh
```
To run an experiment, enter the following three commands in three separate terminals. This should launch a single gui window with four different pluggins. (Note, experiments launched via the command line using `roslaunch barc <file_name>.launch` may save data to bagfiles, but the data will not be pushed to the cloud)
```
roscore
rosrun data_service service.py
rqt -p ~/barc/perspectives/experiment.perspective
```
When the odroid connects to the Internet through a wifi network, all data collected will be automatically pushed to the cloud, where the data will be publicly accessible. 
To view the data, go to [this site](http://dator.forge9.com/) and search for your user name.