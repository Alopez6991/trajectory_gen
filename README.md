# Steps for runing and generating a trajectory

## make a trajectory
* navigate to ``https://github.com/Alopez6991/trajectory_gen/blob/main/src/make_your_trajectory.ipynb``
* design trajectory in body level cordinates
* save trajectory as global postions
* save trajectory as global veliocities
* move the csv files to ``https://github.com/Alopez6991/trajectory_gen/tree/main/trajectories``

## set up ros configurations
* navigate to ``https://github.com/Alopez6991/trajectory_gen/blob/main/trajectories/csv_trajectories.yaml``
* update the ``waypoint_list`` params to the new csv name
* navigate to ``https://github.com/Alopez6991/trajectory_gen/blob/main/config/start_mode.yaml``
* update the ``flight_mode:`` param to 'velocity_global' or 'postion'

## Run the ros scripts
``roslaunch trajectory_gen fight_control.launch``
