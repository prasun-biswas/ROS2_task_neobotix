# ROS2_task_neobotix
Performs following tasks:
1. Measure the total distance the robot travelled in meters. loggs informations if robot moves and updates
  json file with odometry data.
  
  Required scripts "movement_detector_mpo.py" and "store_json.py". 
  
2. For each new navigation goal logs path, total driven distance and checks if goal reached. 

  Required scripts "movement_detector_mpo.py" and "store_json.py". 

Run both files from separate terminal. source ROS2 foxy to be available from new terminal window.

## Install neobotix simulation environment by performing the steps from this link below.

	https://docs.neobotix.de/display/R2/Installation+on+Ubuntu
  
## Set up neobotix simulation environment by performing the steps below.

  Select the robot, workspace and launch simulation from a separate terminal:
  
      export MY_ROBOT=mpo_500

      export MAP_NAME=neo_workshop

      ros2 launch neo_simulation2 simulation.launch.py
  
  
  Select the robot, workspace and launch simulation from a separate terminal:
  
      export MY_ROBOT=mpo_500

      export MAP_NAME=neo_workshop

      ros2 launch neo_simulation2 navigation.launch.py
      
   For teleoperation run the following command on a separate terminal:
   	$ ros2 run teleop_twist_keyboard teleop_twist_keyboard
    
    
    ## To use pycharm IDE with ROS: 
    Go to the directory "/snap/pycharm-community/244/bin" and run following commands.
	$ source /opt/ros/foxy/setup.bash 
  $ ./pycharm.sh
  
  *** comment here if there are any trouble with running the code ***
