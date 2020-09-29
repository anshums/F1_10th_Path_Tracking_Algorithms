# Model Predictive Control

<p align = "center">
<img src = "https://github.com/prateeks97/F1_10th_Path_Tracking_Algorithms/blob/master/Path%20Tracking/MPC/images/MPC.gif">
</p>

## Description

 - MPC works on a receding horizon approach.
 - It involves repeatedly solving a constrained optimization problem, using predictions of future costs, disturbances, and constraints over a moving time horizon to choose the control action.
 - One thing to note in implementation of MPC is that because optimization can take some amount of time, the state of the vehicle when starting the optimization will be different from the state of the vehicle when completing the optimization. As a result, we use a predicted state in the optimization for the time at which the control input will be applied.

## MPC Structure
<p align = "center">
<img src = "https://github.com/prateeks97/F1_10th_Path_Tracking_Algorithms/blob/master/Path%20Tracking/MPC/images/3-s2.0-B9780081017531000024-f02-02-9780081017531.jpg" width= "500" height="400">
</p>

## MPC Working
<p align = "center">
<img src = "https://github.com/prateeks97/F1_10th_Path_Tracking_Algorithms/blob/master/Path%20Tracking/MPC/images/maxresdefault.jpg" width= "500" height="400">
</p>

## MPC Implementation
For implementation details of the plant model and states used, refer [this](https://github.com/AtsushiSakai/PythonRobotics/blob/master/PathTracking/model_predictive_speed_and_steer_control/Model_predictive_speed_and_steering_control.ipynb).

## Steps to setup Environment and run the code

 - First download the [f1tenth_gym_ros](https://github.com/f1tenth/f1tenth_gym_ros) which is a containerized ROS communication bridge for F1TENTH gym environment. Just follow the steps given on the github page.
 - Then launch the simulator by-  `./docker.sh` file present in the simulator repository.​
 - Launch the rviz window and the simulator environment by- `roslaunch f1tenth_gym_ros gym_bridge_ros.launch​`
 - Launch the localization node that uses particle filter algorithm by- `roslaunch  f1tenth_gym_ros localization.launch`
 - We also provide the opponent vehicle a constant velocity to be able to run code on the ego vehicle.
 - Finally we navigate to the directory where the MPC controller python script is located and run it.​
