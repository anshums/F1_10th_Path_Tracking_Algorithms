# Stanley Controller
## Description
 - The [Stanley](https://www.ri.cmu.edu/pub_files/2009/2/Automatic_Steering_Methods_for_Autonomous_Automobile_Path_Tracking.pdf) method is the path tracking approach used by Stanford University’s autonomous vehicle entry in the DARPA Grand Challenge, Stanley.
 - The Stanley method is a nonlinear feedback function of the cross track error , measured from the center of the front axle to the nearest path point (cx, cy).
<p align = "center">
<img src = "https://github.com/prateeks97/F1_10th_Path_Tracking_Algorithms/blob/master/Path%20Tracking/Stanley/images/stanley.gif">
</p>

## Theory
 - The controller uses geometric approach for path planning. This implies that the controller does not take dynamic factors into account.
 - In this the steering angle is fed as the sum of heading error and cross-track error.
 - Heading error is given by the difference of vehicle yaw and the heading of the path.
 - Cross-track error is given by the shortest distance between the front axle and the path.
 - The second term in the formula is tuned based on the values of constant 'k' and velocity (v(t)).

<p align = "center">
<img src = "https://github.com/prateeks97/F1_10th_Path_Tracking_Algorithms/blob/master/Path%20Tracking/Stanley/images/stanley%20formula.JPG" height="500" width = "400">
</p>

## Steps to setup Environment and run the code

 - First download the [f1tenth_gym_ros](https://github.com/f1tenth/f1tenth_gym_ros) which is a containerized ROS communication bridge for F1TENTH gym environment. Just follow the steps given on the github page.
 - Then launch the simulator by-  `./docker.sh` file present in the simulator repository.​
 - Launch the rviz window and the simulator environment by- `roslaunch f1tenth_gym_ros gym_bridge_ros.launch​`
 - Launch the localization node that uses particle filter algorithm by- `roslaunch  f1tenth_gym_ros localization.launch`
 - We also provide the opponent vehicle a constant velocity to be able to run code on the ego vehicle.
 - Finally we navigate to the directory where the Stanley controller python script is located and run it.​
