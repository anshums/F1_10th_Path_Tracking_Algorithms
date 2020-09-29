# Follow the Gap

<p align = "center">
<img src = "https://github.com/prateeks97/F1_10th_Path_Tracking_Algorithms/blob/master/Obstacle%20Avoidance/images/Follow_the_gap.gif">
</p>

## Description
Follow the gap is a reactive method use for local navigation and to avoid static obstacles.

## Steps for Implementation

 - Find nearest LIDAR point and put a “safety bubble” around it of radius rb
 - Set all points inside bubble to distance 0. All nonzero points are considered ‘free space’
 - Find maximum length sequence of consecutive non zeros among the ‘free space’ points- the max gap
 - Find the ‘best’ point among this maximum length sequence
 - Choose the furthest point in free space, and set your steering angle towards it

<p align = "center">
<img src = "https://github.com/prateeks97/F1_10th_Path_Tracking_Algorithms/blob/master/Obstacle%20Avoidance/images/steps.JPG" width= "300" height="500">
</p>

For more details regarding Follow the gap method refer [this](https://f1tenth-coursekit.readthedocs.io/en/stable/lectures/ModuleB/lecture06.html#lecture-6-reactive-methods-follow-the-gap-variants).

## Steps to setup Environment and run the code

 - First download the [f1tenth_gym_ros](https://github.com/f1tenth/f1tenth_gym_ros) which is a containerized ROS communication bridge for F1TENTH gym environment. Just follow the steps given on the github page.
 - Then launch the simulator by-  `./docker.sh` file present in the simulator repository.​
 - Launch the rviz window and the simulator environment by- `roslaunch f1tenth_gym_ros gym_bridge_ros.launch​`
 - Launch the localization node that uses particle filter algorithm by- `roslaunch  f1tenth_gym_ros localization.launch`
 - We also provide the opponent vehicle a constant velocity to be able to run code on the ego vehicle.
 - Finally we navigate to the directory where the follow the gap python script is located and run it.​