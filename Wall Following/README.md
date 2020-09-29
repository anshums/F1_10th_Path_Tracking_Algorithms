# Wall Following

<p align = "center">
<img src = "https://github.com/prateeks97/F1_10th_Path_Tracking_Algorithms/blob/master/Wall%20Following/Wall_following.gif">
</p>

## Implementation Details

 - The code was implemented on f1tenth_gym_ros. The purpose of implementing this code was to get familiarized with the ros topics used to publish and subscribe and learn more about the scan data etc.
 - One finding from implementing the code was that the laser scan moves from -135 to 135 degrees ad have 1080 readings.
 - There was one reading for every 0.25 degrees and we don’t need that many readings for this code to work efficiently so when implementing this code only 1 in 4 reading from laser scan was obtained i.e. one reading for every degree scanned.
 - •Also the vehicle starts to slip at higher speeds so the velocity was kept at 4 m/sec.

## Steps to setup Environment and run the code

 - First download the [f1tenth_gym_ros](https://github.com/f1tenth/f1tenth_gym_ros) which is a containerized ROS communication bridge for F1TENTH gym environment. Just follow the steps given on the github page.
 - Then launch the simulator by-  `./docker.sh` file present in the simulator repository.​
 - Launch the rviz window and the simulator environment by- `roslaunch f1tenth_gym_ros gym_bridge_ros.launch​`
 - We also provide the opponent vehicle a constant velocity to be able to run code on the ego vehicle.
 - Finally we navigate to the directory where the wall following python script is located and run it.​
