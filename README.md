## Navigation Package

Code for autonomous navigation of a prototype Mars Rover, using RRT* algorithm. Simulations were done using turtlebot on gazebo. The code is designed such that your rover will avoid obstacles and it will remember the position of all obstacles it encounters.
This code was implemented on the rover 'Kratos' in IRC 2020
To use this code:-
1. Clone this repo
2. Launch turtlebot on gazebo.
3. Enter 'roslaunch navigation controller.launch' to your terminal
4. Publish your goal coordinates using 'rostopic pub /goal2 Point_xy '[x,y]'
 (where x and y are your goal coordinates)
5. You may use GPS coordinates to determine the goal if you wish, to do so, publish your bot's current GPS on /gps_bot, and the goal on /gps_goal
