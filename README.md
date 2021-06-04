# Turtlebot Tic-Tac-Toe!
## Team Members:
- Tianle Liu
- David Wu 
- Oscar Michel
## Project Description
### Motivation
We all grew up loving Tic-Tac-Toe and thought it would be a reasonable challenge given what we have learned in class (robot localization, perception and movement, learning). We were also interested in applying learning towards a 2-player game, rather than the 1-player game that we did in Q-learning through multi agent reienforcement learning. We thought Tic-Tac-Toe would be a great fit because like the Q-learning project, the states are discontinuous and finite (though there are a lot of them). 
### Objectives
TODO
### High Level Description
TODO
### Learning (MARL)
TODO
### Navigation/SLAM
TODO
### Movement and Kinematics
TODO
### Node and topic communication
TODO

## Project Architecture
List each script, say what it does and what robotics algorithm we used.

## Challenges 

## Future Work 

## Takeaways

## Recordings

## Documentation for launch files
### Tentative Launch Instructions:
#### Two Robots (Very Buggy)
`roscore`\
`roslaunch cmsc20600_final_project two_robots_world.launch`\
`roslaunch cmsc20600_final_project blue_tb3_navigation.launch`\
`roslaunch cmsc20600_final_project red_tb3_navigation.launch`\
`roslaunch cmsc20600_final_project two_robots_action.launch`

#### One Robot
`roscore`\
`roslaunch cmsc20600_final_project one_robot_world.launch`\
`roslaunch cmsc20600_final_project one_robot_navigation.launch`\
`roslaunch cmsc20600_final_project one_robot_action.launch`\
Change nodes in one_robot_world to match the appropriate player node.
