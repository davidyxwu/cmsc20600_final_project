# Turtlebot Tic-Tac-Toe!
## Team Members:
- Tianle Liu
- David Wu 
- Oscar Michel
## Project Description
### Motivation
We all grew up loving Tic-Tac-Toe and thought it would be a reasonable challenge given what we have learned in class (robot localization, perception and movement, learning). We were also interested in applying learning towards a 2-player game, rather than the 1-player game that we did in Q-learning through multi agent reienforcement learning. We thought Tic-Tac-Toe would be a great fit because like the Q-learning project, the states are discontinuous and finite (though there are a lot of them). 
### Objectives
- Implement a gazebo simulation that has a turtlebot robot play tic tac toe moves recieved from nodes.
- Implement a trained player from a Q-learning mini-max framework.
- Implement robot navigation, movement, and kinematics to control the game scene.
### High Level Description
TODO
### Learning (MARL)
The algorithm is a variant of the standard Q-learning algorithm used in the last project. In a two player game like Tic-Tac-Toe, winning and losing can be formulated as maximizing and minimizing a reward function. This characterization lends itself naturally to the stategy analysis of Q learning. This time however, states are valued in a min-max framework where the maximizing player tries to maximize its reward assuming optimal play from the minimizing player. 

### Navigation/SLAM
TODO
### Movement and Kinematics
TODO
### Node and topic communication
TODO

## Project Architecture
Description of each script:

```train.py```: The training of the Q-learning algorithm takes place in this script. A Q-matrix hash table is loaded, and games are repeatedly simulated and Q-values are updated throughout according to the markov game algorithm. The gameplay simulation uses game.py to hold the game state, generate valid moves at each step, and update the moves of the game.

```q_table.py```: This class stores the Q-matrix hash table. Instead of loading every state at once, we instead add states to a hash table on-the-fly as they are encountered during training. Game states are sent in the form (s,a,o) where s is the board state, a the max player's action, and o is the min player's action. Exactly one of a or o will be ```NO_ACTION``` which means that the player is not taking a move on that turn. 

```game.py```: This class is used for simulating a game, generating valid moves, checking whether the game has ended, and sending rewards at the end of the game. The game board is represented as a 9-element array which takes values of 0, 1, or 2, where 0 represents an empty square, 1 an O, and 2 an X. A move is represented as an integer 0-8, the index of the empty cell to be moved in. 

## Challenges
- It is very challenging to get the Q-matrix to converege, and diagnose bugs in the learning algorithm. It is hard to tell whether there is a problem with the algorithm itself, or whether the training is not converging. Additionally, hyperparameter optimization is challending since a lot of trial and error is involved.
- Setting up scenes and launch files is hard. The Gazebo world we built took a long time to build, and was carefully constructed to take into account SLAM mapping, robot navigation, etc. Further, setting up launch files for all our nodes was at times confusing. This was especially evident when we tried to implement two robots. When doing this, everything we tried seemed to fail! We were eventually able to get almost everything working, except that the robot gripper was buggy and malfunctioning. After scouring the internet for sources and trying multiple fixes, each to no avail, we decided to focus on optimizing the one robot approach.  

## Future Work 
- If we had more time, we would definitely want to try to actually implement two robots. The big bug we have with the two robot implementation is the robot arm kinematics. In this included gif, the `gripper_sub` component fluctuates, and the PID control for the move group was not set (we got an error but were unsure how to proceed). Given this, we believe we are very close, and would eventually be able to succeed. 

## Takeaways
- The robotics community is great! There are a lot of resources out there, and it is amazing to see such a commitment to an open source project. That being said, this makes debugging problems very hard. For example, we tried searching for countless resources about multi robot navigation and kinematics, and found that many of them did not work for us. Further, some topics such as examples of multiple turtlebots with arms were super hard to find. Overall, robotics is an exciting field that has unlimited potential!

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

#### SLAM mapping 
`roscore`\
`roslaunch cmsc20600_final_project slam.launch`\
`roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping`\
`roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch`
