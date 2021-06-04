# Turtlebot Tic-Tac-Toe!
## Team Members:
- Tianle Liu
- David Wu 
- Oscar Michel
## Project Description
### Motivation
We all grew up loving Tic-Tac-Toe and thought it would be a reasonable challenge given what we have learned in class (robot localization, perception and movement, learning). We were also interested in applying learning towards a 2-player game, rather than the 1-player game that we did in Q-learning through multi agent reinforcement learning. We thought Tic-Tac-Toe would be a great fit because like the Q-learning project, the states are discontinuous and finite (though there are a lot of them). 
### Objectives
- Implement a gazebo simulation that has a turtlebot robot play tic tac toe moves received from nodes.
- Implement a trained player from a Q-learning mini-max framework.
- Implement robot navigation, movement, and kinematics to control the game scene.
### High Level Description
Using a Q-learning minimax algorithm, two players will play tic-tac-toe, with their actions simulated by a turtlebot in a gazebo world.
![Tic-tac-toe world](https://github.com/davidyxwu/cmsc20600_final_project/blob/main/media/grid_numbering.jpg)
### Learning (MARL)
The algorithm is a variant of the standard Q-learning algorithm used in the last project. In a two player game like Tic-Tac-Toe, winning and losing can be formulated as maximizing and minimizing a reward function. This characterization lends itself naturally to the strategy analysis of Q learning. This time however, states are valued in a min-max framework where the maximizing player tries to maximize its reward assuming optimal play from the minimizing player. 

### Navigation/SLAM
To perform navigation, we first had to create a map of the gazebo world using SLAM. At first, since our world is so wide and empty, the dimensions of the map were not matching up at all as we drove our turtlebot around the room. Our solution was to add more landmarks for the robot to scan, which is why all the stop signs act as pole markers on the grid (this also makes the world look cooler). \
Once we got a map, we were able to follow tutorials about [the navigation stack](http://wiki.ros.org/navigation) and [implementing this in python](https://edu.gaitech.hk/turtlebot/map-navigation.html) with an action client. In essence, the important nodes for navigation include AMCL to track the robot's position with a particle filter, and move_base to set navigation goals while avoiding obstacles. Lastly, we use the `init_pose` function to publish the robot's initial pose relative to the map frame so the robot knows where it is to begin with.

![Slam process](https://github.com/davidyxwu/cmsc20600_final_project/blob/main/media/slam_3x.gif)

Robot performing SLAM

![slam map](https://github.com/davidyxwu/cmsc20600_final_project/blob/main/media/map.png) 

Slam map

![one robot navigation](https://github.com/davidyxwu/cmsc20600_final_project/blob/main/media/rviz_single_3x.gif) 

One robot navigation

![two robot navigation](https://github.com/davidyxwu/cmsc20600_final_project/blob/main/media/rviz_two_3x.gif)

Two robot navigation

### Movement and Kinematics
A large part of robot movement is done by the navigation stack in the function `move_grid(xy)`. Using the `SimpleActionClient`, we are able to send goal requests to the robot's `move_base` topic that allows it to move towards a goal which we define. The rest of the movement is done in `move_to_dumbbell`. Here, we keep track of the center `[cx]` of the dumbbell color we want to visit. Further, we make sure the dumbbell we see is already not placed by rejecting images that have green in them (color of grid). With this, we use PID to slowly move toward the dumbbell and pick it up. 
Picking up the dumbbell is done in the `reset_gripper` and `pick_up_gripper` functions. In `reset_gripper`, we set the arm and gripper to a state where the robot can move the gripper on the handle of the dumbbell to pick it up. In `pick_up_gripper` and `drop_gripper`, we move the arm and gripper to pick up and put down the dumbbell respectively. We found appropriate values for the arm and gripper for both these functions by playing with the GUI and testing through observation to see what worked and what didn't work.

![kinematics](https://github.com/davidyxwu/cmsc20600_final_project/blob/main/media/kinematics_2x.gif)

Robot picking up dumbbell

### Node and topic communication
There are 4 agents involved: the game state manager, the red player, the blue player, and the robots. The robot first communicates to the game state manager using the `/nodestatus` topic to tell everyone it is ready to go. The game state manager then sends an initial game state through the `/tictactoe/gamestate` topic, telling the red and blue players that they can begin moving. The red and blue players make moves, each based on an updated game state and publish to `/tictactoe/red_action` and `/tictactoe/blue_action` respectively. The robot and game state manager listen to these messages to make moves.

![nodes](https://github.com/davidyxwu/cmsc20600_final_project/blob/main/media/node_diagram.png)

State diagram between nodes

## System Architecture (Description of each script)
### Training and game engine 
`train.py`: The training of the Q-learning algorithm takes place in this script. A Q-matrix hash table is loaded, and games are repeatedly simulated and Q-values are updated throughout according to the markov game algorithm. The gameplay simulation uses game.py to hold the game state, generate valid moves at each step, and update the moves of the game.

`q_table.py`: This class stores the Q-matrix hash table. Instead of loading every state at once, we instead add states to a hash table on-the-fly as they are encountered during training. Game states are sent in the form (s,a,o) where s is the board state, a the max player's action, and o is the min player's action. Exactly one of a or o will be ```NO_ACTION``` which means that the player is not taking a move on that turn. 

`game.py`: This class is used for simulating a game, generating valid moves, checking whether the game has ended, and sending rewards at the end of the game. The game board is represented as a 9-element array which takes values of 0, 1, or 2, where 0 represents an empty square, 1 an O, and 2 an X. A move is represented as an integer 0-8, the index of the empty cell to be moved in. 

### Ros node communication

`BlueMinPlayer.py`, `RedMaxPlayer`: These classes use the Q-matrix to select moves based on the other player, and send them to the robot. It publishes messages to `/tictactoe/red_action` and `tictactoe/blue_action` topics to inform the robot and game state manager of their moves.

`BlueRDPlayer.py`, `RedRDPlayer`: These classes randomly selects moves and send them to the robot. It publishes messages to `/tictactoe/red_action` and `tictactoe/blue_action` topics to inform the robot and game state manager of their moves.

`GameNode.py`: This class acts as the game state manager in gazebo. It keeps track of the current board state by receiving messages from the two players, and then publishes the updated board so both players know what is going on. It publishes messages to `/tictactoe/gamestate` for the two players.

### One Robot Action

`one_robot_action.py`: This class acts as the engine for the one robot moving dumbbells to grid locations. Using a set of states in the `run()` function, this script manages all the movement and kinematics described above. It communicates with the `/tictactoe/red_action` and `tictactoe/blue_action` topics regarding actions.

### Two Robot Action

`red.py`, `blue.py`: This class resembles `one_robot_action.py` in their design. They differ in the topics each node is subscribed to in order to match namespaces. Currently, they has not been updated to the final version as our two robot action is bugged.

## Challenges
- It is very challenging to get the Q-matrix to converge, and diagnose bugs in the learning algorithm. It is hard to tell whether there is a problem with the algorithm itself, or whether the training is not converging. Additionally, hyper-parameter optimization is challenging since a lot of trial and error is involved.
- Setting up scenes and launch files is hard. The Gazebo world we built took a long time to build, and was carefully constructed to take into account SLAM mapping, robot navigation, etc. Further, setting up launch files for all our nodes was at times confusing. This was especially evident when we tried to implement two robots. When doing this, everything we tried seemed to fail! We were eventually able to get almost everything working, except that the robot gripper was buggy and malfunctioning. After scouring the web for sources and trying multiple fixes, each to no avail, we decided to focus on optimizing the one robot approach.  

## Future Work 
- If we had more time, we would definitely want to try to actually implement two robots. The big bug we have with the two robot implementation is the robot arm kinematics. In this included gif, the `gripper_sub` component fluctuates, and the PID control for the move group was not set (we got an error but were unsure how to proceed). Given this, we believe we are very close, and would eventually be able to succeed. 

## Takeaways
- The robotics community is great! There are a lot of resources out there, and it is amazing to see such a commitment to an open source project. That being said, this makes debugging problems very hard. For example, we tried searching for countless resources about multi robot navigation and kinematics, and found that many of them did not work for us. Further, some topics such as examples of multiple turtlebots with arms were super hard to find. Overall, robotics is an exciting field that has unlimited potential!

## Recordings
### Blue min vs random
![blue min vs random](https://github.com/davidyxwu/cmsc20600_final_project/blob/main/media/blue_min_red_random_10x.gif)

### Red max vs random
![red max vs random](https://github.com/davidyxwu/cmsc20600_final_project/blob/main/media/red_max_blue_random_10x.gif)

### Random vs random
![random vs random](https://github.com/davidyxwu/cmsc20600_final_project/blob/main/media/random_10x.gif)



### Two robot world gripper bug
![bad gripper](https://github.com/davidyxwu/cmsc20600_final_project/blob/main/media/bad_gripper.gif)

## Documentation for launch files
### Launch Instructions:
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
