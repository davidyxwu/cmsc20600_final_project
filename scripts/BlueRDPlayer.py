#!/usr/bin/env python3

# Random Tic-Tac-Toe player (an action node that publishes actions for robot2 - PLAYER_BLUE)

import rospy
from game import Game, PLAYER_BLUE, PLAYER_RED, PLAYER_SYMBOL
import random
from cmsc20600_final_project.msg import Action, BoardRow, GameState

# convert action (0~8) to position on the game board
def position(action):
    row = action % 3
    col = action - 3 * row
    return (row, col)

"""This class is for a random player for blue"""
class BlueRDPlayer(object):
    def __init__(self):
        self.initialized = False
        rospy.init_node("blue_random_player") # init node

        # initialize action publisher
        self.action_pub = rospy.Publisher("/tictactoe/blue_action", Action, queue_size=10)

        # subscribe to game_state publisher
        self.game_state_sub = rospy.Subscriber("/tictactoe/gamestate", GameState, self.game_state_callback)

        # whether it's the player's turn
        self.is_active_player = False

        # initialize game (matrix representation for simplicity)
        self.game = [0] * 9

        self.game_node_ready = False # Helps keep track of first move

        self.initialized = True
        print("Blue initialized!")

    # Callback for game state
    def game_state_callback(self, data):
        print("recieved game state in blue")
        # Return if not initialized or game ended
        if not self.initialized:
            return
        if data.game_end:
            return
        # check whether Blue is the active player
        if data.curr_player == PLAYER_BLUE:
            self.is_active_player = True
            print("Set blue active player")
        else:
            self.is_active_player = False

        # update board
        if self.game_node_ready:
            # Game is underway! Update the last move from red
            self.game[data.last_move] = PLAYER_RED
            self.publish_action()
        else:
            # Game node was not ready, must have been first move
            self.game_node_ready = True
            self.publish_action()

    # publish action if it's the player's turn
    def publish_action(self):
        print("In blue publish action", self.is_active_player)
        if not self.is_active_player:
            return

        # randomly select a valid action
        valid_actions = [index for index, elmt in enumerate(self.game) if elmt==0]
        random_position = random.choice(valid_actions)
        print("Selecting random move for blue", random_position)
        self.game[random_position] = PLAYER_BLUE
        self.action_pub.publish(Action(player = PLAYER_BLUE, position = random_position))

    def run(self):
        # Keep the program alive.
        rospy.spin()

if __name__ == '__main__':
    # Declare a node and run it.
    node = BlueRDPlayer()
    node.run()
