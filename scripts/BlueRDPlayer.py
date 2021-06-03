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

class BlueRDPlayer(object):
    def __init__(self):
        self.initialized = False
        rospy.init_node("blue_random_player")

        # initialize action publisher
        self.action_pub = rospy.Publisher("/tictactoe/blue_action", Action, queue_size=10)

        # initialize action
        self.action = Action()
        self.action.player = "blue"

        # subscribe to game_state publisher
        self.game_state_sub = rospy.Subscriber("/tictactoe/gamestate", GameState, self.game_state_callback)

        # whether it's the player's turn
        self.is_active_player = False

        # initialize game
        self.game = [0] * 16

        self.game_node_ready = False

        self.initialized = True
        print("Blue initialized!")

    # check whether it's the player's turn
    def game_state_callback(self, data):
        if not self.initialized or not self.game_node_ready:
            return
        if data.game_end:
            return
        self.game_node_ready = True
        # check whether Blue is the active player
        if data.curr_player == PLAYER_SYMBOL[PLAYER_BLUE]:
            self.is_active_player = True
        else:
            self.is_active_player = False

        # update board
        self.game.board[data.last_move] = PLAYER_RED
        self.publish_action()

    # publish action if it's the player's turn
    def publish_action(self):
        if not self.is_active_player or not self.game_node_ready:
            return

        # randomly select a valid action
        valid_actions = [index for index, elmt in enumerate(self.game) if elmt==0]
        random_position = random.choice(valid_actions)
        self.game[random_position] = PLAYER_SYMBOL[PLAYER_BLUE]
        self.action.position = random_position
        self.action_pub.publish(self.action)

    def run(self):
        # Keep the program alive.
        rospy.spin()

if __name__ == '__main__':
    # Declare a node and run it.
    node = BlueRDPlayer()
    node.run()
