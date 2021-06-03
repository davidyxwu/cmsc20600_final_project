#!/usr/bin/end python3

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

class RDPlayer(object):
    def __init__(self):
        self.initialized = False
        rospy.init_node("blue_random_player")

        # initialize action publisher
        self.action_pub = rospy.Publisher("/tictactoe/blue_action", Action, queue_size=10)

        # initialize action
        self.action = Action()
        self.action.player = PLAYER_SYMBOL[PLAYER_BLUE]

        # subscribe to game_state publisher
        self.game_state_sub = rospy.Subscriber("/tictactoe/gamestate", GameState, self.game_state_callback)

        # whether it's the player's turn
        self.is_active_player = False

        # initialize game
        self.game = Game()

        self.initialized = True

    # check whether it's the player's turn
    def game_state_callback(self, data):
        if not self.initialized:
            return

        # check whether X is the active player
        if data.current_player == PLAYER_SYMBOL[PLAYER_BLUE]:
            self.is_active_player = True
        else:
            self.is_active_player = False

        # update board
        self.game.board[data.last_move] = PLAYER_RED

    # publish action if it's the player's turn
    def publish_action(self):
        if not self.is_active_player:
            return

        # randomly select a valid action
        valid_actions = [index for index, elmt in enumerate(self.game.board) if elmt==0]
        random_position = random.choice(valid_actions)
        self.action.position = random_position

        self.action_pub.publish(self.action)
