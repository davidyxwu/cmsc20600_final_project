#!/usr/bin/env python3

# Min Tic-Tac-Toe player (an action node that publishes actions for blue robot- (PLAYER_BLUE)

import rospy
from game import Game, PLAYER_BLUE, PLAYER_RED, PLAYER_SYMBOL
from q_table import Qtable as Q
import os
import random
from cmsc20600_final_project.msg import Action, BoardRow, GameState

# Path of directory on where Qmatrix file is located
path_prefix_qmatrix = os.path.dirname(__file__)

 # read saved qtable into a dictionary
def read_qtable():
    dic = ''
    with open(path_prefix_qmatrix + '/qtable.txt','r') as f:
        for line in f.readlines():
            dic=line
    dic = eval(dic)
    return dic

"""This class is for the Blue min player"""
class BlueMinPlayer(object):
    def __init__(self):
        self.initialized = False
        rospy.init_node("blue_min_player") # initialize node

        # initialize action publisher
        self.action_pub = rospy.Publisher("/tictactoe/blue_action", Action, queue_size=10)

        # subscribe to game_state publisher
        self.game_state_sub = rospy.Subscriber("/tictactoe/gamestate", GameState, self.game_state_callback)

        # whether it's the player's turn
        self.is_active_player = False

        # initialize game
        self.game = Game()

        # Wait for game node to be ready (will get game state msg)
        self.game_node_ready = False

        # get qtable
        self.Q = Q()
        self.Q.q_table = read_qtable()

        self.initialized = True
        print("Blue initialized!")

    # check whether it's the player's turn
    def game_state_callback(self, data):
        print("recieved game state in blue")
        # Don't make move if not initialized or the game ended
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
            # Game in progress, update player red's move
            assert(self.game.player == PLAYER_RED)
            self.game.move(data.last_move)
            self.publish_action()
        else:
            # First message, empty board, Don't update player red's move
            self.game_node_ready = True
            self.publish_action()

    # publish action if it's the player's turn
    def publish_action(self):
        print("In blue publish action", self.is_active_player)
        # Make sure it is blue's turn
        if not self.is_active_player:
            return

        # get action with min q value
        q = self.Q.get_min_q(self.game)
        action = self.Q.get_move_from_q(self.game, q)

        print("Selecting best move for blue", action)
        assert(self.game.player == PLAYER_BLUE)
        # Make a move
        self.game.move(action)
        self.action_pub.publish(Action(player = PLAYER_BLUE, position = action))

    def run(self):
        # Keep the program alive.
        rospy.spin()

if __name__ == '__main__':
    # Declare a node and run it.
    node = BlueMinPlayer()
    node.run()
