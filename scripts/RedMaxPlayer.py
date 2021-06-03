#!/usr/bin/env python3

# Max Tic-Tac-Toe player (an action node that publishes actions for robot1 - PLAYER_red)

import rospy
from game import Game, PLAYER_BLUE, PLAYER_RED, PLAYER_SYMBOL
from q_table import Qtable as Q
import os
import random
from cmsc20600_final_project.msg import Action, BoardRow, GameState

# convert action (0~8) to position on the game board
def position(action):
    row = action % 3
    col = action - 3 * row
    return (row, col)

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

class RedMaxPlayer(object):
    def __init__(self):
        self.initialized = False
        rospy.init_node("red_max_player")

        # initialize action publisher
        self.action_pub = rospy.Publisher("/tictactoe/red_action", Action, queue_size=10)

        # subscribe to game_state publisher
        self.game_state_sub = rospy.Subscriber("/tictactoe/gamestate", GameState, self.game_state_callback)

        # whether it's the player's turn
        self.is_active_player = False

        # initialize game
        self.game = Game()

        self.game_node_ready = False

        # get qtable
        self.Q = Q()
        self.Q.q_table = read_qtable()

        self.initialized = True
        print("Red initialized!")

    # check whether it's the player's turn
    def game_state_callback(self, data):
        print("recieved game state in red")
        if not self.initialized:
            return
        if data.game_end:
            return
        # check whether red is the active player
        if data.curr_player == PLAYER_RED:
            self.is_active_player = True
            print("Set red active player")
        else:
            self.is_active_player = False

        # update board
        if self.game_node_ready:
            assert(self.game.player == PLAYER_BLUE)
            self.game.move(data.last_move)
            self.publish_action()
        else:
            self.game_node_ready = True
            self.publish_action()

    # publish action if it's the player's turn
    def publish_action(self):
        print("In red publish action", self.is_active_player)
        if not self.is_active_player:
            return

        # get action with red q value
        q = self.Q.get_max_q(self.game)
        action = self.Q.get_move_from_q(self.game, q)

        print("Selecting best move for red", action)
        assert(self.game.player == PLAYER_RED)
        self.game.move(action)
        self.action_pub.publish(Action(player = PLAYER_RED, position = action))

    def run(self):
        # Keep the program alive.
        rospy.spin()

if __name__ == '__main__':
    # Declare a node and run it.
    node = RedMaxPlayer()
    node.run()
