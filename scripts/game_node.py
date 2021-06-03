#!/usr/bin/env python3

"""This node keeps track of the game state between two players.
    The two players will publish their moves into this node, and this node
    will publish game states"""

import rospy
import numpy as np
from cmsc20600_final_project.msg import GameState, Action
from game import Game, PLAYER_BLUE, PLAYER_RED, PLAYER_SYMBOL

class GameNode(object):

    def __init__(self):
        self.initialized = False

        rospy.init_node("GameNode")
        rospy.sleep(1)

        # Set up gamestate publisher
        self.gamestate_pub = rospy.Publisher("/tictactoe/gamestate", GameState, queue_size=10)

        # Subscribe to blue robot action
        self.blue_action_sub = rospy.Subscriber("/tictactoe/blue_action", self.blue_action_callback)

        # Subscribe to red robot action
        self.red_action_sub = rospy.Subscriber("/tictactoe/blue_action", self.blue_action_callback)

        self.game = Game()

        self.initialized = True

        rospy.sleep(1)

        self.gamestate_pub.publish(GameState(last_player=PLAYER_BLUE, curr_player=PLAYER_RED, board=self.game.board))


    def blue_action_callback(self, data):
        if not self.initialized:
            return

        if data:
            self.update_game(data.position)

    def red_action_callback(self, data):
        if not self.initialized:
            return

        if data:
            self.update_game(data.position)

    def update_game(self, position):
        last_player = self.game.player
        self.game.move(position)
        self.gamestate_pub.publish(GameState(last_player=last_player, curr_player=self.game.player, board=self.game.board))
