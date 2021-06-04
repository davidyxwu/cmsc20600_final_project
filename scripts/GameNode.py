#!/usr/bin/env python3

"""This node keeps track of the game state between two players.
    The two players will publish their moves into this node, and this node
    will publish game states"""

import rospy
import numpy as np
from cmsc20600_final_project.msg import GameState, Action, RobotInitialized
from game import Game, PLAYER_BLUE, PLAYER_RED, PLAYER_SYMBOL

"""This class implements the Game Node, the game engine in gazebo"""
class GameNode(object):

    def __init__(self):
        # Initialization
        self.initialized = False

        rospy.init_node("GameNode")
        rospy.sleep(1)

        # Set up gamestate publisher
        self.gamestate_pub = rospy.Publisher("/tictactoe/gamestate", GameState, queue_size=10)

        # Subscribe to blue robot action
        self.blue_action_sub = rospy.Subscriber("/tictactoe/blue_action", Action, self.blue_action_callback)

        # Subscribe to red robot action
        self.red_action_sub = rospy.Subscriber("/tictactoe/red_action", Action, self.red_action_callback)

        # Set up game
        self.game = Game()

        # Keep track of actions recieved
        self.actions = []

        # Subscribe to node_status
        self.action_node_status = rospy.Subscriber("/tictactoe/node_status", RobotInitialized, self.node_status_callback)

        self.initialized = True
        print("Game node initialized!")

    """Listen to robot initialization status and send starting board state to players"""
    def node_status_callback(self, data):
        print("recieved node status")
        self.gamestate_pub.publish(GameState(last_player=PLAYER_BLUE, curr_player=PLAYER_RED, board=self.game.board))

    """Callback function for blue player actions"""
    def blue_action_callback(self, data):
        print("Received blue action")
        if not self.initialized:
            return
        self.actions.append(data)
        self.update_game(data.position)

    """Calback function for red player actions"""
    def red_action_callback(self, data):
        print("Received red action")
        if not self.initialized:
            return
        self.actions.append(data)
        self.update_game(data.position)

    """Update the game based on the actions recieved, publish new game state"""
    def update_game(self, position):
        print(self.actions)
        last_player = self.game.player
        self.game.move(position)
        self.gamestate_pub.publish(GameState(last_player=last_player, curr_player=self.game.player,
                                    last_move=position, game_end=self.game.check_for_winner(), board=self.game.board))
        print("In update game, board:", self.game.board, self.game.check_for_winner())

    def run(self):
        # Keep the program alive.
        rospy.spin()

if __name__ == '__main__':
    # Declare a node and run it.
    node = GameNode()
    node.run()