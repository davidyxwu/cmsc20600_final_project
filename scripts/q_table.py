#!/usr/bin/env python3

# Qtable that stores the Q values for all encountered states 

from game import Game 

class Qtable(object):
    def __init__(self):
        self.q_table = {}
        self.size = 0 

    # generate hash key for game state 
    def hash_key(self, board):
        key = 0 
        for i in range(board):
            key += board[i] * (3 ** i)
        return key

    # check whether the state exists in the hashtable 
    def new_state(self, key):
        if key in self.q_table:
            return False
        return True

    # get q value for existing game state, otherwise add state to table
    def get_q_value(self, board, reward):
        key = self.hash_key(board)
        if self.new_state(key):
            self.q_table[key] = reward
            self.size += 1
            return reward
        else:
            return self.q_table[key]

    # update q value for a game state 
    def update_q_value(self, board, q):
        key = self.hash_key(board)
        self.q_table[key] = q 
        return 
    



