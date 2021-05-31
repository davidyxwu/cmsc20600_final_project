#!/usr/bin/env python3

# Qtable that stores the Q values for all encountered states
from collections import defaultdict

from game import Game 

class Qtable(object):
    def __init__(self):
        self.q_table = {} #keys are (hash_key,action) tuple
        self.default_value = 50.0

    # generate hash key for game state 
    def hash_key(self, board):
        key = 0 
        for i in range(len(board)):
            key += board[i] * (3 ** i)
        return key

    # get q value for existing game state, otherwise add state to table
    def get_q_value(self, key,a,o):
        if not (key,a,o) in self.q_table:
            self.q_table[key,a,o] = self.default_value
        return self.q_table[key,a,o]

    def size(self):
        return len(self.q_table)

class Value(object):

    def __init__(self):
        self.value = {}
        self.default_value = 1.0

    # generate hash key for game state
    def hash_key(self, board):
        key = 0
        for i in range(board):
            key += board[i] * (3 ** i)
        return key

    def new_entry(self,key):
        return not key in self.value

    def get_value(self,key):
        if not key in self.value:
            self.value[key] = self.default_value #TODO: need to include no action?
        return self.value[key]








    



