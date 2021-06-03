#!/usr/bin/env python3

# Qtable that stores the Q values for all encountered states
from collections import defaultdict

from game import Game 

NO_ACTION = 9

class Qtable(object):
    def __init__(self):
        self.q_table = {} #keys are (hash_key,action) tuple
        self.default_value = 50.0

    def __str__(self):
        string = ""
        for key in self.q_table:
            string += "key: " + str(key[0]) + ", action: " + str(key[1]) 
            string += " reward: " +  str(self.q_table[key]) + "\n"
        return string

    # generate hash key for game state 
    def hash_key(self, board):
        key = 0 
        for i in range(len(board)):
            key += board[i] * (3 ** i)
        return key

    # get q value for existing game state, otherwise add state to table
    def get_q_value(self, state, a, o):
        if a == NO_ACTION:
            key = (state, o)
        else: # o == NO_ACTION
            key = (state, a)
        if not key in self.q_table:
            self.q_table[key] = self.default_value
        return self.q_table[key]

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








    



