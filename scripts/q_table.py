#!/usr/bin/env python3

# Qtable that stores the Q values for all encountered states
from collections import defaultdict

from game import Game 

class Qtable(object):
    def __init__(self):
        self.q_table = {} #keys are (hash_key,action) tuple
        self.default_value = 0.0

    # generate hash key for game state 
    def hash_key(self, board):
        key = 0 
        for i in range(board):
            key += board[i] * (3 ** i)
        return key

    # check whether the state exists in the hashtable 
    def new_entry(self, board, a,o):
        key = self.hash_key(board)
        return not (key,a,o) in self.q_table

    # get q value for existing game state, otherwise add state to table
    def get_q_value(self, board,a,o):
        key = self.hash_key(board)
        if self.new_entry(board, a,o):
            self.q_table[key,a,o] = self.default_value

        return self.q_table[key,a,o]

    # update q value for a game state 
    def set_q_value(self, board,a,o, q):
        key = self.hash_key(board)
        self.q_table[key,a,o] = q
        return

    def size(self):
        return len(self.q_table)

class Policy(object):

    def __init__(self):
        self.policy = {}
        self.default_value = 1.0/10.0

    # generate hash key for game state
    def hash_key(self, board):
        key = 0
        for i in range(board):
            key += board[i] * (3 ** i)
        return key

    def new_entry(self,board):
        key = self.hash_key(board)
        return not key in self.policy

    def get_policy_value(self,board,a):
        key = self.hash_key(board)
        if self.new_entry(board):
            self.policy[key] = [self.default_value]*10 #TODO: need to include no action?
        return self.policy[key][a]

    def set_policy_value(self,board,a,p):
        key = self.hash_key(board)
        if self.new_entry(board):
            self.policy[key] = [self.default_value] * 10  # TODO: need to include no action?
        self.policy[key][a] = p







    



