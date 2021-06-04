#!/usr/bin/env python3

# Qtable that stores the Q values for all encountered states
from collections import defaultdict
from game import Game
import random

NO_ACTION = 9

class Qtable(object):
    def __init__(self):
        self.q_table = {} #keys are (hash_key,action) tuple
        self.default_value = 0

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
        key = (state, o) if a == NO_ACTION else (state, a)
        if not key in self.q_table:
            return self.default_value
        return self.q_table[key]

    # get q value for existing game state, otherwise add state to table
    def get_q_value_from_key(self, key):
        if not key in self.q_table:
            self.q_table[key] = self.default_value
        #elif self.q_table[key] != 0:
        #   print("found state with q value " + str(self.q_table[key]))
        return self.q_table[key]

    # find existing keys for (current state + valid action)
    def find_existing_keys(self, game):
        state = self.hash_key(game.hash_board)
        # get keys for all possible actions
        moves = game.get_valid_moves()
        new_keys = [(state, x) for x in moves]

        # keep those already in the qtable
        new_keys_exist = [nk for nk in new_keys if nk in self.q_table]
        return new_keys_exist

    # return the max q value for all possible actions
    def get_max_q(self, game):
        new_keys_exist = self.find_existing_keys(game)
        if not new_keys_exist:
            return 0
        # get q values for these states
        qvals = [self.get_q_value_from_key(nk) for nk in new_keys_exist]
        return max(qvals)

    # return the min q value for all possible actions
    def get_min_q(self, game):
        new_keys_exist = self.find_existing_keys(game)
        if not new_keys_exist:
            return 0
        # get q values for these states
        qvals = [self.get_q_value_from_key(nk) for nk in new_keys_exist]
        return min(qvals)

    # get 1st move with the target q value for a give game state
    def get_move_from_q(self, game, q):
        existing_keys = self.find_existing_keys(game)
        for k in existing_keys:
            if self.q_table[k] == q:
                return k[1] # action
        # otherwise choose random move
        moves = game.get_valid_moves()
        move = random.choice(moves)
        return move

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

    # check if we encounters a new entry
    def new_entry(self,key):
        return not key in self.value

    # get value from table, if a new state is encountered, hash it with default value
    def get_value(self,key):
        if not key in self.value:
            self.value[key] = self.default_value #TODO: need to include no action?
        return self.value[key]