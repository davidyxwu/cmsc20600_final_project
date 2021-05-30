#!/usr/bin/env python3
import numpy as np
# Tic-tac-toe game policy and Q matrix implementation

from random import randint

NO_PLAYER = 0
PLAYER_O = 1
PLAYER_X = 2
PLAYER_SYMBOL = {PLAYER_O : 'O', PLAYER_X : 'X', 0: '_'}

WIN_REWARD = 100.0 # with respect to player O
TIE_REWARD = 50.0
LOSS_REWARD = 0.0
OTHER_REWARD = 0.0

class Game(object):
    def __init__(self):
        self.board = [0] * 9 # 3X3 board in row-major order 
        self.player = PLAYER_O # current player 
        self.open_grid = 9 # count number of open grids
        self.last_move = -1

    def __str__(self):
        print("Current player: " + PLAYER_SYMBOL[self.player])
        print("Current board: ")
        str = ''
        for row in range(3):
            line = ''
            for column in range(3):
                line += PLAYER_SYMBOL[self.board[row*3 + column]] + " "
            line += '\n'
            str += line
        return str

    # check whether the current player has won
    def has_won(self, grid): 
        win_val = self.player * 3

        # check column
        sum = 0 
        for i in range(3):
            pos = (grid + 3*i)%9
            if self.player == self.board[pos]:
                sum += self.board[pos]
        if sum == win_val:
            return True

        # check row
        sum = 0 
        row = grid // 3
        for i in range(3):
            pos = row*3+i
            if self.player == self.board[pos]:
                sum += self.board[row*3+i]
        if sum == win_val:
            return True

        # check diagonals
        sum1 = 0
        for i in [0,4,8]:
            if self.player == self.board[i]:
                sum1 += self.board[i]
        sum2 = 0
        for i in [2,4,6]:
            if self.player == self.board[i]:
                sum2 += self.board[i]
        if sum1 == win_val or sum2 == win_val:
            return True

        # no winner yet
        return False 

    # check whether the game ends after current move 
    def game_end(self, grid=-1): #by defualt use last move
        if grid==-1:
            grid = self.last_move
        if self.has_won(grid) or self.open_grid == 0:
            return True
        return False

    # check if move is valid 
    def valid_move(self, grid):
        # if the grid has not been occupied 
        if self.board[grid] == 0:
            return True
        return False

    def get_valid_moves(self):
        return [i for i in range(len(self.board)) if self.valid_move(i)]
     
    # make move   
    def move(self, grid):
        assert self.board[grid] == 0
        # update board 
        self.board[grid] = self.player
        # update open grid count
        self.open_grid -= 1
        # set next player
        if self.player == PLAYER_O:
            self.player = PLAYER_X
        else:
            self.player = PLAYER_O

        self.last_move = grid
        return 

    # return reward for each move 
    def reward(self, grid):
        if self.has_won(grid):
            #print("Player " + PLAYER_SYMBOL[self.player]+ "has won!")
            # return reward 
            if self.player == PLAYER_O:
                return WIN_REWARD
            else:
                return LOSS_REWARD
        elif self.game_end(grid): 
            return TIE_REWARD
        else:
            return OTHER_REWARD    

            




    


