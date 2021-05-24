#!/usr/bin/env python3

# Tic-tac-toe game policy and Q matrix implementation

from random import randint

PLAYER_O = 1
PLAYER_X = 2
PLAYER_SYMBOL = {PLAYER_O : 'O', PLAYER_X : 'X'}

WIN_REWARD = 100 # with respect to player O 
TIE_REWARD = 50
LOSS_REWARD = 0 
OTHER_REWARD = 0 

class Game(object):
    def __init__(self):
        self.board = [0] * 9 # 3X3 board in row-major order 
        self.player = PLAYER_O # current player 
        self.open_grid = 9 # count number of open grids
    
    def __str__(self):
        print("Current player: " + PLAYER_SYMBOL[self.player])
        print("Current board: ")
        for row in range(3):
            for column in range(3):
                print(PLAYER_SYMBOL[self.board[row*3 + column]] + ", ", end="")
            print()
        return 

    # check whether the current player has won
    def has_won(self, grid): 
        win_val = self.player * 3

        # check column
        sum = 0 
        for i in range(3):
            self.board[(grid + 3*i)%9]
        if sum == win_val:
            return True

        # check row
        sum = 0 
        row = int(grid/3)
        for i in range(3):
            sum += self.board[row*3+i]
        if sum == win_val:
            return True

        # check diagonals
        sum1 = 0
        for i in [0,4,8]:
            sum1 += self.board[i]
        sum2 = 0
        for i in [2,4,6]:
            sum2 += self.board[i]
        if sum1 == win_val or sum2 == win_val:
            return True

        # no winner yet
        return False 

    # check whether the game ends after current move 
    def game_end(self, grid):
        if self.has_won(grid) or self.open_grid == 0:
            return True
        return False

    # check if move is valid 
    def valid_move(self, grid):
        # if the grid has not been occupied 
        if self.board[grid] == 0:
            return True
        return False
     
    # make move   
    def move(self, grid):
        # update board 
        self.board[grid] == self.player
        # update open grid count
        self.open_grid += -1 
        # set next player
        if self.player == PLAYER_O:
            self.player = PLAYER_X
        else:
            self.player = PLAYER_O
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

            




    


