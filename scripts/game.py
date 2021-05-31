#!/usr/bin/env python3

# Tic-tac-toe game policy and Q matrix implementation

from random import randint

PLAYER_O = 1
PLAYER_X = 2
PLAYER_SYMBOL = {PLAYER_O : 'O', PLAYER_X : 'X'}

WIN_REWARD = 1.0 # with respect to player O
TIE_REWARD = 0.0
LOSS_REWARD = -1.0
OTHER_REWARD = 0.0

# clockwise rotation of the board grids
rotation = {
    0:2, 2:8, 8:6, 6:0,
    1:5, 5:7, 7:3, 3:1, 
    4:4}

# diagonal flip
flip_diag = {
    0:0, 4:4, 8:8, 
    3:1, 6:2, 7:5, 1:3, 2:6, 5:7}

# vertical flip
flip_ver = {
    1:1, 4:4, 7:7,
    2:0, 5:3, 8:6, 0:2, 3:5, 6:8}


class Game(object):
    def __init__(self):
        self.board = [0] * 9 # 3X3 board in row-major order 
        self.last_move = -1 # track last move
        self.hash_board = self.board # board for hashing, optimized for first 2 steps  
        self.player = PLAYER_O # current player 
        self.open_grid = 9 # count number of open grids

        # operations (rotation/flip) for optimization
        self.operations = []

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

    def get_valid_moves(self):
        return [i for i in range(len(self.board)) if self.valid_move(i)]
     
    # make move   
    def move(self, grid):

        # if this is the first move
        if self.open_grid == 9:
            self.operations.append(self.move1_state_optimization(grid))
        # if this is the 2nd move
        if self.open_grid == 8:
            self.operations.append(self.move2_state_optimization(grid))

        # update board 
        self.board[grid] == self.player
        # update hash_board
        self.hash_board[self.transform_grid[grid]] = self.player
        # update open grid count
        self.open_grid += -1 
        # update last_move
        self.last_move = grid 
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

    # consider the boards to be the same w.r.t rotations, return number of rotations needed
    def move1_state_optimization(self, grid):
        rotation_cnt = {
            0:0, 1:0, 4:0,
            6:1, 3:1, 8:2, 7:2, 2:3, 5:3}
        return (rotation, rotation_cnt[grid])

    # consider the boards to be the same w.r.t certain rotations/flips, 
    def move2_state_optimization(self, grid):
        last_move_transformed = self.transform_grid(self.last_move)
       
        if last_move_transformed == 0:
            flip_cnt = {
                0:0, 1:0, 2:0, 4:0, 5:0, 8:0,
                3:1, 6:1, 7:1}
            return (flip_diag, flip_cnt[grid])

        elif last_move_transformed == 1:
            flip_cnt = {
                0:0, 3:0, 6:0, 1:0, 4:0, 7:0,
                2:1, 5:1, 8:1}
            return (flip_ver, flip_cnt[grid]) 
        # last_move_transformed == 4
        else:
            rotation_cnt = {
            0:0, 1:0, 4:0,
            6:1, 3:1, 8:2, 7:2, 2:3, 5:3}
            return (rotation, rotation_cnt[grid])

    # return grid number after transformations
    def transform_grid(self, grid):
        for op, cnt in self.operations:
            for i in cnt:
                grid = op[grid]
        return grid 
            






            




    


