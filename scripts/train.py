#!/usr/bin/env python3

import random, time, sys
import numpy as np
from q_table import Qtable, Value
from game import Game, PLAYER_RED, PLAYER_BLUE

NO_ACTION = 9
PLAYER_MAX = PLAYER_RED
PLAYER_MIN = PLAYER_BLUE


class Train(object):

    def __init__(self):
        # variables for training algorithm
        self.alpha = 0.5  # 0.5
        self.gamma = 0.8

        # how likely the robot explores a new state during training
        self.explor = 0.25 

        # training status 
        self.batch_size = 1e4
        self.training_step = 0
        self.game_count = 0

        # start game
        self.game = Game()
        self.states = []
        
        # initialize value table
        self.V = Value()     
        # initialize Qtable
        self.Q = Qtable()

        # variables for checking training status
        self.qtable_size = 0
        self.qtable_size_increment = 0
        self.convergence_cnt = 0
        self.convergence_criteria = 4e4
        self.end_train = False

    # calculates value based on Qtable
    def value(self, key):  # max_a min_o q(s,a,o)

        max_q = - 2 ** 64  # small number

        for a in range(10):
            min_q = 2 ** 64  # large number
            for o in range(10):
                q = self.Q.get_q_value(key, a, o)
                if q < min_q:
                    min_q = q
            if max_q < min_q:
                max_q = min_q

        return max_q

    # picks a valid move and executes it,
    # returns an action tuple (a,o) for the 2 players
    def get_next_action(self):
        # print("get_next_action starts")
        # print(self.game)

        moves = self.game.get_valid_moves()
        # print(str(len(moves))+" available moves")
        indx = random.randint(0, len(moves) - 1)
        explore = np.random.binomial(1, self.explor) #this will determine whether agent explores (with prob self.explor)
        if explore:
            # if player O is active
            if self.game.player == PLAYER_MAX:
                a = moves[indx]
                o = NO_ACTION
                self.game.move(a)
                # print("1 - o move to "+str(a))
            # if player X is active
            else:
                a = NO_ACTION
                o = moves[indx]
                self.game.move(o)
                # print("2 - x move to "+str(o))
        else: #otherwise do min max stategy
            state = self.Q.hash_key(self.game.hash_board)

            # if player O is active
            if self.game.player == PLAYER_MAX:
                a = max(moves, key=lambda m: self.Q.get_q_value(state, m, NO_ACTION)) #find move w/ highest q val
                o = NO_ACTION
                self.game.move(a)
                # print("3 - o move to "+str(a))
            # if player X is active
            else:
                a = NO_ACTION
                o = min(moves, key=lambda m: self.Q.get_q_value(state, NO_ACTION, m)) #find move w/ lowest q val
                self.game.move(o)
                # print("4 - x move to "+str(o))

        # optimization for step 1 & 2
        if o == NO_ACTION:
            a = self.game.transform_grid(a)
        else:
            o = self.game.transform_grid(o)

        return a, o

    # execute training 
    def train(self):
        # start game
        self.game = Game()
        self.training_step = 0
        # train in batches of size 1e4
        while self.training_step < self.batch_size:
            prev_state = self.Q.hash_key(self.game.hash_board)
            a, o = self.get_next_action() #next moves
            next_state = self.Q.hash_key(self.game.hash_board)
            r = self.game.reward(a) if o == NO_ACTION else self.game.reward(o) #reward will come from current player

            q = self.Q.get_q_value(prev_state, a, o)
            q_new = int((1 - self.alpha) * q + self.alpha * (r + self.gamma * self.value(next_state))) #update formula
            key = (prev_state, a) if o == NO_ACTION else (prev_state, o)
            self.Q.q_table[key] = q_new #update q val
            self.V.value[prev_state] = self.value(prev_state) #set value of state
            # if q_new != 0 and q != 0 and q_new != q:
            # print("old_q: "+str(q)+", new: "+str(q_new))

            self.training_step += 1

            # check if the Q value stays the same
            if q == q_new:
                self.convergence_cnt += 1
            else:
                self.convergence_cnt = 0

            if self.convergence_cnt % 1000 == 0 and self.convergence_cnt != 0:
                print("convergence_cnt: " + str(self.convergence_cnt) + " ! ")
                print("train time: " + str(time.time() - self.start_time))

            # if the game ends restart the game
            if self.game.game_end():
                self.game = Game()
                # print("New Game!")
                # print(self.game)
                self.game_count += 1
                continue

            # if training reached convergence criteria 
            if self.ready_to_end():
                self.end_train = True

        # update values for convergence condition check
        self.qtable_size_increment = len(self.Q.q_table) - self.qtable_size
        self.qtable_size = len(self.Q.q_table)

        # print("size of QTable: "+str(len(self.Q.q_table)))

    # simulate a game run based on current Qtable
    def test(self):
        self.game = Game()
        while not self.game.game_end():
            valid_moves = self.game.get_valid_moves()
            state = self.Q.hash_key(self.game.hash_board)
            if self.game.player == PLAYER_MAX:
                move = max(valid_moves, key=lambda m: self.Q.get_q_value(state, m, NO_ACTION))
            else:
                move = min(valid_moves, key=lambda m: self.Q.get_q_value(state, NO_ACTION, m))
            self.game.move(move)
            print(self.game)

    # check whether the training is ready to be terminated
    def ready_to_end(self):
        if self.qtable_size_increment < 20 and self.convergence_cnt >= self.convergence_criteria:
            return True
        else:
            return False

    # save Qtable to txt file
    def save(self):
        with open(r'qtable.txt', 'w+') as f:
            f.write(str(self.Q.q_table))

    # train in batches until convergence criteria is reached or ctrl-C is pressed
    def run(self):
        try:
            self.start_time = time.time()
            # train
            b = 0
            while not self.end_train:
                # print('TRAINING BATCH {}'.format(b))
                self.train()
                b += 1

                # print('TESTING BATCH {}'.format(b))
                # self.test()

            print("qtable size: " + str(len(self.Q.q_table)))
            print("train time: " + str(time.time() - self.start_time))
            self.save()

        # simulate one min-max game based on current Qtable, and saves the Qtable
        except KeyboardInterrupt:
            print("Qtable size: " + str(len(self.Q.q_table)))
            print("Train time: " + str(time.time() - self.start_time))
            print("Convergence cnt: " + str(self.convergence_cnt))
            print("Batch: " + str(b))
            self.test()
            self.save()
            sys.exit()


if __name__ == '__main__':
    Train().run()