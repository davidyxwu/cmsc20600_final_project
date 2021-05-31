import random
import numpy as np
from q_table import Qtable, Value
from game import Game, PLAYER_O,PLAYER_X
NO_ACTION = 9
PLAYER_MAX = PLAYER_O
PLAYER_MIN = PLAYER_X
class Train(object):

    def __init__(self):
        self.alpha = 0.5
        self.gamma = 0.8
        self.num_batches = 100
        self.batch_size = 1e4
        self.game = Game()
        self.states = []
        self.explor = 0.5 
        self.V = Value()
        self.training_step = 0
        self.game_count = 0
        self.Q = Qtable()

    def value(self,key): #max_a min_o q(s,a,o)

        max_q = - 2 ** 64 #small number

        for a in range(10):
            min_q = 2 ** 64 #large number
            for o in range(10):
                q = self.Q.get_q_value(key,a,o)
                if q < min_q:
                    min_q = q
            if max_q < min_q:
                max_q = min_q


        return max_q


    def get_next_action(self): #returns tuple (a,o)

        moves = self.game.get_valid_moves()
        indx = random.randint(0,len(moves) - 1)
        explore = np.random.binomial(1,self.explor)
        if explore:
            if self.game.player == PLAYER_MAX:
                a = moves[indx]
                o = NO_ACTION
                self.game.move(a)
            else:
                a = NO_ACTION
                o = moves[indx]
                self.game.move(o)
        else:
            state = self.Q.hash_key(self.game.board)
            if self.game.player == PLAYER_MAX:
                a = max(moves,key=lambda m: self.Q.get_q_value(state,m,NO_ACTION))
                o = NO_ACTION
                self.game.move(a)
            else:
                a = NO_ACTION
                o = min(moves,key=lambda m: self.Q.get_q_value(state,NO_ACTION,m))
                self.game.move(o)

        return a,o

    def train(self):

        self.game = Game()
        self.training_step = 0
        while self.training_step < self.batch_size:
            prev_state = self.Q.hash_key(self.game.board)
            a, o = self.get_next_action()
            next_state = self.Q.hash_key(self.game.board)
            r = self.game.reward(a) if o == NO_ACTION else self.game.reward(o)
            q_val = (1 - self.alpha) * self.Q.get_q_value(prev_state, a, o) + self.alpha * (r + self.gamma * self.value(next_state))
            self.Q.q_table[prev_state, a, o] =  q_val
            self.V.value[prev_state] = self.value(prev_state)
            self.training_step += 1
            if self.game.game_end():
                self.game = Game()
                self.game_count += 1
                continue

    def test(self):
        self.game = Game()
        while not self.game.game_end():
            valid_moves = self.game.get_valid_moves()
            state = self.Q.hash_key(self.game.board)
            if self.game.player == PLAYER_MAX:
                move = max(valid_moves,key=lambda m: self.Q.get_q_value(state,m,NO_ACTION))
            else:
                move = min(valid_moves,key=lambda m: self.Q.get_q_value(state,NO_ACTION,m))
            self.game.move(move)
        print(self.game)


    def run(self):
        for b in range(self.num_batches):
            print('TRAINING BATCH {}'.format(b))
            self.train()
            print('TESTING BATCH {}'.format(b))
            self.test()





if __name__ == '__main__':

    Train().run()