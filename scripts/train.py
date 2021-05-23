import random
import numpy as np


PLAYER_O = 0
PLAYER_X = 1

class Train(object):

    def __init__(self):
        self.alpha = 1.0
        self.gamma = 1.0
        self.board = [0]*9
        self.states = []
        self.explor = 0.5 #TODO: how to set this value?
        self.pi = {} #TODO: are policies for both players needed?
        self.Q = {} #TODO: how to represent this? need to initialize

    def board_to_state(self):

        hash = 0
        for i in range(9):
            hash += (3 ** i) *self.board[i]
        return hash

    def value(self): #TODO: min-max value function, look at current board

        state = self.board_to_state()
        V_s = -1

        for s in self.states:
            x = 2**64
            o = 0
            policy = self.pi[s]
            for i in range(9):
                val = sum(self.Q[state,i,a] for a in range(9))*policy[i]
                if val < x:
                    x = val
                    o = i
            if val > V_s:
                V_s = val

        return V_s

    def reward(self): #TODO: need to return reward at state
        return 0

    def get_next_action(self,player): #TODO: integrate with policy, this should also update board
        return random.randint(0,8)

    def has_converged(self): #TODO: how to determine convergence?
        return False

    def train(self):

        while not self.has_converged():

            s = self.board_to_state()
            a = self.get_next_action(PLAYER_O)
            o = self.get_next_action(PLAYER_X) #calling this will automatically update board state
            r = self.reward()

            V_s = self.value()
            self.Q[s,a,o] = (1 - self.alpha)*self.Q[s,a,o] + self.alpha*(r + self.gamma*V_s)

            #TODO: simulate multiple games? Need board to reset when game is over

            
if __name__ == '__main__':
    Train()
