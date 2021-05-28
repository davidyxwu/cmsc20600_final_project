import random
import numpy as np
from q_table import Qtable, Policy
from game import Game, PLAYER_O,PLAYER_X
NO_ACTION = 9
PLAYER_MAX = PLAYER_O
PLAYER_MIN = PLAYER_X
class Train(object):

    def __init__(self):
        self.alpha = 1.0
        self.gamma = 1.0
        self.game = Game()
        self.states = []
        self.explor = 0.5  # TODO: how to set this value?
        self.pi = Policy()
        self.count = 0
        self.Q = Qtable()

    def value(self):

        # state = self.board_to_state()
        # V_s = -1
        #
        # for s in self.states:
        #     x = 2 ** 64
        #     o = 0
        #     policy = self.pi[s]
        #     for i in range(9):
        #         val = sum(self.Q[state, i, a] for a in range(9)) * policy[i]
        #         if val < x:
        #             x = val
        #             o = i
        #     if val > V_s:
        #         V_s = val


    def get_next_action(self): #returns tuple (a,o) #TODO: integrate with policy, do I need to assign prob to value moves?

        explore = np.random.binomial(1,self.explor)
        moves = self.game.get_valid_moves()

        if self.policy.new_entry(self.game.board): #initialize probabilities to uniform
            if self.game.player == PLAYER_MAX:
                for m in moves:
                    self.policy.set_policy_value(self.game.board, m, 1.0 / len(moves))
            else:
                for i in range(9):
                    self.policy.set_policy_value(self.game.board, i, 0.0)
                self.pi.set_policy_value(self.game.board, NO_ACTION, 1.0)

        if explore and self.game.player == PLAYER_MAX:
            a = np.random.choice(self.pi.policy[self.pi.hash_key(self.game.board)])
            o = NO_ACTION

        else:
            indx = random.randint(0,len(moves) - 1)
            if self.game.player == PLAYER_MAX:
                a = moves[indx]
                o = NO_ACTION
                self.game.move(a)
            else:
                a = NO_ACTION
                o = moves[indx]
                self.game.move(o)

        return (a,o)

    def train(self):

        while not self.has_converged():

            a,o = self.get_next_action()
            V_s = self.value()
            r = self.game.reward(a) if o == NO_ACTION else self.game.reward(o)
            q_val = (1 - self.alpha) * self.Q.get_q_value(self.game.board, a, o) + self.alpha * (r + self.gamma * V_s)
            self.Q.set_q_value(self.game.board,a,o,q_val)
            self.count += 1

    def has_converged(self):
        return self.count < 10000



if __name__ == '__main__':
    q = Train()
    q.train()