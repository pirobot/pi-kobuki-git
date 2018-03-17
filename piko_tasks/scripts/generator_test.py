#!/usr/bin/env python


import random


class  randomwalk_iter:
    def  __init__(self):
        self.last = 1               # init the prior value
        self.rand = random.random() # init a candidate value
        
    def __iter__(self):
        return self                 # simplest iterator creation
    
    def next(self):
        if self.rand < 0.1:         # threshhold terminator
            raise StopIteration     # end of iteration
        else:                       # look for usable candidate
            while abs(self.last-self.rand) < 0.4:
                print '*',           # display the rejection
                self.rand = random.random() # new candidate
        
            self.last = self.rand   # update prior value
        return self.rand
    
for num in randomwalk_iter():
    print num,