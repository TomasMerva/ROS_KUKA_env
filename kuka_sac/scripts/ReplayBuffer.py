#!/usr/bin/env python3

import numpy as np

## A simple FIFO experience replay buffer for SAC agent
#
class ReplayBuffer():
    ## The constructor of the class
    #  Initialize arrays
    #  @param state_dim The dimension of state
    #  @param act_dim The dimension of action
    #  @param size The maximum size of the buffer
    #
    def __init__(self, state_dim, act_dim, size):
        self.obs1_buf = np.zeros([size, state_dim], dtype=np.float32)
        self.obs2_buf = np.zeros([size, state_dim], dtype=np.float32)
        self.acts_buf = np.zeros([size, act_dim], dtype=np.float32)
        self.rews_buf = np.zeros(size, dtype=np.float32)
        self.done_buf = np.zeros(size, dtype=np.float32)
        self.ptr, self.size, self.max_size = 0, 0, size

    ## @brief The method for storing data after each timestep
    #
    def store(self, obs, act, rew, next_obs, done):
        self.obs1_buf[self.ptr] = obs
        self.obs2_buf[self.ptr] = next_obs
        self.acts_buf[self.ptr] = act
        self.rews_buf[self.ptr] = rew
        self.done_buf[self.ptr] = done
        self.ptr = (self.ptr + 1) % self.max_size
        self.size = min(self.size + 1, self.max_size)

    ## @brief The method for random sampling a batch of transitions,
    #  @details B={(s,a,r,s',d)} from D
    def sample_batch(self, batch_size=32):
        idxs = np.random.randint(0, self.size, size=batch_size)
        return dict(obs1=self.obs1_buf[idxs],
                    obs2=self.obs2_buf[idxs],
                    acts=self.acts_buf[idxs],
                    rews=self.rews_buf[idxs],
                    done=self.done_buf[idxs])
