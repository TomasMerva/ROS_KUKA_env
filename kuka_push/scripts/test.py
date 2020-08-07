#!/usr/bin/env python3

import rospy
from kuka_push.srv import *
import numpy as np
import random
import time

if __name__ == "__main__":
    """
    Initialization
    """
    rospy.init_node("agent_test")
    rospy.wait_for_service("kukapush_server")
    step = rospy.ServiceProxy("kukapush_server", rl_env)

    def randomAction():
        action_x = random.uniform(-1.0, 1.0)
        action_y = random.uniform(-1.0, 1.0)
        action_z = random.uniform(-1.0, 1.0)
        action = [action_x, action_y, action_z]
        return action

    reset = True
    action = randomAction()
    resp = step(action, reset)
    obs = np.round(resp.state, decimals=6)

    print("\nobs are \n ", obs)


    reset = False
    start = time.time()
    action = randomAction()
    resp = step(action, reset)
    obs = np.round(resp.state, decimals=6)
    print("\nobs are ", obs)

    print("Time is ", time.time()-start)

    start = time.time()
    action = randomAction()
    resp = step(action, reset)
    obs = np.round(resp.state, decimals=6)
    print("\nobs are ", obs)

    print("Time is ", time.time()-start)

    start = time.time()
    action = randomAction()
    resp = step(action, reset)
    obs = np.round(resp.state, decimals=6)
    print("\nobs are ", obs)

    print("Time is ", time.time()-start)

    # reset = False
    # action = randomAction()
    # resp = step(action, reset, 0)
    # obs = np.round(resp.state, decimals=6)
    # print("\nobs are \n ", obs)
    #
    # action = randomAction()
    # resp = step(action, reset, 0)
    # obs = np.round(resp.state, decimals=6)
    # print("\nobs are \n ", obs)

