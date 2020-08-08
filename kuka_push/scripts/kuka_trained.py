#!/usr/bin/env python3

import rospy
from kuka_push.srv import *
from kuka_push.msg import rlplugin_msg
from std_msgs.msg import Bool

import numpy as np
from ReplayBuffer import ReplayBuffer
import random
import time
import tensorflow as tf
import core
from core import get_vars


class trainedAgent():
    def __init__(self):
        self.sess = tf.Session()
        self.model = tf.train.import_meta_graph\
            ("/home/tomas/catkin_ws/src/kuka_push/saved_model/sac_KUKAPush_trained_model.meta")
        self.model.restore(self.sess, tf.train.latest_checkpoint("/home/tomas/catkin_ws/src/kuka_push/saved_model/"))

        self.graph = tf.get_default_graph()

        """ Placeholders """
        self.state_t_ph = self.graph.get_tensor_by_name("State_t_ph:0")
        self.state_t1_ph = self.graph.get_tensor_by_name("State_t1_ph:0")
        self.act_ph = self.graph.get_tensor_by_name("Actions_ph:0")
        self.ret_ph = self.graph.get_tensor_by_name("Rewards_ph:0")
        self.done_ph = self.graph.get_tensor_by_name("Done_ph:0")

        """ Tensors for policy updating """
        self.pi_loss = self.graph.get_tensor_by_name("pi_loss:0")
        self.q1_loss = self.graph.get_tensor_by_name("q1_loss:0")
        self.q2_loss = self.graph.get_tensor_by_name("q2_loss:0")
        self.v_loss = self.graph.get_tensor_by_name("v_loss:0")
        self.q1 = self.graph.get_tensor_by_name("q1:0")
        self.q2 = self.graph.get_tensor_by_name("q2:0")
        self.v = self.graph.get_tensor_by_name("v:0")
        self.logp_pi = self.graph.get_tensor_by_name("logp_pi:0")

        """ Ops """
        self.train_pi_op = self.graph.get_operation_by_name("train_pi_op")
        self.train_value_op = self.graph.get_operation_by_name("train_value_op")
        self.target_update = self.graph.get_operation_by_name("target_update")

        """Action"""
        self.mean = self.graph.get_tensor_by_name("mean:0")
        self.pi = self.graph.get_tensor_by_name("pi:0")

        # All ops to call during one training step
        self.step_ops = [self.pi_loss, self.q1_loss, self.q2_loss, self.v_loss, self.q1, self.q2, self.v, self.logp_pi,
                         self.train_pi_op, self.train_value_op, self.target_update]

        #self.file_writer = tf.summary.FileWriter("/home/tomas/catkin_ws/src/kuka_sac/scripts/obnovenie", self.sess.graph)

        self.reset = False

    # def update(self, obs1, obs2, acts, rews, done):
    #     feed_dict = {self.state_t_ph: obs1,
    #                  self.state_t1_ph: obs2,
    #                  self.act_ph: acts,
    #                  self.ret_ph: rews,
    #                  self.done_ph: done,
    #                  }
    #     outs = self.sess.run(self.step_ops, feed_dict)

    def getAction(self, state, deterministic=False):
        act_op = self.mean if deterministic else self.pi
        return self.sess.run(act_op, feed_dict={self.state_t_ph: state.reshape(1, -1)})[0]

    def resetEnv(self, msg):
        self.reset = msg.data


if __name__ == "__main__":
    """
    Initialization
    """
    rospy.init_node("kuka_agent")
    rospy.wait_for_service("kukapush_server")
    step = rospy.ServiceProxy("kukapush_server", rl_env)

    pub = rospy.Publisher('/rl_states', rlplugin_msg, queue_size=1)
    msg = rlplugin_msg()

    tf.set_random_seed(0)
    np.random.seed(0)

    """
    Vlastnosti prostredia
    """
    state_dim = 29
    act_dim = 3
    timesteps_per_episode = 50

    """
    Hyperparametre
    """
    buffer_maxsize = 1000000
    batchSize = 256
    hidden_sizes = [256, 256]

    timestep = 0
    ep_len = 0

    """
    Vysledky
    """
    mean_ret = []


    """
    definovanie agent a buffer
    """
    agent = trainedAgent()
    replay_buffer = ReplayBuffer(state_dim, act_dim, buffer_maxsize)

    rospy.Subscriber("reset_env", Bool, agent.resetEnv)


    def reset_env():
        reset = True
        action = [0, 0, 0]
        resp = step(action, reset)
        obs = np.round(resp.state, decimals=6)
        state = np.concatenate((resp.state, resp.desired_goal), axis=None)
        state = np.round(state, decimals=6)
        return state, obs


    def policy_test():
        global mean_ret, ep_len
        state, obs = reset_env()
        reward, ep_len, done, reset = 0, 0, False, False


        while not done:
            action = agent.getAction(state, True)

            """ Send request to server """
            resp = step(action, reset, ep_len)

            """ Handle response from server """
            state = np.round(np.concatenate((resp.state, resp.desired_goal), axis=None), decimals=6)
            reward = resp.reward
            done = resp.done

            """ Cas """
            ep_len += 1

            if reward == 0.0:
                done = True
            if ep_len == 50:
                done = True

        if reward == 0.0:
            mean_ret.append(1)
        elif reward == -1.0:
            mean_ret.append(0)


    msg.training = False
    state, obs = reset_env()
    reset = False
    while True:
        if agent.reset:
            state, obs = reset_env()
            reset, agent.reset = False, False

        action = agent.getAction(state, True)

        """ Send request to server """
        resp = step(action, reset)
        """ Handle response from server """
        state = np.round(np.concatenate((resp.state, resp.desired_goal), axis=None), decimals=6)

        if resp.reward == 0.0:
            msg.act_score = 1
        elif resp.reward== -1.0:
            msg.act_score = 0
        pub.publish(msg)




