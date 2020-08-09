#!/usr/bin/env python3

import rospy
from kuka_push.srv import *
from kuka_push.msg import rlplugin_msg

import numpy as np
from ReplayBuffer import ReplayBuffer
import random
import time
import tensorflow as tf
import core
from core import get_vars
from config import configData


class SAC():
    def __init__(self, state_dim, act_dim, hidden_sizes, gamma=0.99, polyak=0.995,
                 lr=1e-3, alpha=0.2):

        """
        Placeholders initialization
        """
        self.state_t_ph = tf.placeholder(shape=(None, state_dim), dtype=tf.float32, name="State_t_ph")
        self.state_t1_ph = tf.placeholder(shape=(None, state_dim), dtype=tf.float32, name="State_t1_ph")
        self.act_ph = tf.placeholder(shape=(None, act_dim), dtype=tf.float32, name="Actions_ph")
        self.ret_ph = tf.placeholder(shape=(None,), dtype=tf.float32, name="Rewards_ph")
        self.done_ph = tf.placeholder(shape=(None,), dtype=tf.float32, name="Done_ph")

        self.gamma = gamma
        self.alpha = alpha
        self.lr = lr
        self.polyak = polyak

        with tf.variable_scope("main"):
            self.mean, self.pi, self.logp_pi, self.q1, \
            self.q2, self.q1_pi, self.q2_pi, self.v = core.mlp_actor_critic(self.state_t_ph, self.act_ph,
                                                                            hidden_sizes)
        with tf.variable_scope("target"):
            _, _, _, _, _, _, _, self.v_targ = core.mlp_actor_critic(self.state_t1_ph, self.act_ph, hidden_sizes)

        # Min Double-Q:
        self.min_q_pi = tf.minimum(self.q1_pi, self.q2_pi)

        # Targets for Q and V regression
        self.q_backup = tf.stop_gradient(self.ret_ph + self.gamma * (1 - self.done_ph) * self.v_targ)
        self.v_backup = tf.stop_gradient(self.min_q_pi - self.alpha * self.logp_pi)

        # Soft actor-critic losses
        self.pi_loss = tf.reduce_mean(self.alpha * self.logp_pi - self.min_q_pi, name="pi_loss")
        self.q1_loss = 0.5 * tf.reduce_mean((self.q_backup - self.q1) ** 2, name="q1_loss")
        self.q2_loss = 0.5 * tf.reduce_mean((self.q_backup - self.q2) ** 2, name="q2_loss")
        self.v_loss = 0.5 * tf.reduce_mean((self.v_backup - self.v) ** 2, name="v_loss")
        self.value_loss = self.q1_loss + self.q2_loss + self.v_loss

        # Policy train op
        # (has to be separate from value train op, because q1_pi appears in pi_loss)
        self.pi_optimizer = tf.train.AdamOptimizer(learning_rate=self.lr)
        self.train_pi_op = self.pi_optimizer.minimize(self.pi_loss, var_list=get_vars('main/pi'), name="train_pi_op")

        # Value train op
        # (control dep of train_pi_op because sess.run otherwise evaluates in nondeterministic order)
        self.value_optimizer = tf.train.AdamOptimizer(learning_rate=self.lr)
        self.value_params = get_vars('main/q') + get_vars('main/v')
        with tf.control_dependencies([self.train_pi_op]):
            self.train_value_op = self.value_optimizer.minimize(self.value_loss, var_list=self.value_params, name="train_value_op")

        # Polyak averaging for target variables
        # (control flow because sess.run otherwise evaluates in nondeterministic order)
        with tf.control_dependencies([self.train_value_op]):
            self.target_update = tf.group([tf.assign(self.v_targ, self.polyak * self.v_targ + (1 - self.polyak) * self.v_main)
                                    for self.v_main, self.v_targ in zip(get_vars('main'), get_vars('target'))], name="target_update")

        """ alpha tuning"""
        self.target_entropy = -4
        self.log_alpha = tf.compat.v1.get_variable('log_alpha', dtype=tf.float32, initializer=0.0)
        self.alpha = tf.exp(self.log_alpha, name="alpha")
        self.alpha_loss = -tf.reduce_mean(self.log_alpha * tf.stop_gradient(self.logp_pi + self.target_entropy), name="alpha_loss")

        self.alpha_optimizer = tf.compat.v1.train.AdamOptimizer(self.lr, name='alpha_optimizer')
        self.alpha_train_op = self.alpha_optimizer.minimize(loss=self.alpha_loss, var_list=[self.log_alpha], name="alpha_train_op")
        """-------------"""

        # All ops to call during one training step
        self.step_ops = [self.pi_loss, self.q1_loss, self.q2_loss, self.v_loss, self.q1, self.q2, self.v,
                         self.logp_pi, self.train_pi_op, self.train_value_op, self.target_update,
                         self.alpha_loss, self.alpha_train_op]

        # Initializing targets to match main variables
        self.target_init = tf.group([tf.assign(self.v_targ, self.v_main)
                                     for self.v_main, self.v_targ in zip(get_vars('main'), get_vars('target'))])

        """For saving purposes"""
        tf.identity(self.pi, name="pi")
        tf.identity(self.mean, name="mean")
        tf.identity(self.q1, name="q1")
        tf.identity(self.q2, name="q2")
        tf.identity(self.v, name="v")
        tf.identity(self.logp_pi, name="logp_pi")

        """ Tensorboard summaries """
        self.summary_reward_ph = tf.placeholder(tf.float32, name="reward_scalar_ph")
        self.summary_reward = tf.summary.scalar("Learning performance", self.summary_reward_ph)

        """ For saving trained model """
        self.saver = tf.train.Saver()

        self.sess = tf.Session()
        self.sess.run(tf.global_variables_initializer())
        self.sess.run(self.target_init)

        self.file_writer = tf.summary.FileWriter('/home/tomas/catkin_ws/src/kuka_push/scripts/logs', self.sess.graph)

    def SAC_getAction(self, state, deterministic=False):
        act_op = self.mean if deterministic else self.pi
        return self.sess.run(act_op, feed_dict={self.state_t_ph: state.reshape(1, -1)})[0]

    def SAC_update(self, obs1, obs2, acts, rews, done):
        feed_dict = {self.state_t_ph: obs1,
                     self.state_t1_ph: obs2,
                     self.act_ph: acts,
                     self.ret_ph: rews,
                     self.done_ph: done,
                     }
        outs = self.sess.run(self.step_ops, feed_dict)





if __name__ == "__main__":
    # Initialization
    rospy.init_node("kuka_agent")
    rospy.wait_for_service("kukapush_server")
    #service client for interacting with the environment
    step = rospy.ServiceProxy("kukapush_server", rl_env)
    #publisher for RViz plugin
    pub = rospy.Publisher('/rl_states', rlplugin_msg, queue_size=1)
    msg = rlplugin_msg()
    # Load hyperparameters
    params = configData()

    # Setting seed
    tf.set_random_seed(0)
    np.random.seed(0)

    # Properties of the environment
    state_dim = 29
    act_dim = 3
    timesteps_per_episode = 100
    start_steps = 10000

    # Hyperparameters
    buffer_maxsize = 1000000
    batchSize = 256
    hidden_sizes = [256, 256]
    epochs = 30
    iteration_per_epoch = 500

    timestep = 0
    ep_len = 0

    # Recording results
    mean_ret = []
    prev_ret = 0.0

    # lists for Hindsight Experience Replay
    list_achievedGoals = []
    list_states = []
    list_new_states = []
    list_actions = []

    agent = SAC(state_dim, act_dim, hidden_sizes)
    replay_buffer = ReplayBuffer(state_dim, act_dim, buffer_maxsize)

    # request for resetting the environment
    def reset_env():
        reset = True
        action = [0, 0, 0]
        resp = step(action, reset)
        obs = np.round(resp.state, decimals=6)
        state = np.concatenate((resp.state, resp.desired_goal), axis=None)
        state = np.round(state, decimals=6)
        return state, obs

    # generating random actions
    def randomAction():
        action_x = random.uniform(-1.01, 1.01)
        action_y = random.uniform(-1.01, 1.01)
        action_z = random.uniform(-1.01, 1.01)
        action = [action_x, action_y, action_z]
        return action

    # Hindsight Experience Replay algorithm
    def her():
        global ep_len
        global list_states, list_actions, list_new_states, list_achievedGoals
        #check if episode lasted less than 4 timesteps
        if ep_len < 4:
            # create additional goal only from the last timestep
            her_state = np.concatenate((np.array(list_states[0]), np.array(list_achievedGoals[0])))
            her_next_state = np.concatenate((np.array(list_new_states[0]), np.array(list_achievedGoals[0])))
            her_action = list_actions[0]
            her_reward = 0
            her_done = True
            replay_buffer.store(her_state, her_action, her_reward, her_next_state, her_done)
        else:
            # divide an episode into 4 groups/parts, which gives us 4 additional goals
            indexes = np.sort(np.random.randint(0, ep_len, size=3))
            """ first group """
            for i in range(indexes[0] + 1):
                her_state = np.concatenate((np.array(list_states[i]), np.array(list_achievedGoals[indexes[0]])))
                her_next_state = np.concatenate(
                    (np.array(list_new_states[i]), np.array(list_achievedGoals[indexes[0]])))
                her_action = list_actions[i]
                if i == indexes[0]:
                    her_done = True
                    her_reward = 0.0
                else:
                    her_done = False
                    her_reward = -1.0
                replay_buffer.store(her_state, her_action, her_reward, her_next_state, her_done)
            """ second group """
            for i in range(indexes[0] + 1, indexes[1] + 1):
                her_state = np.concatenate((np.array(list_states[i]), np.array(list_achievedGoals[indexes[1]])))
                her_next_state = np.concatenate(
                    (np.array(list_new_states[i]), np.array(list_achievedGoals[indexes[1]])))
                her_action = list_actions[i]
                if i == indexes[1]:
                    her_done = True
                    her_reward = 0.0
                else:
                    her_done = False
                    her_reward = -1.0
                replay_buffer.store(her_state, her_action, her_reward, her_next_state, her_done)
            """ third group """
            for i in range(indexes[1] + 1, indexes[2] + 1):
                her_state = np.concatenate((np.array(list_states[i]), np.array(list_achievedGoals[indexes[2]])))
                her_next_state = np.concatenate(
                    (np.array(list_new_states[i]), np.array(list_achievedGoals[indexes[2]])))
                her_action = list_actions[i]
                if i == indexes[2]:
                    her_done = True
                    her_reward = 0.0
                else:
                    her_done = False
                    her_reward = -1.0
                replay_buffer.store(her_state, her_action, her_reward, her_next_state, her_done)
            """ fourth group """
            for i in range(indexes[2] + 1, ep_len):
                her_state = np.concatenate((np.array(list_states[i]), np.array(list_achievedGoals[ep_len - 1])))
                her_next_state = np.concatenate(
                    (np.array(list_new_states[i]), np.array(list_achievedGoals[ep_len - 1])))
                her_action = list_actions[i]
                if i == ep_len - 1:
                    her_done = True
                    her_reward = 0.0
                else:
                    her_done = False
                    her_reward = -1.0
                replay_buffer.store(her_state, her_action, her_reward, her_next_state, her_done)

    # Run one episode
    def episode():
        state, obs = reset_env()

        global timestep, timesteps_per_episode, mean_ret
        global list_states, list_actions, list_new_states, list_achievedGoals
        global ep_len

        reward, ep_len, done, reset = 0, 0, False, False

        for _ in range(timesteps_per_episode):
            # record state for HER
            list_states.append(obs)

            #first X timesteps do only random actions -> it supports exploration
            if timestep > start_steps:
                action = agent.SAC_getAction(state)
            else:
                action = randomAction()

            # record action for HER
            list_actions.append(action)

            # do an interaction with the environment
            resp = step(action, reset)

            # new state, reward and done flag for the current timestep
            new_state = np.round(np.concatenate((resp.state, resp.desired_goal), axis=None), decimals=6)
            reward = resp.reward
            done = resp.done

            # record new_state and achieved_goal for HER
            achieved_goal = np.round(resp.achieved_goal, decimals=6)
            obs = np.round(resp.state, decimals=6)
            list_new_states.append(obs)
            list_achievedGoals.append(achieved_goal)

            # increment time
            ep_len += 1
            timestep += 1

            # save trajectory into the buffer
            replay_buffer.store(state, action, reward, new_state, done)

            state = new_state

            # at the end of the episode use HER
            if ep_len == (timesteps_per_episode - 1):
                her()
                list_states, list_actions, list_achievedGoals, list_new_states = [], [], [], []
                break


    def policy_test():
        global mean_ret, ep_len
        state, obs = reset_env()
        reward, ep_len, done, reset = 0, 0, False, False

        while not done:
            action = agent.SAC_getAction(state, True)

            # Send request to server
            resp = step(action, reset)

            #Handle response from server
            state = np.round(np.concatenate((resp.state, resp.desired_goal), axis=None), decimals=6)
            reward = resp.reward
            done = resp.done

            ep_len += 1

            if reward == 0.0:
                done = True
            if ep_len == 50:
                done = True

        if reward == 0.0:
            mean_ret.append(1)
        elif reward == -1.0:
            mean_ret.append(0)

    # main loop
    for epoch in range(epochs):
        start_time = time.time()
        for i in range(iteration_per_epoch):
            # data for RViz plugin
            msg.training = True
            msg.epoch = epoch
            msg.episode = i
            msg.max_score = prev_ret
            pub.publish(msg)
            # run one episode
            episode()
            # Update policy
            for j in range(2*ep_len):
                batch = replay_buffer.sample_batch(batchSize)
                agent.SAC_update(batch['obs1'], batch['obs2'], batch['acts'], batch['rews'], batch['done'])
        # test policy after each epoch
        for j in range(10):
            policy_test()
        print('epoch: %3d \t ep_ret_avg: %.3f \t elapsed_time: %3d' % (epoch, np.mean(mean_ret), (time.time() - start_time)))
        # Tensorboard charts
        summary = agent.sess.run(agent.summary_reward, feed_dict={agent.summary_reward_ph: np.mean(mean_ret)})
        agent.file_writer.add_summary(summary, (epoch + 1))

        msg.prev_score = np.mean(mean_ret)

        # save the best policy
        if prev_ret < np.mean(mean_ret):
            agent.saver.save(agent.sess, "/home/tomas/catkin_ws/src/kuka_push/saved_model/sac_KUKAPush_trained_model")
            print("model is saved")
            prev_ret = np.mean(mean_ret)
        mean_ret.clear()

    msg.max_score = prev_ret
    msg.training = False
    pub.publish(msg)
