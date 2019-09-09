# Heavily inspired by (including som code borrowed from)
# https://gist.github.com/n1try/af0b8476ae4106ec098fea1dfe57f578

import gym
import numpy as np
import matplotlib.pyplot as plt

import math
from collections import deque

#from keras.models import Sequential
#from keras.layers import Dense
#from keras.optimizers import Adam

ENV_NAME = "CartPole-v0"

GAMMA = 1.0

LEARNING_RATE_MAX = 1.0
LEARNING_RATE_MIN = 0.1
LEARNING_RATE_DECAY = 0.99

EXPLORATION_MAX = 1.0
EXPLORATION_MIN = 0.01
EXPLORATION_DECAY = 0.99


class QLearningSolver():

    def __init__(self):
        # Setup environment
        self.env = gym.make(ENV_NAME)
        
        self.n_episodes = 1000 # max number of training episodes 
        self.n_win_ticks = 195 # average ticks over 100 episodes required to win

        # Make Q-table, initialize at zero (multi-dimensional array)
        self.buckets = (1, 1, 6, 12) # Discretize continuous state space using buckets
        self.Q = np.zeros(self.buckets + (self.env.action_space.n,))

    # Discretize the continuous observation space into buckets
    def discretize(self, obs):
        upper_bounds = [self.env.observation_space.high[0], 0.5, self.env.observation_space.high[2], math.radians(50)]
        lower_bounds = [self.env.observation_space.low[0], -0.5, self.env.observation_space.low[2], -math.radians(50)]
        ratios = [(obs[i] + abs(lower_bounds[i])) / (upper_bounds[i] - lower_bounds[i]) for i in range(len(obs))]
        new_obs = [int(round((self.buckets[i] - 1) * ratios[i])) for i in range(len(obs))]
        new_obs = [min(self.buckets[i] - 1, max(0, new_obs[i])) for i in range(len(obs))]
        return tuple(new_obs)

    # This chooses the action
    # With probability exploration rate, do a random move
    def act(self, state, epsilon):
        if (np.random.random() <= epsilon):
            return self.env.action_space.sample()
        else:
            return np.argmax(self.Q[state])

    # This is the update rule
    # alpha is the learning rate
    # gamma is the discount factor
    def update_q(self, state_old, action, reward, state_new, alpha):
        self.Q[state_old][action] += alpha * (reward + GAMMA * np.max(self.Q[state_new]) - self.Q[state_old][action])

    # epsilon is the exploration rate
    def get_epsilon(self, t):
         return max(EXPLORATION_MIN, EXPLORATION_MAX * EXPLORATION_DECAY**t)

    # alpha is the learning rate
    def get_alpha(self, t):
        return max(LEARNING_RATE_MIN, LEARNING_RATE_MAX * LEARNING_RATE_DECAY**t)
        
    def save_fig(self,score,output_path,title):
        x = range(len(score))
        y = score

        plt.subplots()
        plt.plot(x,score, label="score per run")

        average_range = 100
        plt.plot(x[-average_range:], [np.mean(y[-average_range:])] * len(y[-average_range:]), linestyle="--", label="last " + str(average_range) + " runs average")

        plt.plot(x, [195] * len(x), linestyle=":", label=str(195) + " score average goal")
    
        if len(score) > 1:
            trend_x = x[1:]
            z = np.polyfit(np.array(trend_x), np.array(y[1:]), 1)
            p = np.poly1d(z)
            plt.plot(trend_x, p(trend_x), linestyle="-.",  label="trend")

        plt.title(title)
        plt.xlabel("Runs")
        plt.ylabel("Scores")
        
        plt.legend(loc="upper left")

        plt.savefig(output_path, bbox_inches="tight")
        plt.close()


    # main function
    def run(self):
        
        mean_buffer = deque(maxlen=100)
        scores = []

        plt.ion()
        
        # Episode loop
        for e in range(self.n_episodes):
            # Reset environment at start of episode. Also get observation of initial state
            state = self.env.reset()

            # Discretize
            state = self.discretize(state)
            
            # Get learning and exploration rates for this episode (These decrease over time)
            alpha = self.get_alpha(e)
            epsilon = self.get_epsilon(e)
            
            # Episode ends when terminal state is reached. This flag is returned by the environment if falling below a certain angle or moving a certain distance
            terminal = False

            # Reset time step counter
            i = 0

            while not terminal:
                # This renders the environment (takes a lot of time. comment to speed things up)
                self.env.render()

                # Choose action based on current (discretized) state and exploration rate
                action = self.act(state, epsilon)
                
                # Apply action to environment. Get new state observation and reward
                next_state, reward, terminal, info = self.env.step(action)
                
                # Discretize observed state into buckets
                next_state = self.discretize(next_state)

                # Update Q-values
                self.update_q(state, action, reward, next_state, alpha)
                    
                state = next_state

                # Count steps per episode (This will be our score)
                i += 1

            # Save score and calculate mean
            mean_buffer.append(i)
            scores.append(i)
            mean_score = np.mean(mean_buffer)

            # Check if mean score for the last 100 episodes is above the winning threshold
            # Also make sure we run for at least 100 episodes.
            if mean_score >= self.n_win_ticks and e >= 100:
                print('Ran {} episodes. Solved after {} trials'.format(e, e - 100)) # why e - 100 ???
                self.save_fig(scores,"./scores.png",ENV_NAME)
                return e - 100

            # Print status every 100 episodes
            if e % 10 == 0:
                print("Episode: " + str(e) + ", exploration: " + str(epsilon) + ", score: " + str(i) + ", mean: " + str(mean_score))

        # If we reach this point, we have reached the max number of episodes before winning
        print('Did not solve after {} episodes'.format(e))
        return e

if __name__ == "__main__":
    solver = QLearningSolver()
    solver.run()
