# https://gym.openai.com/docs/

import gym

env = gym.make('CartPole-v0')

# Run 20 training episodes
for episode in range(20):

    # Reset evironment at start of episode and get initial observation
    observation = env.reset()

    # Count time steps (this is the cumulative reward
    t = 0

    # Flag to check if we are in terminal state (updated by env.step)
    done = False
        
    while not done:
        env.render()
        
        # Choose a random action from the set of allowed actions
        action = env.action_space.sample()

        # Apply that action to the environment
        observation, reward, done, info = env.step(action)

        t += 1

    print("Episode finished after {} timestep".format(t+1))


