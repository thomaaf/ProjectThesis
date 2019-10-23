from matplotlib import pyplot as plt
from gridWorld import gridWorld
import numpy as np

def show_value_function(mdp, V):
    fig = mdp.render(show_state = False, show_reward = False)            
    for k in mdp.states():
        s = k if isinstance(k, tuple) else mdp.legal_states[k]
        fig.axes[0].annotate("{0:-.3f}".format(V[k]), (s[1] - 0.3 ,s[0] + 0.1), size = 70/mdp.board_mask.shape[0])
    plt.show()
    
def show_policy(mdp, PI):
    fig = mdp.render(show_state = False, show_reward = False)
    action_map = {"U": "↑", "D": "↓", "L": "←", "R": "→"}
    for k in mdp.states():
        s = k if isinstance(k, tuple) else mdp.legal_states[k]
        if mdp.terminal[s] == 0:
            fig.axes[0].annotate(action_map[PI[k]], (s[1] - 0.1, s[0] + 0.1), size = 100/mdp.board_mask.shape[0])
    plt.show()
    
####################  Problem 1: Value Iteration #################### 

def value_iteration(mdp, gamma, theta = 1e-3):
    V = np.zeros((len(mdp.states())))    
    while True:
        delta = 0
        for s in mdp.states():
            v = V[s]
            tmp = []
            for a in mdp.actions(s):
                tmp2 = 0;
                for sn in mdp.possible_transitions(s,a):
                    tmp2 = tmp2 + mdp.transition_probability(s,a,sn)*(mdp.reward(s) + gamma*V[sn])
                tmp.append(tmp2)
            if tmp == []:
                V[s] = mdp.reward(s)
            else:
                V[s] = max(tmp)
            delta = max([delta,abs(v - V[s])])
        
        if delta < theta:
            break
    return V
    """
    Input arguments:
        - mdp     Is the markov decision process, it has some usefull functions given below
        - gamma   Is the discount rate
        - theta   Is a small threshold for determining accuracy of estimation
    
    Some usefull functions of the grid world mdp:
        - mdp.states() returns a list of all states [0, 1, 2, ...]
        - mdp.actions(state) returns list of actions ["U", "D", "L", "R"] if state non-terminal, [] if terminal
        - mdp.transition_probability(s, a, s_next) returns the probability p(s_next | s, a)
        - mdp.reward(state) returns the reward of the state R(s)
    """
    raise Exception("Not implemented")
            
    return V

def policy(mdp, V):
    # Initialize the policy list of crrect length
    PI = np.random.choice(env.actions(), len(mdp.states()))
    actions = mdp.actions(0)
    for s in mdp.states():
        tmp = []
        for a in mdp.actions(s):
            tmp2 = 0;
            for sn in mdp.possible_transitions(s,a):
                tmp2 = tmp2 + mdp.transition_probability(s,a,sn)*(mdp.reward(s) + gamma*V[sn])
                
            tmp.append(tmp2)
        if tmp != []:
                PI[s] = actions[tmp.index(max(tmp))]
        

    return PI

####################  Problem 2: Policy Iteration #################### 
def policy_evaluation(mdp, gamma, PI, V, theta = 1e-3):   
    #V = np.zeros((len(mdp.states())))    
    while True:
        delta = 0
        for s in mdp.states():
            v = V[s]
            tmp2 = 0;
            a = PI[s]
            for sn in mdp.possible_transitions(s,a):
                tmp2 = tmp2 + mdp.transition_probability(s,a,sn)*(mdp.reward(s) + gamma*V[sn])
            if mdp.actions(s) == []:
                V[s] = mdp.reward(s)
            else:
                V[s] = tmp2
            delta = max([delta,abs(v - V[s])])
        print(delta)
        if delta < theta:
            break
    return V 


def policy_iteration(mdp, gamma):
    # Make a valuefunction, initialized to 0
    V = np.zeros((len(mdp.states())))
    
    # Create an arbitrary policy PI
    PI = np.random.choice(env.actions(), len(mdp.states()))
    print(PI)
    while True:
        V = policy_evaluation(mdp, gamma, PI, V, theta = 1e-3)
        print(V)
        PI2 = policy(mdp, V)
        #print(PI)
        if np.array_equal(PI, PI2):
            break
        else:
            PI = PI2

    return PI, V        
    """
    YOUR CODE HERE:
    Problem 2b) Implement Policy Iteration
    
    Input arguments:  
        - mdp   Is the markov decision problem
        - gamma Is discount factor

    Some useful tips:
        - Use the the policy_evaluation function from the preveous subproblem
    """
    raise Exception("Not implemented")
            
    return PI, V

if __name__ == "__main__":
    """
    Change the parameters below to change the behaveour, and map of the gridworld.
    gamma is the discount rate, while filename is the path to gridworld map. Note that
    this code has been written for python 3.x, and requiers the numpy and matplotlib
    packages

    Available maps are:
        - gridworlds/tiny.json
        - gridworlds/large.json
    """
    gamma   = .9
    filname = "gridworlds/large.json"



    # Import the environment from file
    env = gridWorld(filname)

    # Render image
    #fig = env.render(show_state = False)
    #plt.show()

    # Run Value Iteration and render value function and policy
    #V = value_iteration(mdp = env, gamma = gamma)

    #PI = policy(env, V)

    #show_value_function(env, V)
    #show_policy(env, PI)
    
    # Run Policy Iteration and render value function and policy
    PI, V = policy_iteration(mdp = env, gamma = gamma)
    show_value_function(env, V)
    show_policy(env, PI)
