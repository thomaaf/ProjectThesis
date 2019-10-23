import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from matplotlib import colors
import numpy as np
import json

class gridWorld(object):
    """
    Environment for the grid world. States are given as tuples, indexing the grid position 
    Actions are given as numbers characters U (Up), D (Down), L (Left), R (Right)
    """
    
    def __init__(self, file = "gridworlds/tiny.json"):
        self.load_from_file(file)

    def __repr__(self):
        string = "-"*(2 * self.board_mask.shape[1] + 1) + "\n"
        for y in range(self.board_mask.shape[0]):
            for x in range(self.board_mask.shape[1]):
                if self.current_state == (y, x):
                    string += '|O'
                elif self.terminal[(y, x)]:
                    string += "|T"
                else:
                    string += "| " if self.board_mask[(y, x)] == 0 else "|X"
            string += "|\n"
            string += "-"*(2 * self.board_mask.shape[1] + 1) + "\n"
        return string
        
    def reset(self, s = None):
        """
        Resets the agent to a state s, or random state if s is None
        """
        if (s is None):
            self.current_state = np.random.choice(self.states(as_tuple=True))
        else:
            s = s if isinstance(s, tuple) else self.legal_states[s]
            self.current_state = s
        
        
    def actions(self, s = None):
        """
        Returns a list of actions available in state s. If s is terminal it returns []
        if state is nonterminal it returns ["U", "D", "L", "R"]. If state is None it 
        returns the actions available in the current state.
        """
        s = self.state(as_tuple=True) if s == None else s
        s = s if isinstance(s, tuple) else self.legal_states[s]
        if self.terminal[s]:
            return []
        return ["U", "D", "L", "R"]
    
    def states(self, as_tuple = False):
        """
        Returns a list of all states [0, 1, 2, ...]. If as_tuple is True, the list will
        contain the tupele representation of the states [(0, 0), (1, 0), (2, 0), ...] 
        """
        if as_tuple:
            return self.legal_states
        else:
            return list(range(len(self.legal_states)))

    def reward(self, s = None):
        """
        Returns the reward of the state s, if s is not set, or s is None, then the 
        function returns the reward of the current state of the agent
        """
        s = self.state(as_tuple=True) if s == None else s
        s = s if isinstance(s, tuple) else self.legal_states[s]
        return self.rewards[s]
    
    def state(self, as_tuple = False):
        """
        Returns the current state of the agent, if as_tuple is True, the function
        returns the tuple representation of the current state
        """
        if as_tuple:
            return self.current_state
        else:
            return self.legal_states.index(self.current_state)
    
    def transition_probability(self, s, a, s_next):
        """
        Returns the probability of transitioning from the state s to the state s_next
        when taking action a i.e. p(s_next | s, a)
        """

        s = s if isinstance(s, tuple) else self.legal_states[s]
        s_next = s_next if isinstance(s_next, tuple) else self.legal_states[s_next]

        if self.terminal[s] > 0:
            return 0
        
        # Find the possible states to transition to
        f = (s[0] + (a == "D") - (a == "U"), s[1] + (a == "R") - (a == "L"))
        r = (s[0] + (a == "R") - (a == "L"), s[1] + (a == "D") - (a == "U"))
        l = (s[0] + (a == "L") - (a == "R"), s[1] + (a == "U") - (a == "D"))
        
        if s_next == f and s_next in self.states(as_tuple = True):
            return self.p
        
        if (s_next == r or s_next == l) and s_next in self.states(as_tuple = True):
            return (1 - self.p)/2
        
        if (s_next == s):
            p = 0
            p += self.p if f not in self.states(as_tuple=True) else 0
            p += (1 - self.p)/2 if r not in self.states(as_tuple=True) else 0
            p += (1 - self.p)/2 if l not in self.states(as_tuple=True) else 0
            return p
        
        return 0.0
    
    def possible_transitions(self, s,a):
        s = s if isinstance(s, tuple) else self.legal_states[s]
        f = (s[0] + (a == "D") - (a == "U"), s[1] + (a == "R") - (a == "L"))
        r = (s[0] + (a == "R") - (a == "L"), s[1] + (a == "D") - (a == "U"))
        l = (s[0] + (a == "L") - (a == "R"), s[1] + (a == "U") - (a == "D"))        
    
        #u = (s[0] - 1, s[1])
        #d = (s[0] + 1, s[1])
        #r = (s[0], s[1] - 1)
        #l = (s[0], s[1] + 1)
        nextState = []
        if f in self.states(as_tuple = True) and f not in nextState:
            nextState.append(self.legal_states.index(f))
        elif self.legal_states.index(s) not in nextState:
            nextState.append(self.legal_states.index(s))

        if r in self.states(as_tuple = True) and r not in nextState:
            nextState.append(self.legal_states.index(r))
        elif self.legal_states.index(s) not in nextState:
            nextState.append(self.legal_states.index(s))           

        if l in self.states(as_tuple = True) and l not in nextState:
            nextState.append(self.legal_states.index(l))
        elif self.legal_states.index(s) not in nextState:
            nextState.append(self.legal_states.index(s))            
        return nextState
    #
    def step(self, action, as_tuple=False):
        """
        Takes the desired action in the current state, and transitions to a new state according to 
        the transition_probability function. 

        Returns the new state, reward and whether or not the new state is terminal
        """
        if self.terminal[self.state(as_tuple=True)]:
            raise Exception("Terminal state was reached")
        
        states, probs, idx = [], [], []
        i = 0
        for s_next in self.states(as_tuple=True):
            if self.transition_probability(self.current_state, action, s_next) > 0:
                idx.append(i)
                i+= 1
                states.append(s_next)
                probs.append(self.transition_probability(self.current_state, action, s_next))
                
        self.current_state = states[np.random.choice(idx, p = probs)]
        
        return self.state(as_tuple = as_tuple), self.rewards[self.state(as_tuple=True)], self.terminal[self.state(as_tuple=True)] 
        
    def render(self, show_reward = True, show_state = True, show_terminal = True):
        """
        Returns a rendered image of the the gridworld as a matplotlib figure
        """
        cmap = colors.ListedColormap(['white', 'gray'])
        
        fig, ax = plt.subplots()
        ax.imshow(self.board_mask, cmap=cmap)
        
        ax.grid(which='major', axis='both', linestyle='-', color='k', linewidth=2)
        ax.set_xticks(np.arange(-.5, self.board_mask.shape[1] , 1));
        ax.set_yticks(np.arange(-.5, self.board_mask.shape[0] , 1));
        
        patches = []
        
        if show_terminal:
            for x in range(self.terminal.shape[1]):
                for y in range(self.terminal.shape[0]):
                    if self.terminal[(y, x)] > 0:
                        patches.append(Rectangle((x-0.5, y-0.5), 1, 1, fill = True, color = ("r" if self.rewards[y, x] < 0 else "g" )))
        
        circ = plt.Circle((self.current_state[1], self.current_state[0]), radius=0.2, color='b')
        if show_state:
            patches.append(circ)
        
        if show_reward:
            for x in range(self.rewards.shape[1]):
                for y in range(self.rewards.shape[0]):
                    if self.board_mask[(y, x)] == 0:
                        ax.annotate(self.rewards[(y, x)], (x-0.4,y+0.4), size = 50/self.board_mask.shape[0])
        
        for patch in patches:
            ax.add_patch(patch)
            
        ax.set_yticklabels([])
        ax.set_xticklabels([])
            
        return fig
        
    def load_from_file(self, filename):
        """
        Loads the grid world specifcations from a json file 
        """
        with open(filename) as file:
            data = json.load(file)
        
        # Load map from json file
        self.current_state = tuple(data['initial_state'])
        self.board_mask = np.array(data['board_mask'])
        self.terminal = np.array(data['terminal'])
        self.rewards = np.array(data['rewards'])
        self.p = data['probability']

        # Make list of all valid states
        ly, lx = self.board_mask.shape
        self.legal_states = [(y, x) for y in range(ly) for x in range(lx) if self.board_mask[y, x] == 0]