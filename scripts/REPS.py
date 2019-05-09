import numpy as np
from scipy.optimize import minimize
import matplotlib.pyplot as plt

NUM_DIM = 3

class bcolors:
  """
  ----------------------- DO NOT MODIFY! ---------------
  """
  HEADER = '\033[95m'
  OKBLUE = '\033[94m'
  OKGREEN = '\033[92m'
  WARNING = '\033[93m'
  FAIL = '\033[91m'
  ENDC = '\033[0m'
  BOLD = '\033[1m'
  UNDERLINE = '\033[4m'

class RelativeEntropyPolicySearch(object):
  """
  Class for Relative Entropy Policy Search (REPS)
  """
  def __init__(self, env, policy, epsilon=0.1):
    self.env = env
    self.policy = policy

    # Weights
    self.weight_mean = np.zeros(NUM_DIM)
    self.weight_std = np.ones(NUM_DIM)

    # REPS
    self.epsilon = epsilon
    self.returns_so_far = []

  def do_rollouts(self, num_rollouts=5):
    """
    ----------------------- TODO ---------------
    Performs rollouts based on current policy on the VREP environment

    Parameters
    ----------
    num_rollouts : The desired number of rollouts

    Returns
    ----------
    returns : a numpy array of size (num_rollouts, 1)
    w : policy parameters the rollouts were attempted for
    """

    #TODO: - Sample weights from the gaussian distribution;

    w = np.random.normal(self.weight_mean, self.weight_std, size=(num_rollouts, NUM_DIM))

    # # Get start and goal state
    # start_state = self.env.RobotPosition
    # goal_state = self.env.GoalPosition
    
    # Obtain trajectories from DMP policy
    trajectories = self.policy.get_trajectories(w, goal_state, start_state)

    # Get returns of the trajectories
    returns = self.env.executeTrajectories(trajectories)

    return returns, w

  def train(self, num_iter=10, num_rollouts_per_iter=10):
    """
    Trains the DMPs with REPS

    Parameters
    ----------
    num_rollouts_per_iter : The number of rollouts of the current policy per iteration
    num_iter: The total number of iterations
    """
    mean_returns = []
    for it in range(num_iter):
      returns, w = self.do_rollouts(num_rollouts=num_rollouts_per_iter)
      mean_returns.append(np.mean(returns))
      print(bcolors.OKGREEN + 'Avg returns is ' + str(np.mean(returns)) + bcolors.ENDC)
      alpha = self.reps_update(returns)
      self.weight_mean, self.weight_std = self.fit_gaussian(alpha, w)

    plt.plot(range(len(mean_returns)), mean_returns)
    plt.xlabel('Iterations')
    plt.ylabel('Avg Return')
    plt.title('Learning curve')
    plt.show()
      
  def reps_update(self, returns):
    """
    ----------------------- TODO, implement REPS here ---------------
    The core function implementing the REPS objective

    Parameters
    ----------
    returns : numpy array of size (num_rollouts, 1)

    Returns
    ----------
    alpha : an output parameter for REPS policy update,
            a numpy array of size (num_rollouts, 1)
    """

    # TODO: implement REPS
    # HINT: checkout scipy.optimize.minimize

    K = returns.size
    init_value = 1
    returns_norm = returns - np.max(returns)
    def obj_function(x, *args):
    	returns, epsilon = args
    	sum_return = 0
    	K = returns.shape[0]
    	sum_return = np.sum(np.exp(returns/x))
    	cost = x*np.log(sum_return/K)+x*epsilon
    	return cost
    bound_ = ((0.00000001, None),)
    eta = minimize(obj_function, init_value, args=(returns_norm,self.epsilon), bounds=bound_)  #method='Nelder-Mead', tol=1e-6,
    alpha = np.exp(returns_norm/eta.x)

    print(bcolors.OKGREEN + 'Alpha for REPS update is ' + str(alpha)+bcolors.ENDC)
    return alpha
    #raise NotImplementedError

  def fit_gaussian(self, alpha, w):
    """
    ----------------------- TODO ---------------
    Updates the gaussian distribution parameters (mean, std) based on w and alpha

    Parameters
    ----------
    alpha : numpy array of size (num_rollouts, 1)
    w : numpy array of size (num_rollouts, NUM_DIM)

    Returns
    ----------
    weight_mean : numpy array of size (NUM_DIM, 1)
    weight_std : numpy array of size (NUM_DIM, 1)
    """

    weight_mean = np.dot(np.transpose(alpha),w)/np.sum(alpha)
    sub = np.subtract(w,weight_mean)
    weight_std = np.sqrt(np.dot(np.transpose(alpha),np.square(sub)))/np.sum(alpha)

    print(weight_mean)
    print(weight_std)
    return weight_mean, weight_std

  def test(self):
    """
    Final testing function; use the mean of the distribution as the DMP policy parameter
    """
    # Use the mean weights
    w = self.weight_mean.flatten().reshape((NUM_DIM))

    # # Get start and goal state
    # start_state = self.env.RobotPosition
    # goal_state = self.env.GoalPosition
    
    # Get dmp trajectory
    trajectory, x = self.policy.get_trajectory(w, goal_state, start_state)

    # Execute trajectory on environment
    return_eval = self.env.executeTrajectory(trajectory)
    print(bcolors.BOLD+'At test, the return obtained is '+str(return_eval)+bcolors.ENDC)
    return return_eval

