from numpy.linalg import cholesky
from scipy.optimize import minimize
import numpy as np
from sklearn.gaussian_process import GaussianProcessRegressor
from sklearn.gaussian_process.kernels import ConstantKernel, RBF
from gaussian_processes_util import plot_gp
from numpy.linalg import inv

from numpy.linalg import inv

class GaussianProcess:
    def __init__(self):
        self.gpr = None
        self.noise = 0.6
        self.location = '../data/dataset_collection.npy'

    def fit_GP(self):
        
        data = np.load(self.location)
        X_train = data[:, 0:7]
        Y_train = data[:, 7]

        rbf = ConstantKernel(1.0) * RBF(length_scale=1.0)
        self.gpr = GaussianProcessRegressor(kernel=rbf, alpha=self.noise**2)

        # Reuse training data from previous 1D example
        self.gpr.fit(X_train, Y_train)

        # Obtain optimized kernel parameters
        l = self.gpr.kernel_.k2.get_params()['length_scale']
        sigma_f = np.sqrt(self.gpr.kernel_.k1.get_params()['constant_value'])

        print('Fitted a gaussian process on the data. Parameters are as follows: ')
        print('Length scale of the kernel ', l)
        print('Height of the kernel ', sigma_f)

    def find_reward(self, sample):
        # X_mean = [0, 0, 0, 0.05, 150, 2, 20]
        # X_cov = [np.pi/9, np.pi/9, np.pi/9, 0.01, 50, 0.6, 5]
        # X_sample = [0, 0, 0, 0, 0, 0, 0]

        # for idx, (constraint_mean, constraint_cov) in enumerate(zip(X_mean, X_cov)):
        #             X_sample[idx] = np.random.normal(constraint_mean, constraint_cov)
        sample = np.asarray(sample).reshape(-1, 7)
        # print(X_sample)

        # Compute posterior predictive mean and covariance
        mu_s, cov_s = self.gpr.predict(sample, return_cov=True)
        print('Reward', mu_s, 'Covariance of the reward', cov_s)
