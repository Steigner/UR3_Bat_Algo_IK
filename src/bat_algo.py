import time
import itertools
import numpy as np
# Parameters of Bat Algo
from params import Params
# Get objective function
from obj_func import Obj_Func

class Bat_Algorithm(Params):
    def __init__(self, target: np.ndarray) -> None:
        """
        Initialization parameters of Bat algorithm

        """
        super(Bat_Algorithm, self).__init__()
        
        # minimum fitness
        self.best_eval = None 

        #lower bound
        self.Lb = np.zeros(self.dim) 

        # upper bound
        self.Ub = np.zeros(self.dim)   
        
        # frequency
        self.f = np.zeros(self.NP) 
        
        # fitness
        self.Fitness = np.zeros(self.NP) 
        
        # best solution
        self.best = np.zeros(self.dim) 

        # velocity
        self.v = np.zeros((self.NP, self.dim))

        # population of solutions
        self.Sol = np.zeros((self.NP, self.dim))

        # objective function with target of IK
        # target = np.append([-0.21, -0.24, 0.2], [-180, 0, -180])
        # target = [-0.21, -0.24, 0.2]
        self.func = Obj_Func(target).comp_error

    def __init_bat(self) -> None:
        """
        Setting up Bat algorithm

        """
        
        l = 0
        # find the best fitness value and record the index on the variable j
        self.Lb[:] = self.dodParam[0]
        self.Ub[:] = self.dodParam[1]

        self.f = np.zeros(self.NP)
        self.v = np.zeros((self.NP, self.dim))

        # store the values ​​of each dimension on the best solution
        for i, j in itertools.product(range(self.NP), range(self.dim)):
            rnd = np.random.uniform(0, 1)
            self.Sol[i,j] = self.Lb[j] + (self.Ub[j] - self.Lb[j]) * rnd
            self.Fitness[i] = self.func(self.Sol[i,:])

            if self.Fitness[i] < self.Fitness[l]:
                l = i

        self.best = self.Sol[l,:]
        # save the fitness value of the best solution
        self.best_eval = self.Fitness[l]

    def __bounds(self, s: np.float64, lower: np.float64, upper: np.float64) -> np.float64:
        """
        compute bounderies

        :param s: represents old joint in algo
        :param lower: down boundery
        :param upper: up boundery
        :returns s: represent new joint corrected by boundery 
        """
        # if the value exceeds the upper limit then set the value to be the upper limit
        if s > upper:
            s = upper
        
        # if the value is less than the lower limit then set the value to be the lower limit
        elif s < lower:
            s = lower

        return s

    def __rand_sam(self) -> np.float64:
        """
        compute randomize sample

        :returns: random sample
        """
        return np.random.random_sample()

    def bat_algo(self) -> tuple[np.ndarray, np.float64, np.float64]:
        """
        Bat Algo

        :returns: joints, error, time of final computation
        """

        times = self.nRuns 
        q_out = np.zeros((times,self.dim))
        eval = np.full(times, float('inf'))
        t = np.array([])
        
        for run_i in range(times):
            start = time.time()
            
            # solution matrix (number of bats x dimension)
            S = np.zeros((self.NP, self.dim))

            self.__init_bat()

            for _, i in itertools.product(range(self.maxGener), range(self.NP)):
                rnd = np.random.uniform(0, 1)
                # find the frequency of each bat using eq. 2 of the bat algorithm
                self.f[i] = self.pF[0] + (self.pF[1] - self.pF[0]) * rnd
                
                # find the new v and x of each bat using eq. 3 and 4 of the bat algorithm
                for j in range(self.dim):
                    self.v[i,j] = self.v[i,j] + (self.Sol[i,j] - self.best[j]) * self.f[i]
                    S[i,j] = self.Sol[i,j] + self.v[i,j]
                    S[i,j] = self.__bounds(S[i,j], self.Lb[j], self.Ub[j])
                
                rnd = self.__rand_sam()

                # if the random value [0,1] is greater than the pulse rate of the bat, then do a local search based on the best bat
                if rnd > self.pR:
                    for j in range(self.dim):
                        S[i,j] = self.best[j] + 0.001 * np.random.normal(-1, 1)
                        S[i,j] = self.__bounds(S[i,j], self.Lb[j], self.Ub[j])

                # calculate the fitness value of the new solution 
                Fnew = self.func(S[i,:])
                
                rnd = self.__rand_sam()

                if (Fnew <= self.Fitness[i]) and (rnd < self.pA):
                    # change the best solution
                    self.Sol[i,:] = S[i,:]
                    self.Fitness[i] = Fnew
                
                if Fnew <= self.best_eval:
                    self.best[:] = S[i,:]
                    self.best_eval = Fnew

            q_out[run_i,:] = self.best
            eval[run_i] = self.best_eval
            
            # t.append(time.time() - start)
            t = np.append(t, [time.time() - start])

            print(f'[INFO] q = {q_out[run_i]}, error = {eval[run_i]:.4f}')
            
            if(eval[run_i] < 0.01):
                break
        
        return q_out[run_i], eval[run_i], t[run_i]