import numpy as np
from numpy import cos, sin
from FK import FK

class Obj_Func(object):
    def __init__(self, target) -> None:
        self.target = target
        # 2 option for translation + rotation
        # self.T_desired = self.__comp_t_desired(target)

    def __comp_t_desired(self, target: np.ndarray) -> np.array:
        """
        Computation target Forward Kinematics D-H table

        :param target: translation, rotation of target
        :returns: 4x4 d-h table of target  
        """
        x = target[0]
        y = target[1]
        z = target[2]
        
        alfa = target[3]
        beta = target[4]
        gamma = target[5]
        
        R_x = np.array([
            [1, 0,           0        ],
            [0, cos(alfa),  -sin(alfa)],
            [0, sin(alfa),   cos(alfa)] 
        ])
        
        R_y = np.array([
            [cos(beta),  0, sin(beta)],
            [0,          1, 0        ],
            [-sin(beta), 0, cos(beta)] 
        ])
        
        R_z = np.array([
            [cos(gamma), -sin(gamma),  0], 
            [sin(gamma),  cos(gamma),  0],
            [0,           0,           1]
        ])

        R = R_z @ R_y @ R_x
        
        return np.r_[
            np.c_[
                R, np.array([x,y,z])
            ], [np.array([0,0,0,1])]
        ]

    # input: x = [x] depands on input dimension, for more dimension change dodParam, more dimension ex. x = [x,x,x,x] 
    # return computed rastrigins f(x) by given points
    def comp_error(self, q: np.ndarray) -> np.float64:
        T_current = FK(q)
        return np.linalg.norm(T_current - self.target)
        
        # 2 option for translation + rotation
        # return np.linalg.norm(T_current - self.T_desired)