import numpy as np
from numpy import pi, cos, sin

def FK(th) -> np.ndarray:
    """
    Computation of Forward Kinematics by classics D-H tables  

    :param th: joints angle
    """
    o = [pi/2, 0, 0, pi/2, -pi/2, 0]
    d = [0.1519, 0, 0, 0.11235, 0.08535, 0.0819]
    a = [0, -0.24365, -0.21325, 0, 0, 0]
    
    for i in range(6):
        A_x = np.array([
            [cos(th[i]),  -sin(th[i]) * cos(o[i]),     sin(th[i]) * sin(o[i]),     a[i] * cos(th[i])], 
            [sin(th[i]),   cos(th[i]) * cos(o[i]),    -cos(th[i]) * sin(o[i]),     a[i] * sin(th[i])],
            [0       ,     sin(o[i]),                  cos(o[i]),                  d[i]             ],
            [0       ,     0,                          0,                          1                ]           
        ])

        if i == 0:
            A = A_x
        
        else:
            A = A @ A_x 
    
    return A[:3,3]
    # 2 option for translation + rotation
    # return A