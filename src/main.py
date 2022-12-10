import numpy as np
from bat_algo import Bat_Algorithm

if __name__ == "__main__":
    # np.random.seed(200543)

    # Program was also tested within UR_RTDE
    # Used library:
    #    https://sdurobotics.gitlab.io/ur_rtde/
    #
    # import rtde_control
    # rtde_c = rtde_control.RTDEControlInterface("192.168.19.128")
    
    # testing demo points translation
    demo = [
        [-0.21, -0.24, 0.2],
        [-0.21, -0.24, 0.5],
        [0, -0.12, 0.42],
    ]

    # translation + rotation
    # target = np.append([-0.21, -0.24, 0.2], [np.deg2rad(-180), np.deg2rad(0), np.deg2rad(-180)])
    # q, eval, time = Bat_Algorithm(target).bat_algo()
    
    for i in range(len(demo)):
        q, eval, time = Bat_Algorithm(demo[i]).bat_algo()
        print("\033[91m" + f'[RESULT] q = {np.rad2deg(q, dtype=np.float16)}, error = {eval:.4f}, t = {time:.4f}' + "\033[0m")
        
        # rtde_c.moveJ(q)
