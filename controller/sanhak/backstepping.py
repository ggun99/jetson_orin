import numpy as np

class Backstepping():

    def __init__(self):
        pass

    def backstepping_control(self, x, y, x_d, y_d, x_dp, y_dp, x_dpp, y_dpp, theta, K1=1.0, K2=0.1, K3=1.0):
            e_x = x_d - x
            e_y = y_d - y
            e_theta = np.arctan2(y_dp, x_dp) - theta

            T_e = np.array([[np.cos(theta), np.sin(theta), 0], [-np.sin(theta), np.cos(theta), 0], [0, 0, 1]])
        
            mat_q = T_e @ np.array([[e_x],[e_y],[e_theta]])

            v_r = np.sqrt(x_dp**2+y_dp**2)
            if np.abs(x_dp**2 + y_dp**2) < 0.01:
                w_r = 0.0
            else: 
                w_r = (x_dp*y_dpp - y_dp*x_dpp)/(x_dp**2 + y_dp**2)
            print('mat_q[2,0]', mat_q[2,0])
            print('mat_q[0,0]', mat_q[0,0])
            v_c = v_r*np.cos(mat_q[2,0]) + K1*mat_q[0,0]
            # v_c = -v_c
            w_c = w_r + K2*v_r*mat_q[1,0] + K3*np.sin(mat_q[2,0])

            return v_c, w_c