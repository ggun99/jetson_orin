import numpy as np


class Bezier():
    def __init__(self):
        pass

    def bezier_curve(self, control_points, t_robot, total_time=10):
        t = t_robot/total_time
        """Calculates a Bezier curve from a set of control points."""
        x_d = control_points[0][0]*(1-t)**2 + control_points[1][0]*2*t*(1-t) + control_points[2][0]*t**2
        y_d = control_points[0][1]*(1-t)**2 + control_points[1][1]*2*t*(1-t) + control_points[2][1]*t**2
        print(x_d)
        print(y_d)
        x_dp = -2*control_points[0][0] + control_points[0][0]*2*t + 2*control_points[1][0] - 4*control_points[1][0]*t + 2*control_points[2][0]*t
        y_dp = -2*control_points[0][1] + control_points[0][1]*2*t + 2*control_points[1][1] - 4*control_points[1][1]*t + 2*control_points[2][1]*t
        x_dpp = 2*control_points[0][0] - 4*control_points[1][0] + 2*control_points[2][0]
        y_dpp = 2*control_points[0][1] - 4*control_points[1][1] + 2*control_points[2][1]
        print(x_dp)
        print(y_dp)
        return x_d, y_d, x_dp, y_dp, x_dpp, y_dpp