"""
Bezier Curves written for Tech Made Simple by Devansh
Attribute code to Devansh- https://artificialintelligencemadesimple.substack.com/
"""

import numpy as np
import matplotlib.pyplot as plt

def bezier_curve(control_points, num_points=400):
    """Calculates a Bezier curve from a set of control points."""
    n = len(control_points) - 1

    t = np.linspace(0, 1, num_points)
    curve = np.zeros((num_points, 2))
    # curve_v = np.zeros((num_points, 2))
    # curve_a = np.zeros((num_points, 2))

    for i in range(num_points):
        for j in range(n + 1):
            curve[i] += (
                control_points[j]
                * (
                    np.math.factorial(n)
                    / (np.math.factorial(j) * np.math.factorial(n - j))
                )
                * t[i] ** j
                * (1 - t[i]) ** (n - j)
            )
            # curve_v[i] += (
            #     control_points[j]
            #     * (
            #         np.math.factorial(n)
            #         / (np.math.factorial(j) * np.math.factorial(n - j))
            #     )
            #     (j * t[i] ** (j-1) * (1 - t[i]) ** (n - j) - (n - j) * t[i] ** j * (1-t[i]) ** (n-j-1))
            # )
    return curve #, curve_v, curve_a

# Example control points
control_points = np.array([[0, 0], [266, -94], [444, -118]])
curve = bezier_curve(control_points)
print(curve)
plt.plot(curve[:, 0], curve[:, 1])  # Plot the curve
plt.scatter(control_points[:, 0], control_points[:, 1])  # Plot control points
plt.show()