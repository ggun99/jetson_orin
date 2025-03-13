import numpy as np

class AdmittanceController:
    def __init__(self, M, B, K):
        """
        Initialize the Admittance Controller.

        Parameters:
            M: Diagonal mass matrix (6x6) [kg]
            B: Diagonal damping matrix (6x6) [Ns/m]
            K: Diagonal stiffness matrix (6x6) [N/m]
            dt: Time step [s]
        """
        self.M = M
        self.B = B
        self.K = K
        # self.dt = dt

        # State variables
        self.position = np.zeros(6)  # [x, y, z, roll, pitch, yaw]
        self.velocity = np.zeros(6)  # [vx, vy, vz, omega_roll, omega_pitch, omega_yaw]

    def update(self, force_torque, dt):
        """
        Update the position and velocity based on the force-torque input.

        Parameters:
            force_torque: External force-torque vector (6x1) [N, Nm]

        Returns:
            position: Updated position (6x1) [m, rad]
        """
        # Calculate acceleration: a = M^(-1) * (F - B*v - K*x)
        # acc = np.linalg.inv(self.M) @ (force_torque - self.B @ self.velocity - self.K @ self.position)

        # if np.any(np.isnan(force_torque)) or np.any(np.isinf(force_torque)):
        #     print("force_torque contains invalid values.")
        # if np.any(np.isnan(self.velocity)) or np.any(np.isinf(self.velocity)):
        #     print("self.velocity contains invalid values.")
        # if np.any(np.isnan(self.position)) or np.any(np.isinf(self.position)):
        #     print("self.position contains invalid values.")
        # if np.any(np.linalg.inv(self.M)) or np.any(np.linalg.inv(self.M)):
        #     print("self.M contains invalid values.")
        #     print("Contains NaN:", np.isnan(self.M).any())
        #     print("Contains Inf:", np.isinf(self.M).any())
        acc = np.linalg.inv(self.M) @ (force_torque - self.B @ self.velocity - self.K @ self.position)
        # inv_M = np.linalg.inv(self.M)
        # print("Inverse of M:", inv_M)
        # Update velocity: v = v + a * dt
        self.velocity += acc * dt

        # Update position: x = x + v * dt
        self.position += self.velocity * dt

        return self.position, acc

    def forward_kinematics_ur5e(self, q):
        
        T = np.array([ 
                [ np.cos(q[5]) * (np.sin(q[0]) * np.sin(q[4]) + np.cos(q[1] + q[2] + q[3]) * np.cos(q[0]) * np.cos(q[4])) - np.sin(q[1] + q[2] + q[3]) * np.cos(q[0]) * np.sin(q[5]), -np.sin(q[5]) * (np.sin(q[0]) * np.sin(q[4]) + np.cos(q[1] + q[2] + q[3]) * np.cos(q[0]) * np.cos(q[4])) - np.sin(q[1] + q[2] + q[3]) * np.cos(q[0]) * np.cos(q[5]), np.cos(q[4]) * np.sin(q[0]) - np.cos(q[1] + q[2] + q[3]) * np.cos(q[0]) * np.sin(q[4]), d6 * (np.cos(q[4]) * np.sin(q[0]) - np.cos(q[1] + q[2] + q[3]) * np.cos(q[0]) * np.sin(q[4])) + d4 * np.sin(q[0]) + a2 * np.cos(q[0]) * np.cos(q[1]) + d5 * np.sin(q[1] + q[2] + q[3]) * np.cos(q[0]) + a3 * np.cos(q[0]) * np.cos(q[1]) * np.cos(q[2]) - a3 * np.cos(q[0]) * np.sin(q[1]) * np.sin(q[2]) ],
                [ -np.cos(q[5]) * (np.cos(q[0]) * np.sin(q[4]) - np.cos(q[1] + q[2] + q[3]) * np.cos(q[4]) * np.sin(q[0])) - np.sin(q[1] + q[2] + q[3]) * np.sin(q[0]) * np.sin(q[5]), np.sin(q[5]) * (np.cos(q[0]) * np.sin(q[4]) - np.cos(q[1] + q[2] + q[3]) * np.cos(q[4]) * np.sin(q[0])) - np.sin(q[1] + q[2] + q[3]) * np.cos(q[5]) * np.sin(q[0]), -np.cos(q[0]) * np.cos(q[4]) - np.cos(q[1] + q[2] + q[3]) * np.sin(q[0]) * np.sin(q[4]), a2 * np.cos(q[1]) * np.sin(q[0]) - d4 * np.cos(q[0]) - d6 * (np.cos(q[0]) * np.cos(q[4]) + np.cos(q[1] + q[2] + q[3]) * np.sin(q[0]) * np.sin(q[4])) + d5 * np.sin(q[1] + q[2] + q[3]) * np.sin(q[0]) + a3 * np.cos(q[1]) * np.cos(q[2]) * np.sin(q[0]) - a3 * np.sin(q[0]) * np.sin(q[1]) * np.sin(q[2]) ],
                [ np.cos(q[1] + q[2] + q[3]) * np.sin(q[5]) + np.sin(q[1] + q[2] + q[3]) * np.cos(q[4]) * np.cos(q[5]), np.cos(q[1] + q[2] + q[3]) * np.cos(q[5]) - np.sin(q[1] + q[2] + q[3]) * np.cos(q[4]) * np.sin(q[5]), -np.sin(q[1] + q[2] + q[3]) * np.sin(q[4]), d1 + d5 * (np.sin(q[1] + q[2]) * np.sin(q[3]) - np.cos(q[1] + q[2]) * np.cos(q[3])) + a3 * np.sin(q[1] + q[2]) + a2 * np.sin(q[1]) - d6 * np.sin(q[4]) * (np.cos(q[1] + q[2]) * np.sin(q[3]) + np.sin(q[1] + q[2]) * np.cos(q[3])) ],
                [0, 0, 0, 1]
                ])
        return T

# Example usage
if __name__ == "__main__":
    # Define system parameters
    M = np.diag([1.0, 1.0, 1.0, 0.1, 0.1, 0.1])  # Mass matrix
    B = np.diag([20.0, 20.0, 20.0, 2.0, 2.0, 2.0])  # Damping matrix
    K = np.diag([50.0, 50.0, 50.0, 5.0, 5.0, 5.0])  # Stiffness matrix
    dt = 0.01  # Time step

    # Initialize controller
    admittance_controller = AdmittanceController(M, B, K)

    # Simulated force-torque input (example)
    force_torque_input = np.array([5.0, 0.0, -2.0, 0.0, 0.1, 0.0])  # [Fx, Fy, Fz, Tx, Ty, Tz]

    # Update loop
    for step in range(100):
        position = admittance_controller.update(force_torque_input)
        print(f"Step {step}: Position: {position}")