import numpy as np



class TarSimulator:


    def __init__(self, l1:float, l2:float, lc1:float, lc2:float, m1:float, m2:float, theta_init:np.ndarray, theta_dot_init:np.ndarray, timestep:float=0.016667) -> None:
        
        # Retrieve the constructor arguments
        self.l1             = l1
        self.l2             = l2
        self.lc1            = lc1
        self.lc2            = lc2
        self.m1             = m1
        self.m2             = m2
        self.b1             = 0.09
        self.b2             = 0.07
        self.g              = 9.80665
        self.dt             = timestep
        self.max_length     = l1 + l2
        self.I1             = (0.667)*(m1*lc1*l1)
        self.I2             = (0.667)*(m2*lc2*l2)


        # States
        self.theta      = theta_init
        self.theta_dot  = theta_dot_init
        self.tau        = np.array([[0.0], [0.0]])
        self.time       = 0.0


        # Calculate constant values inside matrices
        # M matrix
        self.M11_const_a = m1*lc1**2 + m2*(l1**2 + lc2**2) + self.I1 + self.I2
        self.M11_const_b = 2.0*m2*l1*lc2
        self.M12_const_a = m2*lc2**2 + self.I2
        self.M12_const_b = m2*l1*lc2
        self.M21_const_a = self.M12_const_a
        self.M21_const_b = self.M12_const_b
        self.M22_const_a = m2*lc2**2 + self.I2
        self.M = lambda theta: np.array([
            [self.M11_const_a + self.M11_const_b*np.cos(theta[1].item()), self.M12_const_a + self.M12_const_b*np.cos(theta[1].item())],
            [self.M21_const_a + self.M21_const_b*np.cos(theta[1].item()), self.M22_const_a]
        ])


        # C matrix
        self.C_const = m2*l1*lc2
        self.C = lambda theta, theta_dot: np.array([
            [-2.0*self.C_const*np.sin(theta[1].item())*theta_dot[1].item(), -self.C_const*np.sin(theta[1].item())*theta_dot[1].item()],
            [self.C_const*np.sin(theta[1].item())*theta_dot[0].item(), 0]
        ])


        # b matrix
        self.b = np.array([
            [self.b1, 0],
            [0, self.b2]
        ])


        # G matrix
        self.G_const_a = m1*self.g*lc1 + m2*self.g*l1
        self.G_const_b = m2*self.g*lc2
        self.G = lambda theta: np.array([
            [self.G_const_a*np.cos(theta[0].item()) + self.G_const_b*np.cos(theta[0].item() + theta[1].item())],
            [self.G_const_b*np.cos(theta[0].item() + theta[1].item())]
        ])


        # Jacobian
        self.J = lambda theta : np.array([
            [-self.l1*np.sin(theta[0].item()) - self.l2*np.sin(theta[0].item() + theta[1].item()), -self.l2*np.sin(theta[0].item() + theta[1].item())],
            [self.l1*np.cos(theta[0].item()) + self.l2*np.cos(theta[0].item() + theta[1].item()), self.l2*np.cos(theta[0].item() + theta[1].item())]
        ])


        # Forward kinematics
        self.fk_x1  = lambda theta: np.array([
            [self.l1*np.cos(theta[0].item())], 
            [self.l1*np.sin(theta[0].item())]
        ])
        self.fk_x2  = lambda theta: self.fk_x1(theta) + np.array([
            [self.l2*np.cos(theta[0].item() + theta[1].item())], 
            [self.l2*np.sin(theta[0].item() + theta[1].item())]
        ])



    def getSimulationTime(self) -> None:
        return self.time



    def getTheta(self) -> None:
        return self.theta[0].item(), self.theta[1].item()



    def torqueInput(self, torques:np.ndarray) -> None:
        self.tau = torques



    def stepSimulation(self) -> None:

        # Calculate theta_ddot
        theta       = self.theta
        theta_dot   = self.theta_dot
        tau         = self.tau
        theta_ddot  = np.linalg.inv(self.M(theta))@(tau - (self.C(theta, theta_dot) + self.b)@theta_dot - self.G(theta))


        # Compute theta and theta_dot
        self.theta_dot  = theta_dot + theta_ddot*self.dt
        self.theta      = theta + self.theta_dot*self.dt


        # Update time
        self.time += self.dt