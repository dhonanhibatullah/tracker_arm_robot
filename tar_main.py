from modules.tar_graphic import *
from modules.tar_simulator import *



# Initial values
theta_0     = np.array([[-np.pi/2 + 0.1*np.random.rand()], [0.1*np.random.rand()]])
theta_dot_0 = np.array([[0.0], [0.0]])



# Initiate graphic and simulator
tar_graphic     = TarGraphic()
tar_simulator   = TarSimulator(
    l1              = 1.0,
    l2              = 1.0,
    lc1             = 0.5,
    lc2             = 0.5,
    m1              = 1.0,
    m2              = 1.0,
    theta_init      = theta_0,
    theta_dot_init  = theta_dot_0,
)



# Determine the control law
KP          = 80
KI          = 12
KD          = 4
MAX_TORQUE  = 100

dt      = 0.016667
time    = 0.0

last_xd     = theta_0
last_xd_dot = theta_dot_0

int_xe  = 0.0 
last_xe = 0.0


def calcControlLaw() -> np.ndarray:
    
    # Get global variables
    global last_xd
    global last_xd_dot
    global int_xe
    global last_xe


    # Retrieve all the informations needed
    cursor      = tar_graphic.getMouseHover()
    reachable   = cursor[2]                                 # This tells us about does the cursor reachable by the arm

    if reachable:
        xd          = np.array([[cursor[0]], [cursor[1]]])      # Desired x
        xd_dot      = (xd - last_xd)/dt                         # Desired x_dot
        xd_ddot     = (xd_dot - last_xd_dot)/dt                 # Desired x_ddot

        theta       = tar_simulator.theta                       # Current robot theta
        theta_dot   = tar_simulator.theta_dot                   # Current robot theta_dot
        xn          = tar_simulator.fk_x2(theta)                # Current robot x

        M           = tar_simulator.M(theta)                    # Current robot M matrix
        J           = tar_simulator.J(theta)                    # Current robot J matrix
        C           = tar_simulator.C(theta, theta_dot)         # Current robot C matrix
        G           = tar_simulator.G(theta)                    # Current robot G matrix


        # Calculate errors
        xe          = xd - xn
        int_xe      += (last_xe + xe)*dt/2.0
        thetae_dot  = np.linalg.inv(J)@xd_dot - theta_dot


        # Compute control law
        F = xd_ddot + KP*xe + KI*int_xe
        torque = np.clip(M@(J.T@F + KD*thetae_dot) + C@theta_dot + G, -MAX_TORQUE, MAX_TORQUE)


        # Update buffers
        last_xd     = xd
        last_xd_dot = xd_dot
        last_xe     = xe


    else:
        torque = np.array([
            [0.0],
            [0.0]
        ])


    # Return control value
    return torque



# Simulation
while True:

    # Step simulation
    tar_simulator.torqueInput(calcControlLaw())
    tar_simulator.stepSimulation()
    time = tar_simulator.getSimulationTime()


    # Render animation
    theta1, theta2 = tar_simulator.getTheta()
    tar_graphic.stepRender(theta1, theta2)