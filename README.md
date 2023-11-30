# Tracker Arm Robot
To run the script, make sure you have installed the following dependencies:
```
pip install numpy
pip install matplotlib
pip install pygame
```

## Short Introduction
The dynamics of an armature linkage can be written generally as

$$\tau = M(\theta)\ddot{\theta} + C(\theta, \dot{\theta})\dot{\theta} + g(\theta)$$

or in this particular case:

$$\begin{bmatrix}\tau_1 \\ \tau_2\end{bmatrix}=
\begin{bmatrix}m_1l_{c1}^2+m_2(l_1^2+l_{c2}^2+2l_1l_{c2}\cos(\theta_2))+I_1+I_2 & m_2(l_{c2}^2+l_1l_{c2}\cos(\theta_2))+I_2 \\ m_2(l_{c2}^2+l_1l_{c2}\cos(\theta_2))+I_2 & m_2l_{c2}^2+I_2\end{bmatrix}\begin{bmatrix}\ddot{\theta}_1 \\ \ddot{\theta}_2\end{bmatrix} + \begin{bmatrix}-2m_2l_1l_{c2}\sin(\theta_2)\dot{\theta}_2 & -m_2l_1l_{c2}\sin(\theta_2)\dot{\theta}_2 \\ m_2l_1l_{c2}\sin(\theta_2)\dot{\theta}_1 & 0\end{bmatrix}\begin{bmatrix}\dot{\theta}_1 \\ \dot{\theta}_2\end{bmatrix} - \begin{bmatrix}m_1gl_{c1}\cos(\theta_1)+m_2g(l_1\cos(\theta_1)+l_{c2}\cos(\theta_1 + \theta_2)) \\ m_2gl_{c2}\cos(\theta_1 + \theta_2)\end{bmatrix}$$

We can rewrite the equation as follows to simulate the dynamics.

$$\ddot{\theta} = M^{-1}(\theta)\left( \tau - C(\theta, \dot{\theta})\dot{\theta} - g(\theta) \right)$$

To make the dynamics less ideal, let us add some viscous frictions on each joint with a positive semi-definite matrix $b$ such that:

$$\ddot{\theta} = M^{-1}(\theta)\left( \tau - \left(C(\theta, \dot{\theta}) + b\right)\dot{\theta} - g(\theta) \right)$$

The control law used for $\tau$ is

$$F = \ddot{x}_d(t) + K_px_e(t) + K_i\int_0^tx_e(t)dt$$
$$\tau = \tilde{M}(\theta)\left(J^TF + K_d\left(J^{-1}\dot{x}_d - \dot{\theta} \right) \right) + \tilde{h}(\theta, \dot{\theta})$$

You can tune $K_p$, $K_i$, and $K_d$ values in tar_main.py.