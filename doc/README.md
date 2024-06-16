# Motion and Dynamic equaion
inference some need dynmaic and kinematic equation for quadcopter

quadcopter according to the motor thrust $f_1, f_2,f_3, f_4$, compute the the accleration and integrate to be current attitude. And controller according to current attitude with target waypoint get error, and use error to generate 

## Allocation Matrix

$$
\begin{bmatrix}f\\\tau_x\\\tau_y\\\tau_Z\end{bmatrix}=\begin{bmatrix}1&1&1&1\\L&0&-L&0\\0&L&0&-L\\C_{\tau f}&C_{\tau f}&C_{\tau f}&C_{\tau f}\end{bmatrix}\begin{bmatrix}f_1\\f_2\\f_3\\f_4\end{bmatrix}
$$

which $C_{\tau f}$ is the lift drag coefficient (drag constant). Thus, from the force (thrust) of motor $f_1,f_2,f_3,f_4$, we can obtain the force in body frame $\mathcal F^B$ and the acceleration in inertia frame $\mathcal F^I$ will be

$$
\begin{bmatrix}\ddot{x}\\\ddot{y}\\\ddot{z}\end{bmatrix}=\begin{bmatrix}0\\0\\-g\end{bmatrix}+\frac{1}m{R^I_B}\begin{bmatrix}0\\0\\f
\end{bmatrix}
$$

and the angular accelerations will be
$$
\begin{bmatrix}\ddot{\phi}\\\ddot{\theta}\\\ddot{\psi}\end{bmatrix}=\mathcal J^{-1}\Bigg(\begin{bmatrix}\tau_x\\\tau_y\\\tau_Z\end{bmatrix}-\omega\times\mathcal J\omega
 \Bigg)
$$

and we can integrate the acceleration (which typically obtain by sensor) into the velocity and position

## Controller

