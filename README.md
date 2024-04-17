![Header-removebg](https://github.com/jeffxhansen/FlappyBirdOptimalControl/assets/62043475/c1e1351d-213c-45f5-b4ca-f1767beccdd2)

By: Jeff Hansen, Gwen Martin, Benjamin McMullin, Eliza Powell

![animation_gif](https://github.com/poweleli/flappy_bird/assets/62043475/2c89792c-7496-4769-af15-2f76f412b627)

# Abstract
We employ optimal control techniques to create a solution to the popular mobile game Flappy Bird. Our obstacle avoidance model allows the bird to safely navigate through a series of pipes by adding spikes of acceleration to contend with the constant pull of gravity. We formulate this problem with a cost functional that contains a reward and cost component. We reward the control when it successfully guides the bird in between the pipes, and we penalize the control when its shape deviates from a tapping motion. 

# Problem Statement and Motivation
Flappy Bird, a mobile game released in 2013, swiftly ascended to become the most popular free game by January 2014 [`Ang`]. The premise of this addictive game is simple: avoid obstacles. In Flappy Bird, a bird advances forward at a constant pace, and players must guide it through a series of randomly positioned vertical pipes at equally spaced intervals. Users exert control solely over the bird's vertical movement, achieved by tapping the screen to provide a boost to counter the bird's constant descent.

This project aims to optimize the bird's trajectory by determining the optimal moments for screen taps, ensuring safe passage through the obstacles. While prior research has explored optimization strategies for Flappy Bird and similar games, predominantly through deep learning methods like q-networks and genetic algorithms [`Gan22`], our approach distinguishes itself by leveraging optimization and control techniques such as Pontryagin's Maximum Principle. By mathematically modeling the optimal flight path, our methods offer a framework for gameplay efficiency.

# Methods and Approach
We explore two strategies for maneuvering an object through obstacles. Subsequently, we adjust our control parameters to emulate a discrete tapping motion. 

## Obstacle Avoidance
Our initial approach to obstacle navigation tackles avoiding pipes with defined x and y dimensions. Consequently, the state representation for the Flappy Bird includes both its x and y positions and velocities. This approach necessitates several modifications to the original Obstacle Avoidance lab.

We create a continuous and differentiable obstacle to simulate a pipe by utilizing the equation of an ellipse with a larger radius along the y-axis and a smaller radius along the x-axis. We set our radius along the x-axis to be .1 and then vary the y-axis radius depending on the height of the pipe. $W_1$ serves as a constant that can be adjusted to control the penalty when the bird encounters an obstacle along its path, and the value $\lambda$ specifies the vanishing rate of the penalty.  Putting all of this together yields the following pipe obstacle cost:

$$\begin{align*}
C(x, y) = \frac{W_1}{\left( \frac{(x - c_x)^2}{r_x} + \frac{(y - c_y)^2}{r_y} \right)^{\lambda} + 1}
\end{align*}$$

Adding this penalty to the cost functional deters the bird from navigating near the pipe. Our control $u(t)$ governs the acceleration of the bird in the horizontal and vertical direction. To penalize large acceleration values to mimic the game physics, we add a $W_2 |u(t)|^2$ term to the cost functional. $C_l$ and $C_u$ represent the lower and upper pipe obstacles respectively. Combining these components yields our cost functional and state space:  

$$\begin{align*}
J[u] = \int_{t_0}^{t_f} \left( 1 + C_l(x(t), y(t)) + C_u(x(t), y(t)) + W_2 |u(t)|^2 \right) \, dt
\end{align*}$$

$$\begin{align*}
% state equation
\mathbf{x}'(t) = \begin{bmatrix} x(t) \\ y(t) \\ x'(t) \\ y'(t) \end{bmatrix}',  \quad \mathbf{x}(0) =  \begin{bmatrix} x_0 \\ y_0 \\ 0 \\ 0 \end{bmatrix},  \quad \mathbf{x}(t_f) =  \begin{bmatrix} x_{t_f} \\ y_{t_f} \\ 0 \\ 0 \end{bmatrix}
\end{align*}$$

![obstacle_avoidance](https://github.com/jeffxhansen/FlappyBirdOptimalControl/assets/62043475/5c18a21e-23b0-4cdc-9093-4a877b849347)

As seen above, this approach performs well in some situations, but fails in others. The solution's success depends largely on the start and end state locations, so if the bird starts and ends too high the path will collide with the pipes, even with large values of $W_2$. Because of the narrow shape of the pipes, we infer that the solver prefers quickly passing through the pipe instead of expending the effort to go around the length of the pipe. 

Also, in the actual Flappy Bird game, the user cannot change the $x''(t)$ acceleration since the forward velocity is constant. When we alter this system to have constant x-velocity and no control for the $x''(t)$ term, the numerical solver raises a `Singular Jacobian Matrix` error. This arises from our state-space matrix containing an entire row of zeros (from the constant velocity and zero change in acceleration). To combat this error, we remove the $x(t)$ component in our state-space and restructure the problem as shown in the following section.

## Endpoint Reward

In our second approach, we include a reward for passing in between the obstacle's gap instead of penalizing obstacle location. We begin by setting the final time of the problem to be the moment the bird reaches the middle of the pipes, and then we add a negative endpoint cost as depicted in this code and figure:

![reward](https://github.com/jeffxhansen/FlappyBirdOptimalControl/assets/62043475/ee87974a-2816-44c9-9406-58df62a34cdc)

Our endpoint cost function 

$$\begin{align*}
\phi (y(t_f)) = - \exp\left(\frac{-\left(y(t_f) - \frac{(P_l+P_u)}{2}\right)^2}{\sigma^2}\right)
\end{align*}$$

is similar to a rotated Gaussian distribution. We set the mean of the Gaussian to be the center location between the pipes $\frac{(P_l+P_u)}{2}$, and we set the variance $\sigma^2$ to a small number ($0.1^2$). This pushes the bird's path between the two pipes without forcing it to end exactly at a certain point; thus, the model adjusts for circumstances in which slightly different endpoints are needed without violating the laws of the model. 

Along with adding an endpoint cost, we also adjust our state equations to remove the $x(t)$ dependence. This not only avoids the `Singular Jacobian Matrix` error, but also simplifies the model for quicker convergence. Our initial state includes a specified vertical position and zero velocity. Our endpoint state incorporates a free variable for the bird's final vertical position. This allows the endpoint incentive to adapt to various game conditions. We also specify zero velocity at the endpoint in order to chain multiple sub-problems together (see the multiple pipes section for an implementation of this). Finally, we add a running cost $\frac{1}{2}u(t)^2$ to our functional to penalize large values of the control in order to simulate the constrained physics of the game.

Our new cost functional and state space are:

$$\begin{align*}
    J[u] &= \int_{t_0}^{t_f} \frac{1}{2}u(t)^2 \, dt - \exp\left(\frac{-\left(y(t_f) - \frac{(P_l+P_u)}{2}\right)^2}{\sigma^2}\right)
\end{align*}$$

$$\begin{align*}
\mathbf{x}'(t) &= \begin{bmatrix} y(t) \\ y'(t) \end{bmatrix}' =  \begin{bmatrix} y'(t) \\ u(t) - 9.8 \end{bmatrix}, \quad \mathbf{x}(0) = \begin{bmatrix} y_0 \\ 0 \end{bmatrix}, \quad \mathbf{x}(t_f) = \begin{bmatrix} \text{free} \\ 0 \end{bmatrix}
\end{align*}$$

We demonstrate the solution to this successful system in the following code and figures.

![trajectory_reward](https://github.com/jeffxhansen/FlappyBirdOptimalControl/assets/62043475/bdc7a7f1-704e-4a63-84f3-1ab70feeecaf)

![control_reward](https://github.com/jeffxhansen/FlappyBirdOptimalControl/assets/62043475/06f58c9c-b934-44b0-a071-d442dd68a099)

As seen in this figure,  the bird successfully navigates through the center of the pipes, so the reward and state-space equations converge to a valid solution. This straight line control causes the bird to smoothly glide through the middle of the pipe. 

## Guiding the Control
In the actual Flappy Bird game, the user can not control the bird with a smooth acceleration; rather, the user taps the screen at distinct times to give the bird large spikes of acceleration. To realistically model this aspect of the game, we must guide the continuous control function to mimic distinct taps experienced during game play.

To accomplish this, we change the running-cost in our functional by penalizing the shape of the control instead of large values of the control. We do this by first engineering a function $f(t)$ that resembles a sequence of taps, and then we add a penalty $W(u(t) - f(t))^2$ to the running cost. This penalizes the control when its shape does not match $f(t)$'s shape for each $t$. We use the continuous function $f(t) = \beta\max\{0, \sin(\alpha t)\}$, which only recognizes the positive portion of the sine wave. By manipulating the amplitude ($\beta$) and the wavelength ($\alpha$) of the sine component, the function results in a continuous series of sharp spikes that simulate discrete taps. 

With these additional adjustments, our final cost functional is:

$$\begin{align*}
    J[u] &= \int_{t_0}^{t_f} W \left[u(t) - \beta \max\{0, \sin(\alpha t)\} \right]^2 \, dt - \exp\left(\frac{-\left(y(t_f) - \frac{(P_l+P_u)}{2}\right)^2}{\sigma^2}\right)
\end{align*}$$

where the running cost inside the integral guides the control to look like discrete taps, and the endpoint cost acts as a reward when the bird successfully flies through the middle of the pipes.

We define our Hamiltonian as follows:

$$\begin{align*}
H = \mathbf{p} \cdot \mathbf{f} - L 
 = p_0 y'(t) + p_1 (u(t) - 9.8) - W\left[u(t) - \beta\max\{0, \sin(\alpha t)\} \right]^2
\end{align*}$$

The Hamiltonian captures the total energy of the system, taking into account the dynamics of the state variables ($y'$ and $u$) and their conjugate momenta ($p_0$ and $p_1$), as well as the Lagrangian ($L$), which represents our runtime cost.

Following Pontryagin's Maximum Principle, we solve for our costate equations as follows:

$$\begin{align*}
p_0' &= -\frac{\partial H}{\partial y} = 0 \\
p_1' &= -\frac{\partial H}{\partial y'} = -p_0
\end{align*}$$

The costate equations describe the evolution of the costate variables $p_0$ and $p_1$ over time, reflecting how changes in the Hamiltonian affect their dynamics.

At the final time $t_f$, the costates are given by:

$$\begin{align*}
p_0(t_f) &= -\frac{\partial \phi}{\partial y(t_f)} = \left(\frac{-( 2y(t_f) - (P_l+P_u))}{\sigma^2}\right)  \exp\left(\frac{-\left(y(t_f) - \frac{(P_l+P_u)}{2}\right)^2}{\sigma^2}\right) \\
p_1(t_f) &= \text{free}
\end{align*}$$

The costates at the final time provide insight into the terminal conditions required for optimal control, with $p_0(t_f)$ encoding the sensitivity of the cost to changes in the final state $y(t_f)$.

Finally, we compute the optimal control input $\tilde{u}$ by maximizing the Hamiltonian with respect to $u$:

$$\begin{align*}
\frac{ \partial H}{\partial u} = 0 = p_1 + 2W \left[ u(t) - \beta \max\{0, \sin(\alpha t)\} \right] \\
\implies \tilde{u} = \beta \max\{0, \sin(\alpha t)\} -\frac{p_1}{2W}
\end{align*}$$

We determine the optimal control input $\tilde{u}$ by balancing the cost associated with deviations from the desired trajectory $\beta\max\{0, \sin(\alpha t)\}$ against the benefits of achieving desired state transitions.

![trajectory_single](https://github.com/jeffxhansen/FlappyBirdOptimalControl/assets/62043475/505e447e-d32d-42ec-99af-83599d512da5)

![control_accel_single](https://github.com/jeffxhansen/FlappyBirdOptimalControl/assets/62043475/9f2f8b0b-b842-4b6e-b2df-540bffe1fc8b)

The Flappy Bird Trajectory plot shows that the bird successfully ends in between the two pipes, and its course appears as the path in the actual game. The acceleration plot demonstrates that when the control is zero, the bird is accelerating at $-9.8$ which is the gravitational constant. The velocity plot shows how the bird alternates between going up and down like the actual physics in the game. 

# Interpretation
Our final model successfully navigates pipes of varying heights and starting positions. We acknowledge extensive hyperparameter tuning specific to the Flappy Bird game was necessary. These parameters include frequency, amplitude, and weights. Adjusting these hyperparameters to deviate from the game scenario leads to performance degradation. For instance, doubling the wavelength causes the bird to glide towards the pipes on a smooth trajectory instead of a jagged path like in the actual game. Similarly, increasing the amplitude guides the bird to the pipe center without considering collisions, and the bird often will intersect the pipe obstacles (see the Appendix for an example of these hyperparameter situations).

## Multiple Pipes

To simulate the game by guiding the bird through a sequence of random pipes, we break the multiple-pipe problem into sub-problems (start-pipe1, pipe1-pipe2, etc.). We then use solve\_bvp on each subsection where the current section's initial conditions are set to the final state of the previous solution. Finally, we concatenate all of the data from these solutions into one large plot, and add the pipes in the appropriate locations. 

![trajectory_6371210_3_0 9](https://github.com/jeffxhansen/FlappyBirdOptimalControl/assets/62043475/dd2162da-d3e1-46f8-9678-7300a9a0620d)

![control__6371210_3_0 9](https://github.com/jeffxhansen/FlappyBirdOptimalControl/assets/62043475/676f6b24-1e85-4d05-9835-ab53b44302c1)

![velocity_6371210_3_0 9](https://github.com/jeffxhansen/FlappyBirdOptimalControl/assets/62043475/b59f2fd3-a9fa-434b-8115-fa3ce8d2b90b)

As seen in this figure, our model produces a valid solution that both avoids the pipes, and also successfully models the flow of the game. On top of this, our optimal control successfully navigates any choice of pipe locations. 

While this approach produces a valid solution which accurately navigates the pipes, this may not be the optimal solution overall. This is reflected in the control, which exhibits jagged jumps as the algorithm transitions between solutions for individual pipe pairs. These jumps occur because we enforce zero velocity at the start and end of each sub-problem, resulting in unnatural movements for the bird.

# Conclusion
This project seeks to find the most efficient path for a Flappy Bird to navigate through a series of pipe obstacles. Our first approach focuses on obstacle avoidance. In performing this, we begin with a cost functional that incorporates the obstacles in our functional and penalizes the area of the obstacles. In developing these early stages of our model, we encounter several hurdles in solving our functional, including Singular Jacobian matrices. Upon restructuring our state space to solely include the vertical velocity and acceleration, we identify the constant horizontal velocity as the root of these errors. Leveraging Pontryagin's Maximum Principle as well as addressing these initial "obstacles", we successfully create an algorithm to beat the Flappy Bird game. Rather than penalizing obstacle areas, we utilize a Gaussian distribution to incentivize the bird to pass through the pipes. In order to simulate the discrete tapping motion of the bird through the obstacles, we also incorporate a guide to the control. This combination of tapping motion and endpoint reward results in a successful model. Future model iterations could include structuring a control function that would be able to vary in frequency. This would allow the model to dynamically adjust the timing of taps, leading to more natural and efficient obstacle navigation. 

Our current approach simulates the multi-pipe Flappy Bird scenario by sequentially solving for the optimal path between individual pipes. These solutions are then combined to form a complete trajectory.  While this method successfully navigates the game and avoids collisions, future research could explore finding the optimal path through a sequence of pipes in one solution, potentially leading to a more efficient approach compared to the current piece-wise version.

# References 

[Ang] Alex Angry. The flappy bird phenomenon: A story about dependency
in the app store.

[Gan22] R. Gan. Flappy bird: Optimization of deep q-network by genetic algo-
rithm. In 2022 IEEE International Conference on Artificial Intelligence
and Computer Applications (ICAICA), pages 703–707, Dalian, China,
2022

# Appendix

### Hyperparameter example - alpha is too high

![trajectory_5_10_0 9](https://github.com/jeffxhansen/FlappyBirdOptimalControl/assets/62043475/8cb9513d-7db1-4967-ab33-d28c09669624)

![control__5_10_0 9](https://github.com/jeffxhansen/FlappyBirdOptimalControl/assets/62043475/6d576d32-db11-4f3c-a37d-2a98e8051c72)

![velocity_5_10_0 9](https://github.com/jeffxhansen/FlappyBirdOptimalControl/assets/62043475/2187cbee-23b2-41d1-90ad-72ce46763283)

### Hyperparameter example - beta is too high

![trajectory_2571210_3_10](https://github.com/jeffxhansen/FlappyBirdOptimalControl/assets/62043475/901dbed4-4569-42cf-8ed3-d14d76489933)

![control__2571210_3_10](https://github.com/jeffxhansen/FlappyBirdOptimalControl/assets/62043475/1d380df9-da15-4598-a342-8a21aa730cd5)

![velocity_2571210_3_10](https://github.com/jeffxhansen/FlappyBirdOptimalControl/assets/62043475/e591a852-8e37-4f46-889a-da6b7b076142)











