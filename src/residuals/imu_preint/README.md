Preintegration of IMU measurements.

Inertial Measurement Unit (IMU) preintegration is a common technique used to summarize multiple IMU measurements into a single constraint used in preintegration. Our implementation is based on "local coordination integration", similar to the one [used in gtsam by default](https://github.com/borglab/gtsam/blob/4.2.0/doc/ImuFactor.pdf).
# Example
```
use factrs::residuals::{ImuPreintegrator, Accel, Gravity, Gyro, ImuCovariance};
use factrs::variables::{SE3, VectorVar3, ImuBias};
use factrs::assign_symbols;

assign_symbols!(X: SE3; V: VectorVar3; B: ImuBias);

let mut preint =
    ImuPreintegrator::new(ImuCovariance::default(), ImuBias::zeros(), Gravity::up());

let accel = Accel::new(0.1, 0.2, 9.81);
let gyro = Gyro::new(0.1, 0.2, 0.3);
let dt = 0.01;
// Integrate measurements for 100 steps
for _ in 0..100 {
   preint.integrate(&gyro, &accel, dt);
}

// Build the factor
let factor = preint.build(X(0), V(0), B(0), X(1), V(1), B(1));
```

# Theory
<div class="warning">
Note, our convention throughout the library is to always put rotations first
in any list, vector, matrix ordering, etc.
</div>

Our state will consist of five components, specifically the rotation from body to the world frame $R$, body velocity in the world frame $v$, body position in the world frame $p$,
angular velocity bias $b^\omega$, and linear acceleration bias $b^a$. We'll
denote the vector $X$ as these components together in the order listed
previously.

An IMU measures body angular velocity $\omega$ and body linear acceleration
$a$. This results in the continuous time differential equation

$$\begin{aligned}
\dot{R} &= R (\omega - b^\omega) ^\wedge \\\\
\dot{v} &= g + R (a - b^a) \\\\
\dot{p} &= v \\\\
\end{aligned}$$

with initial conditions given by $R_0, v_0, p_0, b^{\omega}_0, b^a_0$, or
$X_0$.

The goal here is to find an integration that is *independent* of
these initial conditions. In other words, we want a solution $\Delta X$ that
allows us to perform $X_1 = X_0 \cdot \Delta X$ for some custom operation
$\cdot$. This results in an integration we can do beforehand, aka a
"preintegration".

## Handling Gravity

The first thing to note is there is a number of constants in the system that
can be removed to simplify things. For example, gravity in the velocity
differential equation,
$$
\dot{v} \triangleq \dot{v}^g + \dot{v}^a = g + R(a - b^a)
$$
$\dot{v}^g$ is a constant and can be integrated afterwards. Similarly, for
the position component, we have
$$
\dot{p} \triangleq \dot{p}^0 + \dot{p}^g + \dot{p}^a = v_0 + v^g + v^a
$$
where $\dot{p}_0$ and $\dot{p}^g$ are constants and can be integrated
afterwards. This brings our differential equations to
$$ \begin{aligned}
\dot{R} &= R (\omega - b^\omega) ^\wedge \\\\
\dot{v}^a &= R(a - b^a) \\\\
\dot{p}^a &= v^a \\\\
\end{aligned} $$

## Local Coordinates

Additionally, rather than integrate in the global frame, we can integrate in
a local frame that is aligned with the initial rotation $R_0$. We recommend
reviewing (CITE) for more information on this. This results in
the equations 
$$ \begin{aligned}
\dot{\theta} &= H(\theta)^{-1} (\omega - b^\omega) \\\\
\dot{v}^a &= \exp(\theta^\wedge) (a - b^a) \\\\
\dot{p}^a &= v^a \\\\
\end{aligned} $$
where $\theta$ is the local rotation in the Lie algebra and $H(\theta)$ is
the Jacobian of the exponential map at $\theta$.

## Discretization

Moving this into discrete time using a simple Euler scheme results in,
$$ \begin{aligned}
\theta_{k+1} &= \theta_k + H(\theta_k)^{-1} (\omega_k - b^\omega_0) \Delta t \\\\
v^a_{k+1} &= v^a_k + \exp(\theta_k^\wedge) (a_k - b^a_0) \Delta t \\\\
p^a_{k+1} &= p^a_k + v^a_k \Delta t + \exp(\theta_k^\wedge) (a_k - b^a_0) \Delta t^2 / 2 \\\\ 
\end{aligned} $$
where we have assumed a constant bias over our timestep. We are now
independent of the initial conditions (except for bias which we'll cover
shortly). Additionally, the bias will have the simple discretized form,
$$ \begin{aligned}
b^\omega_{k+1} &= b^\omega_k \\\\
b^a_{k+1} &= b^a_k \\\\
\end{aligned} $$

## Covariance Propagation

TODO: Include some reference to Kalibur github on noise model units

We also propagate the covariance of the entire system. There is six noise
terms that we introduce to account for the various errors of the system,
each of which is a 3-vector with a 3x3 covariance matrix. These are
- $\epsilon_{\omega}$: the noise of the angular velocity measurement
- $\epsilon_{a}$: the noise of the linear acceleration measurement
- $\epsilon_{\omega^b}$: the noise of the angular velocity bias
- $\epsilon_{a^b}$: the noise of the linear acceleration bias
- $\epsilon_{int}$: the noise of the integration error
- $\epsilon_{init}$: the noise of the bias initialization

These will be stacked into a 18-vector with a block-diagonal 18x18 covariance matrix $Q$. We introduce the noise as follows,
$$ \begin{aligned}
\theta_{k+1} &= \theta_k + H(\theta_k)^{-1} (\omega_k + \epsilon_\omega - b^\omega_0 - \epsilon_{init}) \Delta t \\\\ 
v^a_{k+1} &= v^a_k + \exp(\theta_k^\wedge) (a_k + \epsilon_a - b^a_0 - \epsilon_{init}) \Delta t \\\\
p^a_{k+1} &= p^a_k + v^a_k \Delta t + \exp(\theta_k^\wedge) (a_k + \epsilon_a - b^a_0 - \epsilon_{init}) \Delta t^2 / 2 + \epsilon_{int} \\\\
b^\omega_{k+1} &= b^\omega_k + \epsilon_{\omega^b} \\\\
b^a_{k+1} &= b^a_k + \epsilon_{a^b} \\\\
\end{aligned} $$

We can then propagate a covariance matrix for our preintegrated values using the Jacobian of the system with respect to the noise terms. 
$$ \begin{aligned}
A_k &\triangleq \frac{\partial f}{\partial X} = \begin{bmatrix} 
I - \frac{\Delta t}{2} \tilde{\omega}_k & 0 & 0 & -H(\theta_k)^{-1}\Delta t & 0 \\\\
-R_k \tilde{a}^\wedge H(\theta) \Delta t & I & 0 & 0 & -R_k \Delta t \\\\ 
-R_k \tilde{a}^\wedge H(\theta) \Delta t^2 / 2 & I \Delta t & I & 0 & -R_k \Delta t^2 /2 \\\\ 
0 & 0 & 0 & I & 0 \\\\
0 & 0 & 0 & 0 & I \\\\
\end{bmatrix} \\\\
B_k &\triangleq \frac{\partial f}{\partial \epsilon} = \begin{bmatrix}
H(\theta_k)^{-1} \Delta t & 0 & 0 & 0 & 0 & -H(\theta)^{-1}\Delta t \\\\
0 & R_k \Delta t & 0 & 0 & 0 & -R_k \Delta t \\\\
0 & R_k \Delta t^2 / 2 & 0 & 0 & I & -R_k \Delta t^2 / 2 \\\\
0 & 0 & I & 0 & 0 & 0 \\\\
0 & 0 & 0 & I & 0 & 0 \\\\
\end{bmatrix} \\\\
\end{aligned} $$

where we have used 
$$ \begin{aligned}
\tilde{\omega} &\triangleq \omega - b^\omega_0 \\\\
\tilde{a} &\triangleq a - b^a_0 \\\\
R_k &\triangleq \exp(\theta_k^\wedge) \\\\
\frac{\partial H(\theta_k)^{-1} \tilde{\omega}}{\partial \theta} &\approx \frac{-1}{2}\tilde{\omega}^\wedge \\\\
\frac{\partial R_k \tilde{a}}{\partial \theta} &\approx R_k \tilde{a}^\wedge H(\theta_k) \\\\
\end{aligned} $$

Put all together, we have the covariance propagation equation
$$
\Sigma_{k+1} = A_k \Sigma_k A_k^T + B_k Q B_k^T
$$

## First-order Bias Updates

As mentioned previously, there still exists a dependence on the initial bias parameters. We approximate the impact of bias changes using a first order approximation. This is done using the Jacobian of the system with respect to the bias terms. We can construct these Jacobians iteratively as follows, letting $x = [\theta, v, p]^\top$, and linearizing about an initial bias $\tilde{b}^\omega_0$

$$ \begin{aligned}
H_{\omega}^{x_{k+1}} &\triangleq \frac{\partial \theta_{k+1}}{\partial b^\omega_0} = \frac{\partial \theta_{k+1}}{\partial \theta_k} \frac{\partial \theta_k}{\partial b^\omega_0} + \frac{\partial \theta_{k+1}}{\partial \omega_k} = \frac{\partial \theta_{k+1}}{\partial \theta_k} H_{\omega}^{x_{k}} + \frac{\partial \theta_{k+1}}{\partial \omega_k} 
\end{aligned} $$

Where the acceleration bias update is similar. Fortunately, the Jacobians required here are already computed in the $A_k$ matrix defined previously and we can simply extract them as needed.

## Custom Operator

Returning to our custom operator that we mentioned earlier to accomplish $X_1 = X_0 \cdot \Delta X$, we can define what it will be, taking into account the first-order updates, local coordinates, and the gravity terms we removed earlier.

First, we update our preintegration equations to include any changes due to bias updates,
$$ \begin{aligned}
\tilde{\theta} &= \theta + H_{\omega}^{\theta_{k+1}} (b^\omega_0 - \tilde{b}^\omega_0) + H_{a}^{\theta_{k+1}} (b^a_0 - \tilde{b}^a_0) \\\\
\tilde{v}^a &= v^a + H_{\omega}^{v_{k+1}} (b^\omega_0 - \tilde{b}^\omega_0) + H_{a}^{v_{k+1}} (b^a_0 - \tilde{b}^a_0) \\\\
\tilde{p}^a &= p^a + H_{\omega}^{p_{k+1}} (b^\omega_0 - \tilde{b}^\omega_0) + H_{a}^{p_{k+1}} (b^a_0 - \tilde{b}^a_0) \\\\
\end{aligned} $$

Then, we can define our custom operator as follows,

$$ \begin{aligned}
R_1 &= R_0 \exp(\tilde{\theta}^\wedge) \\\\
v_1 &= R_0 \tilde{v}^a + v_0 + g \Delta t \\\\
p_1 &= R_0 \tilde{p}^a + p_0 + v_0 \Delta t + g \Delta t^2 / 2 \\\\
b_1^\omega &= b_0^\omega \\\\
b_1^a &= b_0^a \\\\
\end{aligned} $$

The covariance we propagated earlier will exist in the local frame (the right hand side) of these measurements. We thus can define our residual using a "right ominus" operator, 
$$ \begin{aligned}
X_1 = X_0 \cdot (\Delta X \cdot \epsilon) \implies r = \epsilon = \log( X_1^{-1}  (X_0 \cdot \Delta X) )
\end{aligned} $$


## Implementation Details

[ImuPreintegrator] handles the integration of $\Delta X$ and the covariance propagation. It'll be the main entrypoint for doing Imu preintegration. It can produce a factor that contains the proper residual and covariance for use in a factor graph.

[ImuCovariance] holds the six different covariances utilized in the covariance propagation. It also contains a handful of helper functions for simplify setting covariance values.

Internally (if you care to peruse the codebase), there is a number of structures that follow naturally from the theory. For example,
- `ImuDelta` represents $\Delta X$ and contains $g, \Delta t, \theta, v^a, p^a, \tilde{b}^\omega, \tilde{b}^a$.
- `ImuState` represents $X$ and contains $R, v, p, b^\omega, b^a$.
- `ImuBias` represents the bias terms and contains $b^\omega, b^a$.
- `ImuPreintegrationResidual` is used to calculate the final residual.