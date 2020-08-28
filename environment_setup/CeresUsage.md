# Use Ceres

Ceres is  an open source C++ library for modeling and solving large, complicated optimization problems.

When using Ceres, we only need to define the optimization problem following certain steps, then hand over the problem to the solver for computation.

The general form of least squares problems in Ceres is as following:
$$
\min_x \frac{1}{2} \sum_i\rho_i(\Vert f_i(x_{i_1},\cdots, x_{i_n})\Vert^2) \\
s.t.~~l_j\le x_j\le u_j
$$
$x_1,\cdots,x_n$ is the **optimization variables** (also called **Parameter blocks**)

$f_i$ is the **Cost function** (also called **Residual blocks**)

$l_j$ and $u_j$ are the **upper limit** and the **lower limit** of the $j$th optimization variable

$\rho$ is the **Kernel function**

Assuming the simplest situation, we make $j_i=-\infty,u_j=\infty$ (we do not limit the upper and lower  boundary of the optimization variables). So the square items $\Vert f_i(x_{i_1},\cdots, x_{i_n})\Vert^2$ go through the kernel function $\rho$, then we sum up the results and divide it by 2 to get the final output.

If we make the kernel function $\rho$ an identity function $f(x)=x$, the final output is just the square sum of those items $\Vert f_i(x_{i_1},\cdots, x_{i_n})\Vert$, which results in an unconstrained least squares problem.

To use Ceres for solution, the following steps need to be done:

1. **Define every Parameter blocks.** Parameter blocks are usually normal vectors. In SLAM, we can also define them with some special structure such as quaternions and Lie algebra. If it is a vector, we need to assign a double array for the storage of the variable value.
2. **Define the computing method for the Residual blocks**. The residual block usually combine several parameter blocks together, performing some user-custom calculation, and return the residual value. Then Ceres will compute their square sum as the final output value (return value of target function).
3. **Define the Jacobian calculation method for the residual block.** In Ceres, we can use the "automatic derivation" feature, or we can manually specify the calculation process of Jacobian. If automatic derivation is used, the residual block needs to be written in a specified format.
4. **Add all the parameter blocks and residual blocks to the Problem object defined by Ceres, then call the Solve function for solution.** Before calling the "Solve" function, we can input some configuration information such as number of iterations and termination condition. (Or just use the default configuration).