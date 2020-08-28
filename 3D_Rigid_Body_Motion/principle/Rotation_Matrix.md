# Rotation Matrix

## Point, Vector and Coordinate

In linear algebra, a certain point in 3D space can be described with $\mathbb{R}^3$. 

Assuming we have found a basis of the linear space $(\mathbf{e_1}, \mathbf{e_2}, \mathbf{e_3})$. Then for arbitrary vector $\mathbf{a}$, we can find a coordinate under this set of basis:
$$
\mathbf{a}=[\mathbf{e_1}, \mathbf{e_2}, \mathbf{e_3}]
\left [
\begin{matrix}
   a_1 \\
   a_2 \\
   a_3
\end{matrix} 
\right ]
=a_1\mathbf{e_1}+a_2\mathbf{e_2}+a_3\mathbf{e_3}
$$
For $\mathbf{a},\mathbf{b}\in \mathbb{R}^3$, their inner product can be written as:
$$
\mathbf{a}\cdot \mathbf{b}=\mathbf{a}^T\mathbf{b}=\sum^3_{i=1}a_ib_i=|\mathbf{a}||\mathbf{b}|cos\langle \mathbf{a},\mathbf{b}\rangle
$$
$\langle \mathbf{a},\mathbf{b}\rangle$ refers to the angle between vector $\mathbf{a},\mathbf{b}$.

Their outer product is:
$$
\mathbf{a}\times \mathbf{b}=
\left\Vert
\begin{matrix}
   \mathbf{e_1} & \mathbf{e_2} & \mathbf{e_3} \\
   a_1 & a_2 & a_3 \\
   b_1 & b_2 & b_3 
\end{matrix} 
\right\Vert
=
\left [
\begin{matrix}
   a_2b_3-a_3b_2 \\
   a_3b1-a_1b_3 \\
   a_1b_2-a_2b_1
\end{matrix} 
\right ]
=
\left [
\begin{matrix}
   0 & -a_3 & a_2 \\
   a_3 & 0 & -a_1 \\
   -a_2 & a_1 & 0
\end{matrix} 
\right ]
\mathbf{b}=\mathbf{a}^\wedge\mathbf{b}
$$
The result of outer product is a vector that its direction is perpendicular to  $\mathbf{a},\mathbf{b}$ and its magnitude is $|\mathbf{a}||\mathbf{b}|sin\langle \mathbf{a},\mathbf{b}\rangle$

For the outer product computation, we can introduce a symbol $\wedge$, which turns $\mathbf{a}$ into a **Skew-symmetric Matrix**. With this we can make the outer product $\mathbf{a}\times\mathbf{b}$ a linear computation between a matrix and a vector $\mathbf{a}^\wedge\mathbf{b}$.
$$
\mathbf{a}^\wedge=
\left [
\begin{matrix}
   0 & -a_3 & a_2 \\
   a_3 & 0 & -a_1 \\
   -a_2 & a_1 & 0
\end{matrix} 
\right ]
$$

## Euclidean Transformation between Coordinates

The transformation between two coordinates is a combination of a rotation and a translation, which is called *rigid body motion*. The length and angle of a vector in different coordinates will remain the same during a *rigid body motion*. We could say that there is a difference of a **Euclidean Transformation** between the initial coordinate and the resulted coordinate after the *rigid body motion*.

We will consider rotation first. We assume some unit orthogonal basis $(\mathbf{e_1}, \mathbf{e_2}, \mathbf{e_3})$ turns into  $(\mathbf{e_1'}, \mathbf{e_2'}, \mathbf{e_3'})$ after a rotation. Then for a vector $\mathbf{a}$, its coordinate is $[a_1,a_2,a_3]^T$ before the rotation and $[a_1',a_2',a_3']^T$ after the rotation. Note that after the rotation $\mathbf{a}$ itself does not change, so we get the following:
$$
\left [
\begin{matrix}
   \mathbf{e_1} & \mathbf{e_2} & \mathbf{e_3}
\end{matrix} 
\right ]
\left [
\begin{matrix}
   a_1 \\
   a_2 \\
   a_3
\end{matrix} 
\right ]
=
\left [
\begin{matrix}
   \mathbf{e_1}' & \mathbf{e_2}' & \mathbf{e_3}'
\end{matrix} 
\right ]
\left [
\begin{matrix}
   a_1' \\
   a_2' \\
   a_3'
\end{matrix} 
\right ]
$$
Left-multiply $\left [
\begin{matrix}
   \mathbf{e_1}^T \\ \mathbf{e_2}^T \\ \mathbf{e_3}^T
\end{matrix} 
\right ]$ on the left and right sides of the above equation, we get:
$$
\left [
\begin{matrix}
   a_1 \\
   a_2 \\
   a_3
\end{matrix} 
\right ]=
\left [
\begin{matrix}
   \mathbf{e_1^Te_1'} & \mathbf{e_1^Te_2'} & \mathbf{e_1^Te_3'} \\
   \mathbf{e_2^Te_1'} & \mathbf{e_2^Te_2'} & \mathbf{e_2^Te_3'} \\
   \mathbf{e_3^Te_1'} & \mathbf{e_3^Te_2'} & \mathbf{e_3^Te_3'}
\end{matrix} 
\right ]
\left [
\begin{matrix}
   a_1' \\
   a_2' \\
   a_3'
\end{matrix} 
\right ]
=R\mathbf{a'}
$$
We take the matrix in the middle out and define it as $R$. It describe the rotation and we call it **Rotation Matrix** (AKA **Direction Cosine Matrix**).Note that the rotation matrix $R$ is an orthogonal matrix, that is, $R^T=R^{-1}$. And $R^{-1}$ describe an inverse rotation. 
$$
\mathbf{a}=R\mathbf{a'}=R^T\mathbf{a'}
$$
Taking translation $\mathbf{t_{12}}$ (from coordinate 1 origin to coordinate 2 origin) into consideration, assuming $R_{12}$ as the transformation from coordinate 2 to coordinate 1, $\mathbf{a_1}$ is the vector described in coordinate 1 and $\mathbf{a_2}$ is the vector described in coordinate 2, we get:
$$
\mathbf{a_1}=R_{12}\mathbf{a_2}+\mathbf{t_{12}}
$$

## Transform Matrix & Homogeneous Coordinates

Assuming there is a vector $\mathbf{a}$ and we have performed 2 transformations $R_1,\mathbf{t}_1$ and $R_2,\mathbf{t}_2$, the resulted vector is $\mathbf{c}$
$$
\mathbf{c}=R_2(R_1\mathbf{a}+\mathbf{t}_1)+\mathbf{t}_2
$$
If we have multiple transformation to perform, the formula will be too complex, therefore, we introduce homogeneous coordinates.
$$
\left [
\begin{matrix}
   \mathbf{a}' \\
   1
\end{matrix} 
\right ]
=
\left [
\begin{matrix}
	R & \mathbf{t} \\
	0^T & 1
\end{matrix} 
\right ]
\left [
\begin{matrix}
   \mathbf{a} \\
   1
\end{matrix} 
\right ]
=T
\left [
\begin{matrix}
   \mathbf{a} \\
   1
\end{matrix} 
\right ]
$$
Here we call matrix $T$ the **Transform Matrix**. $T^{-1}$ describes an inverse transform and has the following form:
$$
T^{-1}=
\left [
\begin{matrix}
   R^T & -R^T\mathbf{t} \\
   0^T & 1
\end{matrix} 
\right ]
$$
