

### System Dynamics with redundant state representation

If the chosen state representation is redundant ( $n$ of variables $>$ $dof$ ) the coordinates will be related by some constraints.

The constraint can be expressed in the Pfaffian form:
$$A(q) \dot q = 0$$
<!-- $$A(q) =\begin{pmatrix}
        1 &0 &-r \sin(\phi_1 + \phi_2) &-r \sin(\phi_1 + \phi_2) & \cos(\phi_1 + \phi_2)\\
        0 &1  & r \cos(\phi_1 + \phi_2) & r \cos(\phi_1 + \phi_2) &\sin(\phi_1 + \phi_2)
    \end{pmatrix} \in \mathbb{R}^d$$ -->
And the equations then become:
$$\begin{cases}
    M(q)\ddot q + C(q, \dot q) \dot q + G(q) + A^{T}(q)\lambda = b_m \tau \\
    A(q) \dot q = 0
\end{cases}
$$
where $\bm \tau = (0,\,0,\,0, 0,\tau_{r},\tau_{l},0,0)^T$ and the lagrangian multiplier $\lambda$ represents the vincular forces that act on the system.
Using the pseudo-velocity method the equations can be rewritten starting from the consideration that the velocities $\dot q$ must belong to the null space of $A(q)$. We thus define $S(q)$, a basis for the Null Space of $A(q)$. The following relationship holds:
$$ \dot q = S(q) b_m \nu $$
where $\bm \nu \in \mathbb{R}^{n-d}$ is the pseudo-velocity vector useful to represent .

Deriving the equation:
$$ \ddot q = S(q) \dot{b_m \nu} + \dot{S}(q) b_m \nu$$ 
Substituting in the dynamics equations and pre multipyling for $S(q)^T$ the equations can be rewritten as:
$$  \dot q = S(q) b_m \nu$$
$$  \dot{\bm \nu} = - \overline{M}(q)^{-1} \overline{h}(q, b_m \nu) + \overline{M}(q)^{-1} S(q)^T b_m \tau 
$$
where:
$$\overline{M}(q) = S^T M S $$
$$\overline{h}(q, \bm \nu) = S^T(C S \bm \nu + G + M \dot{S} \bm \nu)$$ 