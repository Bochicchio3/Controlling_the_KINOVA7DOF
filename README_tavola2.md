# Robots Control Project: Tavola 2 e 3

## Table of Content

- [Robots Control Project: Tavola 2 e 3](#robots-control-project-tavola-2-e-3)
  - [Table of Content](#table-of-content)
  - [Introduction](#introduction)
  - [Inverse spherical anolonome pendulum](#inverse-spherical-anolonome-pendulum)
    - [Non Minimal state representation](#non-minimal-state-representation)
    - [No slip condition and Non holonomic constraints](#no-slip-condition-and-non-holonomic-constraints)
    - [Minimal State representation](#minimal-state-representation)
    - [Parameters](#parameters)
    - [Direct Kinematics](#direct-kinematics)
    - [Kinetic Energy](#kinetic-energy)
    - [Potential Energy and Lagrangian](#potential-energy-and-lagrangian)
    - [Dynamics model and Equations of Motion](#dynamics-model-and-equations-of-motion)
    - [Free motion simulation](#free-motion-simulation)
    - [Normal Form](#normal-form)
    - [Controllability and Accessibility](#controllability-and-accessibility)
    - [Osservability](#osservability)
    - [Feedback Linearization in MIMO systems](#feedback-linearization-in-mimo-systems)


<p align="center">
  <!-- <img align="Center" src="images/2020-06-04-01-14-39.png" alt="drawing" class="center" width="250" /> -->
  <img align="Center" src="images/2020-06-04-12-17-37.png" alt="drawing" class="center" width="500" />
    <figcaption  align="Center"> Fig.1 - Anolonome Spherical Inverted pendulum.</figcaption>

</p>

## Introduction

The problem of balancing an inverted pendulum has attracted the attention of control researchers in recent decades. There exist a wide variety of inverted-pendulum-type systems, such as the pendubot, the acrobot, the pendulum on a cart, the inertial wheel pendulum and the Furuta pendulum. All these systems are under-actuated, that is, they are systems with fewer actuators than degrees of freedom.

This work deals with the modelling and control of a spherical inverted pendulum (SIP), which is another member of the family of inverted pendulum systems with degree of under-actuation of two.

The system consists of a rigid rod coupled in its base to an under-actuated universal joint in such a way that the extreme of the rod moves over a spherical surface with its centre at the base of the rod (see Figure 1). As such, through the motion of the base of the pendulum on the horizontal plane it is possible to balance the extreme of the rod in its upright position.


## Inverse spherical anolonome pendulum

The Spherical anolonome inverted pendulum is a underactuated system with 4 dof. 
- Two rotations are needed to describe the pose of the pendulum on a spherical surface (the third angle is not necessary as it is an symmetry axes for the pendulum) 
- Two coordinates are needed for the bases of the pendulum (eg. the angles of the two wheels).

There are multiple ways to parametrize the state, but the main choiche is wether to choose a minimal or a non minimal state representation.

### Non Minimal state representation

In this case the state would be:

<!-- $$q(t)=\{\theta_r,\theta_l,\psi,\phi,x,y,\theta\}$$ -->
![](2020-06-16-00-50-54.png)
In this case it would be easier to compute the direct kinematic as many quantities would be related directly to $x$ and $y$.

If a system is parametrized with m $m$ coordinates but only has $n(n<m)$ dof; then, there should exist $p=m−n$ constraints which allow the reduction of the order of the system from m to n; in others words, if we choose n independent joint variables there must exist p dependent joint variables.

![](2020-06-16-00-51-07.png)


In this case the constraints is a non slip non differentiable constraint, which will be modelled in the next section.
### No slip condition and Non holonomic constraints

 <!-- The constraints are rolling without slipping for the two wheels. This results in $x,y$ and $\theta$ being excluded from the state and only related to $\theta_r, \theta_l$ with the following relationship: -->
![](2020-06-16-01-04-31.png)
<!-- $$\theta =\frac{r (\theta_r(t)-   \theta_l(t)}{l_{a}}$$

$$\dot x=\frac{r \cos (\theta ) (\dot     {\theta} r(t)-\dot{\theta}l(t))}{2} $$

$$\dot y=\frac{r \sin (\theta ) (\dot     {\theta} r(t)-\dot{\theta}l(t))}{2} $$ -->
![](2020-06-16-00-52-56.png)
### Minimal State representation

Explicitly modelling the constraints allows a minimal state representation. In this case the configuration would be:


<!-- 
$$\theta_r(t):\text{left wheel}                                 $$
$$\theta_{l}(t): \text{right wheel}                             $$
$$\psi (t): \text{X rotation in the XYZ parametrization}        $$
$$\phi (t): \text{Y rotation in the XYZ parametrization}        $$ -->
![](2020-06-16-00-53-11.png)
So that the resulting minimal representation is:

![](2020-06-16-00-53-19.png)
<!-- 
$$q(t)=\left\{\theta_r(t),\theta_l(t),\psi (t),\phi (t)\right\}$$


$$\dot q(t)=\{\dot{\theta_r}(t),\dot{\theta}_{l}(t),\dot{\psi}(t),\dot{\phi}(t)\}$$ -->




### Parameters

Just for reference, the used parameters:
<!-- 
$$G\to 9.81,m_p\to 1      ,m_a\to 1      ,m_r\to 0.5    ,l_p\to 0.7    ,l_a\to 1      ,r\to 0.3            ,I_a\to 0.8$$ -->

![](2020-06-16-00-53-35.png)


### Direct Kinematics

The direct kinematics problem consists in finding expressions for the pose (i.e., position and orientation) variables of an element of interest of the system, described by the vector x in terms of the actuated joint variables; this can be expressed as
![](2020-06-16-00-54-00.png)
<!-- $$x=f(q,β(q))$$
	

In addition, by taking the time derivative of the previous equation we get the so-called differential kinematics equation

$$\dot x=J(ρ)\dot q$$


where 
$$J(ρ)=∂f(q,β(q))∂q$$


On the other hand, the inverse kinematics problem consists in establishing the value of the actuated joint coordinates that correspond to a given pose of the element of interest, and can be expressed as

$$q=f^{-1}(x)$$ -->
	

The inverse kinematics problem can be solved in general form by geometric or analytic approaches.






The following quantities need to be defined to compute the terms necessary for the lagrangian formulation of the equations of motion:

![](2020-06-16-00-54-33.png)
<!-- - $O$ center of the wheels axis $\{$ $x,y,0\}$ 
- $OP$ center of mass of the pendulum
- $v_{rl}$: velocity of the center of mass of the left wheel
- $v_{rr}$: velocity of the center of mass of the right wheel
- $v_{p}$: velocity of the center of mass of the pendulum
- $v_a$: velocity of the wheel axis
- $\omega P_{g}$: angular velocity of pendulum in fixed frame
- $\omega P_{p}$: angular velocity of pendulum in pendulum frame
- $x$: velocity of the center of mass of the left wheel
- $y$: velocity of the center of mass of the left wheel
- $\theta$: velocity of the center of mass of the left wheel
  -->
The velocity of the wheels are easy to obtain from state coordinates through the no slip condition:
![](2020-06-16-00-55-00.png)
<!-- $$ v_a=\{ \dot x, \dot y, 0\}^T$$

 $$v_{rl}=r  \dot{\theta}_l(t);$$

$$v_{rr} =r \dot{\theta}_r(t) ;$$
 


$$R_p=R_x(\psi (t)).R_y(\phi (t)).R_z(\eta (t));$$


$$\omega P_g=\dot \eta (t) R_x(\psi (t)) R_y(\phi (t)) \times \{0,0,1\}+\dot \phi (t) R_x(\psi (t))\times \{0,1,0\}+\{1,0,0\} \dot \psi (t);$$ -->

<!-- $$\omega P_s=\dot \psi (t) (R_y}(\phi (t)).R_z(\eta (t)))^T {1,0,0\}+\dot \phi (t) Rz}(\eta (t))^T.\{0,1,0\}+\{0,0,1\} \eta '(t)$$ -->

<!-- $$ v_p = v_a + \omega P_g \times AP$$ -->

### Kinetic Energy

<!-- $$E_{tot}=E_{p2}+E_{a1}+E_{a2}+E_{wl1}+E_{wr1}+E_{wl2}+E_{wr2} $$
$$E_{p1}=\frac{m_p  v_P^T v_P}{2}$$
$$E_{a2}=\frac{I_{a} \theta^2  }{2} $$
$$E_{p2}=\frac{1}{2}   \omega P_s J_{pg} \omega P_s$$
$$E_{a1}=\frac {m_{p}  v_{a}   v_{a}}{2}$$
$$E_{wl2}=\frac{w_{rr}J_{r}    w_{rr}}{2}$$
$$E_{wr2}=\frac{w_{rl} J_{r}   w_{rl}}{2}$$
$$E_{wl1}=\frac{m_{r}  v_{rr}^2}{2}; $$
$$E_{wr1}=\frac{m_{r}  v_{rl}^2}{2}$$ -->
![](2020-06-16-00-55-19.png)
### Potential Energy and Lagrangian
![](2020-06-16-00-55-28.png)
<!-- $$ mpgz_p $$

$$ L=E_{tot}-U $$ -->

### Dynamics model and Equations of Motion

The Equations of motions can be now explicitly solved for some initial conditions, and with the help of a 3D animation It is clear if the solution is physically reasonable.
<!-- $$ \frac{\partial }{\partial t}\frac{\partial L}{\partial \text{qd}(t)}-\frac{\partial L}{\partial q(t)}=\tau_q$$

$$\tau_q=\{0, 0,0,0,\tau_r, \tau_l,0 ,0\}^T$$  -->
![](./images/pendulum.gif)

### Free motion simulation

The best way to understand if the equation of motion found are correct is to simulate with an ODE solver the equations of motion from different starting conditions and verify if the system behaves in a physically plausible way. 
In this case I tried to simulate the system starting near the upright position, and the result is the following:

### Normal Form


### Controllability and Accessibility

The controllability of the non linear system can be analyzed starting from the Chow's theorem, that states: A system is small time locally accessile in $x_0 \in \mathbb{R} ^n$ if, given the system 

<p align="center"><img align="Center" src="images/2020-06-04-15-48-59.png" alt="drawing" class="center" width="300"/></p>

and the distributions: 

<p align="center"><img align="Center" src="images/2020-06-04-11-51-21.png" alt="drawing" class="center" width="190"/></p>

the smallest $\Delta$-invariant containing+ $\Delta_0$, which is $< \Delta | \Delta_0>$, has dimensions $n$ in $x_0$.
The dimensions of  $< \Delta | \Delta_0>$ can be computed using the filtration procedure, for which the following quantity must be computed iteratively:

<p align="center"><img align="Center" src="images/2020-06-04-15-49-28.png" alt="drawing" class="center" width="230"/></p>

dove $[\Delta_i, \Delta]$ rappresenta la Lie-bracket tra i campi vettoriali delle due distribuzioni. La procedura viene fermata quando $i=n$ o quando si trova un valore $k$ tale per cui le distribuzioni $\Delta_k$ e $\Delta_{k+1}$ sono non-singolari nel punto di interesse $x_0$ e $\dim \Delta_k(x_0) = \dim \Delta_{k+1}(x_0)$.
Dalla procedura di filtrazione applicata al sistema in esame si ottiene che la massima dimensione di $\Delta_k$ è 5, per cui il sistema in fase di volo non è \textit{small-time locally accessible}, e quindi neanche controllabile, per nessun valore di $x_0$. Questo risultato coincide con quanto ci si può intuitivamente aspettare, in quanto il sistema è in caduta libera e non si ha la possibilità di applicare forze esterne per controllarne la posizione).\\
Si nota inoltre che questo valore rappresenta la dimensione massima data dalla procedura di filtrazione, in quanto essa è stata effettuata simbolicamente e sostituendo valori specifici di un punto $x_0$ all'interno della distribuzione si può avere un'ulteriore perdita di rank.

### Osservability

To study the observability of a system there are three different approaches. In this case Iwill use the geometric differentiable approach. Analoug to the controllability case, a filtration procedure between the co-distribution $\Omega_0 = \frac{\partial h(x)}{\partial x}$ and the distribution $\Delta = span\{f(x), g_1(x),...,g_m(x)\}$
In questo caso la procedura è simile a quella riportata in precedenza, ad eccezione delle Lie-Bracket tra i campi vettoriali che sono sostituite dalle Lie-Bracket tra i campi co-vettoriali in $\Omega_k$ e i campi vettoriali in $\Delta$.\\
Applicando la procedura al sistema in esame si ottiene che (con l'uscita scelta) il massimo rank dato dalla procedura di filtrazione è 2. Questo significa che esiste un sotto-spazio di dimensione 6 per cui ogni coppia di valori dello stato iniziale $x_{f,0},\dot x_{f,0}$ che differiscono tra loro solo in questo sotto-spazio sono indistinguibili, dall'uscita scelta, per qualsiasi ingresso $u_f$.

### Feedback Linearization in MIMO systems

The standart theory for MIMO feedback linearization for a non linear square system is here reviewed. 

Given the system:


<p align="center"><img align="Center" src="images/2020-06-04-11-51-21.png" alt="drawing" class="center" width="200"/></p>
<p align="center"><img align="Center" src="images/2020-06-04-11-51-42.png" alt="drawing" class="center" width="500"/></p>
<p align="center"><img align="Center" src="images/2020-06-04-11-51-50.png" alt="drawing" class="center" width="500"/></p>
<p align="center"><img align="Center" src="images/2020-06-04-11-51-58.png" alt="drawing" class="center" width="150"/></p>
<p align="center"><img align="Center" src="images/2020-06-04-11-52-03.png" alt="drawing" class="center" width="200"/></p>
<p align="center"><img align="Center" src="images/2020-06-04-11-52-10.png" alt="drawing" class="center" width="180"/></p>
<p align="center"><img align="Center" src="images/2020-06-04-11-52-17.png" alt="drawing" class="center" width="130"/></p>
<p align="center"><img align="Center" src="images/2020-06-04-11-52-26.png" alt="drawing" class="center" width="400"/></p>
<p align="center"><img align="Center" src="images/2020-06-04-11-52-35.png" alt="drawing" class="center" width="170"/></p>
<p align="center"><img align="Center" src="images/2020-06-04-11-52-42.png" alt="drawing" class="center" width="150"/></p>
<p align="center"><img align="Center" src="images/2020-06-04-11-52-48.png" alt="drawing" class="center" width="400"/></p>
<p align="center"><img align="Center" src="images/2020-06-04-11-52-54.png" alt="drawing" class="center" width="350"/></p>
<p align="center"><img align="Center" src="images/2020-06-04-11-52-59.png" alt="drawing" class="center" width="300"/></p>


In this case the two outputs chosen for feedback linearization are the two angles: $\phi$ and $\psi$.
<!-- 
- $\psi$
- $\phi$ -->












