# The 7 DOF Franka Emika Panda robot


## Table of Content

- [The 7 DOF Franka Emika Panda robot](#the-7-dof-franka-emika-panda-robot)
  - [Table of Content](#table-of-content)
  - [Appendix A with results plot at full size](#appendix-a-with-results-plot-at-full-size)
  - [The robotic arm](#the-robotic-arm)
    - [Robot modelling](#robot-modelling)
    - [Overview of the Implemented Controllers](#overview-of-the-implemented-controllers)
      - [Non adaptive](#non-adaptive)
      - [Adaptive Methods](#adaptive-methods)
    - [Code Structure](#code-structure)
    - [Reference Trajectories](#reference-trajectories)
    - [Pick and Place](#pick-and-place)
      - [Trajectory definition](#trajectory-definition)
      - [Dynamic simulation](#dynamic-simulation)
      - [Independent PD controller](#independent-pd-controller)
        - [Case I: Robot gravity is ignored](#case-i-robot-gravity-is-ignored)
        - [Case II: Robot gravity is no longer ignored](#case-ii-robot-gravity-is-no-longer-ignored)
        - [Case III: Gravity Compensation](#case-iii-gravity-compensation)
    - [Independent PID controller](#independent-pid-controller)
    - [Computed Torque](#computed-torque)
    - [Computed torque Robustness and errors in the dynamical model](#computed-torque-robustness-and-errors-in-the-dynamical-model)
    - [Pick and Place Results Overview](#pick-and-place-results-overview)
  - [Trajectory Tracking](#trajectory-tracking)
      - [The Backstepping Control](#the-backstepping-control)
      - [Response to different trajectories speed and amplitude](#response-to-different-trajectories-speed-and-amplitude)
  - [Perturbations of Dynamical parameters and Adaptive strategies](#perturbations-of-dynamical-parameters-and-adaptive-strategies)
    - [Dynamic Regressor](#dynamic-regressor)
- [Appendix: Full scale Images for circumference Tracking](#appendix-full-scale-images-for-circumference-tracking)
    - [Circumference](#circumference)
      - [Computed Torque Controller](#computed-torque-controller)
      - [Backstepping Controller](#backstepping-controller)
      - [Controller Overview](#controller-overview)
    - [Helix](#helix)
      - [Desired Joint Trajectory:](#desired-joint-trajectory)
      - [Computed Torque controller](#computed-torque-controller-1)
      - [Backstepping Control](#backstepping-control)
      - [Backstepping vs Computed Torque](#backstepping-vs-computed-torque)


## Appendix A with results plot at full size

  - [Circumference](#circumference)
    - [Computed Torque Controller](#computed-torque-controller)
    - [Backstepping Controller](#backstepping-controller)
    - [Controller Overview](#controller-overview)
  - [Helix](#helix)
    - [Desired Joint Trajectory:](#desired-joint-trajectory)
    - [Computed Torque controller](#computed-torque-controller-1)
    - [Backstepping Control](#backstepping-control)
    - [Backstepping vs Computed Torque](#backstepping-vs-computed-torque)
 
## The robotic arm

This  7 DOF Franka Emika Panda robot is  equipped  with 7 revolute joints, each mounting a torque sensor, and it has a total weight of approximately 18kg, having the possibility to handle payloads up to 3kg.

  Figure 1 shows  the  Franka  Emika  Panda  robot, its geometrical parameters and the used DH convention.   

<p align="center">
  <img align="Center" src="./images/2020-06-03-11-50-28.png" alt="drawing" class="center" width="450" />
</p>


### Robot modelling

The first step was to model the robot using Matlab Robot Control toolbox.

The robot is defined according to the DH table and its parameter are defined and assigned. This is done in 
```robot_gen.m```

The model is saved as ```PANDA```.

The following paramters are used: 
<!-- $$d1 = 0.333;d3=0.316;d5=0.384;a4=0.0825;a5=-0.0825;a7=0.088;$$
$$m1=3.4525; m2=3.4821; m3=4.0562; m4=3.4822; m5=2.1633; m6=2.3466; m7=0.31290;$$ -->

![](./images/2020-06-10-19-31-49.png)
The parameters are found in  [this online repository](https://github.com/marcocognetti/FrankaEmikaPandaDynModel.git) while the exact procedure is described here:

[Dynamic Identification of the Franka Emika Panda Robot with Retrieval of Feasible Parameter Using Penalty-based Optimization](https://hal.inria.fr/hal-02265293/document)

The resulting model is: 

<!-- <p align="center">

</p>

Following the Denavit-Hartenberg convention: -->

<!-- $$ L1 = Link([0,d1,  0,  0]), $$
$$ L2 = Link([0,0, 0,  -pi/2])   $$
$$ L3 = Link([0,d3, 0,  pi/2])   $$
$$ L4 = Link([0,0,  a4,  pi/2])   $$
$$ L5 = Link([0,d5,  a5,  -pi/2])   $$
$$ L6 = Link([0,0,  0,  pi/2])   $$
$$ L7 = Link([0,0,  a7,  pi/2])   $$ -->




<p align="center">
  <!-- <img align="Center" src="./images/2020-06-04-01-14-39.png" alt="drawing" class="center" width="250" /> -->
  <img align="Center" src="./images/2020-06-03-11-50-45.png" alt="drawing" class="center" width="600" />
  <img src="./images/2020-05-27-17-00-51.png" alt="drawing" class="bottom" width="600" margin="100" />
</p>


Once we have the dynamical model properly set up, we can proceed to implement the controllers.

### Overview of the Implemented Controllers


#### Non adaptive


|                              | Pick and Place     | Circumference      | Helix              |
| ---------------------------- | ------------------ | ------------------ | ------------------ |
| PD                           | :heavy_check_mark: | :x:                | :x:                |
| PD with gravity compensation | :heavy_check_mark: | :x:                | :x:                |
| PID                          | :heavy_check_mark: | :x:                | :x:                |
| Computed torque              | :heavy_check_mark: | :heavy_check_mark: | :heavy_check_mark: |
| Backstepping                 | :x:                | :heavy_check_mark: | :heavy_check_mark: |

####  Adaptive Methods 

|                          | Pick and Place     | Circumference      | Helix              |
| ------------------------ | ------------------ | ------------------ | ------------------ |
| Adaptive Computed Torque | :heavy_check_mark: | :heavy_check_mark: | :heavy_check_mark: |
| Adaptive Backstepping    | :x:                | :heavy_check_mark: | :heavy_check_mark: |
| Li Slotine               |:x: | :heavy_check_mark: |  :heavy_check_mark: |


### Code Structure

Before I dive into the report, I would line to outline the code structure. In the repository, there are essentially three folders. 
- Pick and place: It contains every result for the pick and place simulation. Every controller will be found in 
```
./pick_and_place_results/pickandplace_ (desired controller).m
```
like: 
```
./pick_and_place_results/pickandplace_computedtorque.m
```
Most of the results are already stored as (desired controller)_results.mat

Comparative plots for the pick and place scenario can be found in 

```
./pick_and_place_results/plot_results.m
```

- Non adaptive control



- Adaptive control

### Reference Trajectories

I defined three reference trajectories:

|   Pick and Place	|   Circumference	|   Helix	|
|---	|---	|---	|
|   <img src="./images/2020-05-27-17-05-02.png" alt="drawing" class="center" width="300" />	|   <img src="./images/CircularTrajectory.gif" alt="drawing" class="center" width="300" />	|   	<img src="./images/HelixTrajectory.gif" alt="drawing" class="center" width="300" />|

Each trajectory will now be discussed in details with the implemented controllers.

### Pick and Place

#### Trajectory definition

Initial Conditions

![](./images/2020-06-10-19-32-33.png)
<!-- $$ q_{ini} = [0, 0, 0, 0, 0, 0, 0] $$ -->

Final Conditions

![](./images/2020-06-10-19-32-44.png)
<!-- $$ q_{des} = [pi/3, 0, pi/3, pi/3, pi/6, 0 , 0]$$ -->



#### Dynamic simulation

Starting from the initial conditions I compute the following quantities in a for loop over a 10s period with time constant 1ms :

- Error, derivate of the error, integral of the error
![](./images/2020-06-10-19-32-56.png)
<!-- $$ err = q_{des} - q $$
$$ \frac {\partial err}{\partial t} = \frac {\partial (q_{des}-q)}{\partial t}  $$
$$ \Delta ierr=(err+err_{old})*\delta_{t/2} $$ -->

- Dynamic matrices

<!-- $$ F = \text{FrictionTorque}(dq) $$ 
$$ G = \text{GravityVector(q)} $$
$$ C = \text{CoriolisVector(q,dq)} $$
$$ M = \text{MassMatrix(q)} $$ -->
![](./images/2020-06-10-19-33-07.png)

- Torque, depending on the controller (PD, PD with G compensation, PID)
 
<!-- $$\tau = ( Kv*(derr') + Kp*(err'))'; $$


<!-- $$ \text{PD Controller with G compensation}$$ -->
<!-- $$ \tau = ( Kv*(derr') + Kp*(err') + G' $$ -->

<!-- $$\text{PID Controller }$$ -->
<!-- $$ \tau = ( Kv*(derr') + Kp*(err')+ Ki*(ierr') + G'$$  --> 
![](./images/2020-06-10-19-33-30.png)

And last, accelleration, velocity and displacement resulting from the compute torque applied for a delta t of 1ms.


![](./images/2020-06-10-19-33-44.png)
<!-- $$\ddot q = M^{-1}*(\tau - C- G)$$ -->

Tustin integration:
![](./images/2020-06-10-19-33-52.png)

<!-- $$\Delta \dot q = (\ddot q_{old} + \ddot q) * \delta_{t} / 2$$

$$\Delta q = (\dot q_{old}+\dot q   ) * \delta_{t} /2$$ -->



#### Independent PD controller

The first and most simple solution to control the end effector to a desired pose is to apply standard independent PD controller to follow references. 

The aim of using Proportional-Derivative (PD) controller is to increase the stability of the system by improving control since it has an ability to predict the future error of the system response. In order to avoid effects of the sudden change in the value of the error signal, the derivative is taken from the output
response of the system variable instead of the error signal. Therefore, D mode is
designed to be proportional to the change of the output variable to prevent the
sudden changes occurring in the control output resulting from sudden changes in the error signal. 

<p align="center">
<img src="./images/2020-06-03-11-21-30.png" alt="drawing" class="center" width="180" />
</p>

The error is defined as: 
<p align="center">
<img src="./images/2020-06-03-11-34-17.png" alt="drawing" class="center" width="180" />
</p>
And the control law is defined as: 

<p align="center">
<img src="./images/2020-06-03-11-36-03.png" alt="drawing" class="center" width="230" />
</p>
The Dynamic thus will be:
<p align="center">
<img src="./images/2020-06-03-11-39-35.png" alt="drawing" class="center" width="350" />
</p>

In order to verify the stability at the desired value Direct Lyapunov is used with the following candidate:
<p align="center">
<img src="./images/2020-06-03-11-38-03.png" alt="drawing" class="center" width="250" />
</p>
Deriving the equation we obtain the semi negative definite value:
<p align="center">
<img src="./images/2020-06-03-11-39-09.png" alt="drawing" class="center" width="600" />
</p>
Also we must verify that:
<p align="center">
<img src="./images/2020-06-03-11-41-56.png" alt="drawing" class="center" width="380" />
</p>

This implies that PD control the error converges to 0 if the following hypothesis are true:
<!-- 
 $$ G(qd) = 0 $$

 $$  q_d=H(t) \text{ (Heaviside Step function)}$$  -->

![](./images/2020-06-10-19-34-10.png)
The first hypotesis will be analyzed in the simulation, while the second means that only step reference function can be given as input. This is ideal for pick and place.

I start with the following gains and then proceed to tune them for each specific controller:


<!-- $$ Kp = diag([30, 30, 30, 30, 30, 30, 30]) $$
$$ Kv = diag([1, 1, 1, 1, 1, 1, 1]) $$ -->
![](./images/2020-06-10-19-34-21.png)
<!-- <p float="left">
  <img src="./images/PD_control_no_gravity.jpg" width="220" />
  <img src="./images/PD_with_gravity.jpg" width="300" /> 
  <img src="./images/PD_with_G_compensation.jpg" width="258" />
</p> -->

##### Case I: Robot gravity is ignored

If the remove the G term from the dynamic equation than both hypothesis are true and the error converges to 0.


<p align="center">
<img src="./images/PD_control_no_gravity.jpg" alt="drawing" class="center" width="800" />
</p>

##### Case II: Robot gravity is no longer ignored

If we reinsert gravity one hypothesis is no longer true and we have an error offset in the solution:
<p align="center">
<img src="./images/PD_with_gravity.jpg" alt="drawing" class="center" width="800" height="700" />
</p>



It is clear how gravity affects only some joints depending on the configuration of the robot.

##### Case III: Gravity Compensation

We can solve this by compensating the gravity term with the following law:
![](./images/2020-06-10-19-34-33.png)
<!-- $$\tau = Kv*derr + Kp*err + G$$ -->
<p align="center">
<img src="./images/PD_with_G_compensation.jpg" alt="drawing" class="center" width="800" height="730" />
</p>


### Independent PID controller



Proportional-Integral-Derivative (PID) controller has the optimum control
dynamics including zero steady state error, fast response (short rise time), no
oscillations and higher stability. The necessity of using a derivative gain
component in addition to the PI controller is to eliminate the overshoot and the
oscillations occurring in the output response of the system. 


<p align="center">
<img src="./images/2020-06-03-18-11-23.png" alt="drawing" class="center" width="500" />
</p>

| PID Without gravity compensation  | PID With gravity compensation  |
|---|---|
|The initial offset slowly converges back to the right value (null steady state error). This PID doesn't require any knowledge of the gravity load for the manipulator.| We don't have anymore the big error offset on the second and sixth configuration variable. However it doesn't have the big advantage of the PID (zero knowledge about gravity load ) |
| ![](./images/2020-06-03-14-31-06.png)  | ![](./images/2020-05-28-18-27-50.png)  |

 
It is worth mentioning that tuning the PID to achieve stability is much more difficult than tuning just the P and D components.

### Computed Torque
<p align="center">
<img src="./images/2020-06-03-12-03-58.png" alt="drawing" class="center" width="400" />
</p>


In order to achieve perfect reference tracking, the system should have an in built inner dynamical model that can be used to compute a feedforward action that complements the feedback action seen previously. A control that does this is the following:
<p align="center">
<img src="./images/2020-06-03-12-06-26.png" alt="drawing" class="center" width="430" />
</p>

Substituting this in the dynamics we obtain:
<p align="center">
<img src="./images/2020-06-03-12-07-38.png" alt="drawing" class="center" width="530" />
</p>
Which means that the error dynamics will be:
<p align="center">
<img src="./images/2020-06-03-12-08-41.png" alt="drawing" class="center" width="270" />
</p>
<p align="center">
<img src="./images/2020-06-03-12-08-47.png" alt="drawing" class="center" width="200" />
</p>

Essentially, the perfect knowledge of the dynamical model allows for complete linearization and desired pole placement. The trajectory tracking can be solved with arbitrarly fast convergence.

![](./images/2020-05-28-23-56-17.png)



### Computed torque Robustness and errors in the dynamical model

When Noise in dynamical parameters knowledge or friction or other sources of error are introduced It is much more difficult to have stability guarantees for the Computed Torque controller. 
In the following results, it is clear how deeply even a random error $<5\%$ in the mass estimation affects the performances.
![](./images/2020-06-03-23-25-00.png)

A more formal and correct approach would be to apply a control law with estimated M, G and C:
<!-- 
$$ \tau=\tilde M(q)\ddot{q_d}+K_v\dot{e}+Kp e+  \tilde C(q, \dot q)\dot q + \tilde G(q) $$ -->
![](./images/2020-06-10-19-35-07.png)
Substituting the dynamics:
![](./images/2020-06-10-19-35-30.png)
<!-- $$ \ddot e +Kv \dot e +Kp e = \eta $$

$$ \eta = \tilde M ^{-1}(\tilde M(q)\ddot{q_d}+  \tilde C(q, \dot q)\dot q + \tilde G(q)) $$ -->

Where $\eta$ depents non linearly from q.

### Pick and Place Results Overview

The PD surely is the easiest control, but suffers severe limits in terms of both hypotesis and performances. Gravity needs to be compensated to allow it converges to the desired value.

The integral component can compensate for some of the limits of the PD independent controller, however it was extremely difficult to achieve a good tuning.

The Computed Torque method allows desired pole placement and performances, however requires the exact dynamical model to linearize the dynamic.

The computer Torque method is only robust up to a certain error margin in the estimation of the dynamical parameters.

![](./images/2020-05-29-00-23-10.png)

## Trajectory Tracking

Given a trajectory in the cartesian space, I compute the references in the joint space with the following block:

![](./images/2020-05-28-19-28-23.png)

I use the non weighted pseudoinverse, therefore the solution will minimize the norm of the q.

Now I will proceed to define the used trajectories and the results for each controller.

| Circumference  | Helix  |
|---|---|
| <img src="./images/2020-06-03-17-55-13.png" width="300" />  |<img src="./images/2020-06-03-17-55-25.png" width="380" />   |
| <img src="./images/circumference.gif" width="400" />  |   <img src="./images/helix2.gif" width="400" />   |
| Desired Circumference Joint Trajectories <img src="./images/2020-05-28-19-41-46.png" width="400" />  |  Desired Helix Joint Trajectories<img src="./images/2020-06-03-14-44-44.png" width="400" />  |
|Computed Torque Controller   <img src="./images/2020-06-03-17-53-57.png" width="400" height ="320" />  | Computed Torque Controller  <img src="./images/2020-06-03-16-39-35.png" width="400" />  |
| Backstepping Controller<img src="./images/2020-06-03-17-51-02.png" width="400" />   |  Backstepping Controller<img src="./images/2020-06-03-16-44-13.png" width="400" />  |
|  Performance comparison between CT and BS <img src="./images/2020-06-03-17-50-01.png" height ="275" width="400" />  |  Performance comparison between CT and BS <img src="./images/2020-06-03-16-46-33.png" width="350" />  |



<!-- Given the following trajectories:

- Circumference with changing End Effector pose ($\theta$ function of time)

$$x = x_0 + r * cos(t*2*pi)      $$
$$y = y_0                     $$
$$z = z_0 + r * sin(t*2*pi)      $$
$$\theta = 0.1*sin(t/5*2*pi)                        $$
$$\phi = 0                             $$
$$\psi = 0                             $$


- Helix (With constant End Effector Orientation)


$$x = x_0 + r * cos(t*num*2*pi);         $$
$$y = y_0 + t*num*shift;                 $$
$$z = z_0 + r * sin(t*num*2*pi);         $$
$$\theta =  0;                           $$
$$\phi =    0;                           $$
$$\psi =    0;                           $$ -->




#### The Backstepping Control

The backstepping control can be used when system dynamics can be partitioned such that: 

<p align="center">
<img src="./images/2020-06-04-09-05-59.png" alt="drawing" class="center" width="310" />
</p>


The purpose of the backstepping controller is to assume that a controller design exist for the higher level system, which means that a feedback law exist for $\xi_2$ such that a Lyapunov function for $\xi_1$ is negative semi-definite.
<p align="center">
<img src="./images/2020-06-04-09-11-22.png" alt="drawing" class="center" width="190" />
</p>

<p align="center">
<img src="./images/2020-06-04-09-11-30.png" alt="drawing" class="center" width="310" />
</p>

<p align="center">
<img src="./images/2020-06-04-09-11-41.png" alt="drawing" class="center" width="410" />
</p>


<p align="center">
<img src="./images/2020-06-04-09-11-49.png" alt="drawing" class="center" width="350" />
</p>

<p align="center">
<img src="./images/2020-06-04-09-12-01.png" alt="drawing" class="center" width="430" />
</p>

<p align="center">
<img src="./images/2020-06-04-09-12-12.png" alt="drawing" class="center" width="260" />
</p>

<p align="center">
<img src="./images/2020-06-04-09-12-18.png" alt="drawing" class="center" width="350" />
</p>



#### Response to different trajectories speed and amplitude


I will now investigate what happens if the speed of the reference trajectory is increased. The equation now is:
![](./images/2020-06-10-19-35-54.png)
<!-- $$x = x_0 + r * cos(t*6*pi)      $$
$$y = y_0                     $$
$$z = z_0 + r * sin(t*6*pi)      $$
$$\theta = 0.1*sin(t/3*2*pi)                        $$
$$\phi = 0                             $$
$$\psi = 0                             $$ -->

Which correpsonds to the following joint trajectory:

<p align="center">
<img src="./images/faster_trajectory.gif" alt="drawing" class="center" width="410" />
<img src="./images/2020-06-04-00-46-36.png" alt="drawing" class="center" width="433" />
</p>

With the computed torque method, it is really simple to find a good set of paramteres to have really good controller performances:

Parameters for CT control:
<!-- $$ Kp = diag([3, 3 ,3 ,3 ,5 ,3 ,30]) $$
$$ Kv = 1/2 *diag([1 ,1 ,1 ,1 , 7 ,2 ,1]) $$ -->
![](./images/2020-06-10-19-36-07.png)
Parameters for Backstepping:
<!-- $$Kp = 1* diag([1 ,1 ,1 ,1 ,3 ,1 ,1])$$ -->
![](./images/2020-06-10-19-36-31.png)
| CT results  |  Backstepping results |
|---|---|
|  <img src="./images/2020-06-04-00-56-19.png" alt="drawing" class="center" width="370" /> |  <img src="./images/2020-06-04-01-00-38.png" alt="drawing" class="center" width="377" /> |






The controllers provide very good performances despite of the speed of the reference trajectory.

## Perturbations of Dynamical parameters and Adaptive strategies

What happens if dynamical parameters estimation is biased? 

```
for j = 1:num_of_joints % for each joint/link
    PANDAmodel.links(j).m = PANDAmodel.links(j).m .* (1+randi(5,1)/100); %masse [1x1]
end
```

In the following section dynamical parameters are randomly increased up to 5%.

### Dynamic Regressor

The regressor has been computed in Wolfram Mathematica 12, using the Screw Calclus Package. 
The computation of the regressor for the Kinova 7DOF manipulator revealed to be too expensive, therefore I proceeded to compute the regressor for a simplyfied version of the robot (removing the last two links). The computation for the 5 DOF arm infact was the limit in terms of the sheer size of the output that mathematica could handle.

From now on I will refer to this simplyfied model.




















 
  
   



       





# Appendix: Full scale Images for circumference Tracking


### Circumference 


![](./images/circumference.gif)

Desired joint trajectories:

![](./images/2020-05-28-19-41-46.png)

#### Computed Torque Controller


![](./images/2020-06-03-17-53-57.png)


#### Backstepping Controller

In order to implement the backstepping controller I have to define some other quantities.

$$\tau = M*ddqr + C*dqr + F + G + Kp*s + err $$

![](./images/2020-06-03-17-51-02.png)

#### Controller Overview

![](./images/2020-06-03-17-50-01.png)

### Helix 

![](./images/helix2.gif)

#### Desired Joint Trajectory:

![](./images/2020-06-03-14-44-44.png)

#### Computed Torque controller

The controller applies the following torque:


$$\tau = ( M*(ddq_{des} + Kv*derr + Kp*err) + C*dq + F + G )$$


After some gains tuning the following tracking is achieved. The tracking is almost perfect but again, the computed torque method requires complete knowledge of the dynamical system. It is much more interesting to understand what happens when there are dynamical parameters uncertainties.

![](./images/2020-06-03-16-39-35.png)

#### Backstepping Control

![](./images/2020-06-03-16-44-13.png)

#### Backstepping vs Computed Torque

![](./images/2020-06-03-16-46-33.png)