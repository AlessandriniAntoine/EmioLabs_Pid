# Lab PID

::: highlight
##### Overview

This lab is dedicated to linear control. Its goals are to make you understand:

1. **Identify** a transfert function of the dynamic of the reduced state.
2. The use ot this transfert function to **control** the robot.

This lab relies on control theory. It is recommended to check [control](https://python-control.readthedocs.io/en/0.10.2/) library documentation for more details on the implementation.
:::

::: collapse {open} Set up Emio  for the Lab
## Set up Emio

In this lab session, we will use only the following configuration: Emio with one <span style="color:rgba(200, 200, 0, 1);">*yellow leg*</span>,
the <span style="color:grey">*grey mass*</span>, and the <span style="color:green">*green marker*</span>. We also use the **FEM** modeling of the leg to simulate the dynamics of the system.

![](assets/data/images/labPid-setup-emio.png){width=75% .center}
:::



::::::: collapse Open Loop Control
## Open Loop Control

**Open Loop Control.**
The first step consists in using open-loop control within the simulation environment to generate data required for the identification of a linear model of the system. It is crucial that the simulation accurately captures the system’s dynamics, and that an appropriate time step has been chosen beforehand.

The motor used in the simulation is subject to physical constraints, including a maximum speed of 75 revolutions per minute (rpm), which must be strictly enforced. Additionally, the initial position of the motor must be specified, along with the corresponding minimum and maximum angles that can be reached from this starting point.

To avoid high-frequency oscillations in motor position, a low-pass filter is applied, defined by a given cutoff frequency $f_c$. The transfer function of the filter is given by:
$$H(s) = \frac{1}{\tau p + 1}$$
where $\tau = \frac{1}{2\pi f_c}$ is the time constant of the filter. This filter smooths the input commands to the motor, preventing abrupt changes that could lead to unrealistic behavior in the simulation. This filter can be implemented in discrete time using Euler's implicit method, resulting in the following difference equation:
$$ s(k+1) = \frac{dt}{\tau + dt} s(k) + (1 - \frac{dt}{\tau + dt}) u(k) $$
where $s(k)$ is the filtered signal at time step $k$, $u(k)$ is the signal at time step $k$, and $dt$ is the simulation time step.

To excite the system in a way that is informative for model identification, it is recommended to apply inputs drawn from a normal distribution, resulting in a pseudo-random motion of the motor. To further enrich the data, random noise is also added to the input commands.

The final open loop control structure is as follows:
|  ![](assets/data/images/labPid-openloop-structure.png)   |
|:------------------------------------------------:|
| **Open loop control structure** |


:::::: exercise
**Exercise 1:**

Check the following file to do the tasks listed below.:
#open-button("assets/labs/EmioLabs_Pid/scripts/baseController.py")

 1. Implement the function `filter` to apply the low-pass filter to a signal.

2. Set up the scene with the following parameters and try to simulate the robot with the open loop.

::::: group-grid {style="grid-template-rows:repeat(5, 0fr);"}
**Motor**
Init, Min, Max (rad)
#input("motorInit")

#input("motorMin")

#input("motorMax")

* * *
**Frame rate (Hz)**
:::: select fps
::: option 60
::: option 120
::: option 180
::: option 240
::::

**Cutoff frequency (Hz)**
#input("cutoffFreq")

:::::

#runsofa-button("assets/labs/EmioLabs_Pid/lab_pid.py" "--controller" "openloop" "--framerate" "fps" "--motorCutoffFreq" "cutoffFreq" "--motorInit" "motorInit" "--motorMin" "motorMin" "--motorMax" "motorMax")
::::::

:::::::

::::: collapse Transfer Function Identification

## Transfer Function Identification

**Transfer Function Identification.** We now aim to identify a **continuous-time transfer function** of the system.

A transfer function represents the relationship between the input and output of a **linear time-invariant (LTI)** system in the Laplace domain. It is typically written as:

$$
G(s) = \frac{Y(s)}{U(s)} = \frac{b_0 s^m + b_1 s^{m-1} + \dots + b_m}{s^n + a_1 s^{n-1} + \dots + a_n}
$$

where:
- $U(s)$ is the Laplace transform of the input signal $u(t)$,
- $Y(s)$ is the Laplace transform of the output signal $y(t)$,
- $G(s)$ is the transfer function.

A transfert function is defined by:
- its number or zeros: the number of of the numerator ($m-1$),
- its number of poles: the number of of the denominator ($n-1$),
- its order: the degree of the denominator $n$.

The transfer function describes how each frequency component of the input is modified by the system to produce the output. It is particularly useful for:
- *Frequency-domain analysis* (Bode plots, Nyquist plots),
- *Stability assessment*,
- *Control design*.


If we have time-domain measurements of the input $u(t)$ and output $y(t)$, the identification problem consists of finding the parameters $a_i$ and $b_i$ such that the transfer function's response matches the measured output.

The general steps are:

1. *Model structure selection*: Decide the expected number of zeros $m-1$ and poles $n-1$ based on physical knowledge of the system or prior analysis.
2. *Parameter estimation*: Use optimization methods, like gradient descent, to adjust $a_i$ and $b_i$ so that the model output $\hat{y}(t)$ best matches the measured $y(t)$.


Once identified, the transfer function can be validated by comparing simulated outputs against new experimental data or by analyzing the model's frequency response.

We will use a python toolbox that gives you an application to identify a transfert function. It is higly based on the function [minimize](https://docs.scipy.org/doc/scipy/reference/generated/scipy.optimize.minimize.html) from the *scipy.optimize* library. It miminize a cost function based on the error between the model output $\hat{y}(t)$ and the measured output  $y(t)$.

:::: exercise

**Exercise 3:**
Check the following file to do the tasks listed below.:
#open-button("assets/labs/EmioLabs_Pid/scripts/identification.py")

Given the reduction matrix that you generated on the previous step, you will:
1. Define the number of poles and zeros of the transfer function.
2. Compute the transfer function given an initial solution and the data.
4. Simulate the model given the same input as the system and compare the results.
#runsofa-button("assets/labs/EmioLabs_Pid/scripts/identification.py")
::::
:::::

::::::: collapse PID Tuning

## PID Tuning

**PID Tuning** A *PID controller* is one of the most common control strategies for continuous-time systems.
Its control law is given by:

$$
u(t) = K_p e(t) + K_i \int_0^t e(\tau) \, d\tau + K_d \frac{d e(t)}{dt}
$$

where:
- $e(t) = r(t) - y(t)$ is the tracking error,
- $K_p$ is the *proportional gain* (reaction to the present error),
- $K_i$ is the *integral gain* (reaction to accumulated error),
- $K_d$ is the *derivative gain* (reaction to the rate of change of the error).

Each part has a precise objective:
- *Proportional* term: corrects large errors quickly.
- *Integral* term: eliminates steady-state error.
- *Derivative* term: anticipates changes and improves stability.


We will identify the optimal $K_p$, $K_i$, and $K_d$ using a Python toolbox that searches for the parameters minimizing a *cost function*. The cost function combines *four weighted criteria*:

1. *Response time* – how fast the system reaches the reference.
2. *Transient behavior* – overshoot, oscillations, and settling time.
3. *Steady-state error* – final tracking accuracy.
4. *Control effort* – size and smoothness of the control input $u(t)$.

The optimization will be performed using [`scipy.optimize.minimize`](https://docs.scipy.org/doc/scipy/reference/generated/scipy.optimize.minimize.html),   which adjusts $K_p$, $K_i$, and $K_d$ to minimize the total cost.

Once tuned, the PID controller should be tested on:
- *Step responses* (to check speed and overshoot),
- *Different reference signals* (to check robustness),
- *Disturbances* (to check stability under perturbations).
- *Noise* (to check robustness to noise measurement).

:::::: exercise

**Exercise 4:**
Check the following file to do the tasks listed below.:
#open-button("assets/labs/EmioLabs_Pid/scripts/controller.py")

Given the linear model that you generated on the previous step, you will:
1. Select the order of the system.

::::: group-grid {style="grid-template-rows:repeat(5, 0fr);"}
**Number of zeros**
:::: select nb_zeros
::: option 0
::: option 1
::: option 2
::: option 3
::: option 4
::: option 5
::::

**Number of poles**
:::: select nb_poles
::: option 0
::: option 1
::: option 2
::: option 3
::: option 4
::: option 5
::::
:::::

2. Compute the PID gains.
#runsofa-button("assets/labs/EmioLabs_Pid/scripts/controller.py" "--nb_zeros" "nb_zeros" "--nb_poles" "nb_poles")
::::::
:::::::

::::: collapse Closed Loop Control

## Closed Loop Control

**Closed Loop Control.**

This setup will first be implemented in SOFA, which provides the system dynamics and sensor measurements. It enables evaluation of the full closed-loop system without requiring physical hardware, while still respecting realistic sensing and estimation constraints.

#runsofa-button("assets/labs/EmioLabs_Pid/lab_pid.py" "--controller" "closedloop" "--framerate" "fps" "--motorCutoffFreq" "cutoffFreq" "--motorInit" "motorInit" "--motorMin" "motorMin" "--motorMax" "motorMax" "--nb_zeros" "nb_zeros" "--nb_poles" "nb_poles")

In a second step, the same control law can be deployed on the real robot. The observer will estimate the state based on measured outputs (e.g., marker positions), and the computed control input $u$ will be applied to the motor.

```bash
$ python scripts/hardware.py --motorCutoffFreq cutoffFreq --motorInit motorInit --motorMin motorMin --motorMax motorMax --nb_zeros nb_zeros --nb_poles nb_poles
```

::: highlight
#icon("warning") Insight:
This architecture enables real-time control using only partial measurements, and bridges the gap between model-based design and physical implementation. The exact same controller and observer gains designed in simulation can be reused on the real system, as long as the model accurately captures the dynamics.
:::
:::::
