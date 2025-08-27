::::::: collapse Hand made PID

## Hand made PID

**PID Controller.** A *PID controller* is one of the most common control strategies for continuous-time systems.
Its control law is given by:

$$
u(t) = K_p e(t) + K_i \int_0^t e(\tau) \, d\tau + K_d \frac{d}{dt}e(t)
$$

where:
- $u(t)$ is the position of the motor,
- $e(t) = r(t) - y(t)$ is the tracking error,
- $K_p$ is the *proportional gain* (reaction to the present error),
- $K_i$ is the *integral gain* (reaction to accumulated error),
- $K_d$ is the *derivative gain* (reaction to the rate of change of the error).

Each part has a precise objective:
- *Proportional* term: corrects large errors quickly.
- *Integral* term: eliminates steady-state error.
- *Derivative* term: anticipates changes and improves stability.

The pid control structure is as follows:
|  ![](assets/data/images/labPid-pid-structure.png)   |
|:------------------------------------------------:|
| **PID control structure** |

In order to implement the PID controller, you will need to compute the three parts of the control law:
- The <span style="color:rgba(200, 0, 0, 1);">*integral term*</span>
- The <span style="color:rgba(0, 0, 200, 1);">*proportional term*</span>
- The <span style="color:rgba(0, 200, 0, 1);">*derivative term*</span>

The integral term can be computed using Euler explicit integration which leads to:
$$
integral(k) = integral(k-1) + e(k) \Delta t
$$

The derivative term can be computed using Euler explicit integration which leads to:
$$
derivative(k) = \frac{e(k) - e(k-1)}{\Delta t}
$$

:::::: exercise

**Exercise 2:**
Check the following file to do the tasks listed below.:
#open-button("assets/labs/EmioLabs_Pid/scripts/closedLoopController.py")

1. Implement the PID control law by computing: the proportional term, the integral term, and the derivative term based on the error using Euler explicit integration.
2. Select the gains of your pid controller.
::::: group-grid {style="grid-template-rows:repeat(2, 0fr);"}
**Proportional Gain**
#input("proportionalGain")

**Integral Gain**
#input("integralGain")

**Derivative Gain**
#input("derivativeGain")
:::::

3. Test your PID controller on the simulation.
#runsofa-button("assets/labs/EmioLabs_Pid/lab_pid.py" "--controller" "closedloop" "--framerate" "fps" "--motorCutoffFreq" "cutoffFreq" "--motorInit" "motorInit" "--motorMin" "motorMin" "--motorMax" "motorMax" "--nb_zeros" "nb_zeros" "--nb_poles" "nb_poles" "--optimal" "0"  "--proportionalGain" "proportionalGain" "--integralGain" "integralGain" "--derivativeGain" "derivativeGain")
::::::
:::::::
