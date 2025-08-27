::::::: collapse Integral Anti-windup

# Anti-windup in a PID Controller

**Integral anti-windup.** In a PID controller, the *integral term* accumulates the error over time to eliminate steady-state error.
However, when the control action is *saturated* (e.g., actuator limits such as a motor voltage between 0 V and 10 V), the integrator may keep growing.

This leads to *integrator windup*, which can cause:
- large overshoot,
- slower recovery,
- temporary instability.

Windup can happen not only because of actuator saturation, but also due to:
- *disturbances* that prevent the system from reaching the desired command,
- *unreachable commands* that exceed actuator capabilities.

The idea is to *prevent the integrator from accumulating unrealistic error* when the control action cannot be achieved.

Common Methods
1. *Clamping (conditional integration)*:
   Stop integration if the control signal is saturated *and* the error would push further in the same direction.

2. *Back-calculation*:
   Add a correction term to the integrator based on the difference between the saturated control $\bar{u}(t)$ and the unsaturated control $u(t)$:
   $$ \text{integral term} = K_i \int_{0}^{t}e(\tau)d\tau + K_{b}\,(\bar{u}(t) - u(t)) $$
   where $K_{b}$ is the anti-windup gain.

We will use the back-calculation structure. It leads to the following control structure:
|  ![](assets/data/images/labPid-antiwindup-structure.png)   |
|:------------------------------------------------:|
| **PID control structure with back-calculation anti-windup** |

:::::: exercise

**Exercise 5:**
Check the following file to do the tasks listed below.:
#open-button("assets/labs/EmioLabs_Pid/scripts/closedLoopController.py")

1. Modify the integral term to include the back-calculation.
2. Select the gain for you back-calculation.
::::: group-grid {style="grid-template-rows:repeat(2, 0fr);"}
**Back Calculation Gain**
#input("backCalculationGain")
:::::

3. Test your uptaded PID controller on the simulation.
#runsofa-button("assets/labs/EmioLabs_Pid/lab_pid.py" "--controller" "closedloop" "--framerate" "fps" "--motorCutoffFreq" "cutoffFreq" "--motorInit" "motorInit" "--motorMin" "motorMin" "--motorMax" "motorMax" "--nb_zeros" "nb_zeros" "--nb_poles" "nb_poles" "--optimal" "1"  "--backCalculationGain" "backCalculationGain")
::::::

:::::::
