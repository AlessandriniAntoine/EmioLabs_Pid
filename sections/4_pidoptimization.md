::::::: collapse Optimal PID Tuning

## Optimal PID Tuning

**PID Tuning**. Based on the model, you can you optimization technics to determine

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
