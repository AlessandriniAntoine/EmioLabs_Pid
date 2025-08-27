::::: collapse Transfer Function Identification

## Transfer Function Identification

**Transfer Function Identification.** PID tuning can be complex. Therefore, it is often useful to identify the transfer function of a system to better understand its behavior and design an optimal controller.

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
