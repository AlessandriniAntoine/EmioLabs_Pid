# EmioLabs_PID
Lab for dynamic pid control of Emio with one leg.

Emio is a robotic platform designed for educational purposed by [Compliance Robotics](https://compliance-robotics.com/compliance-lab/).

# Installation
An additional python package ([tf-pid-tools](./tf-pid-tools)) is required to run the labs. Is it included within this lab and can be installed in the `requirements.txt` file. To install it, run the following command:

```bash
pip install -r requirements.txt
```

# Objectives

The objective of this lab is to control the Emio platform using a PID controller. The lab will cover the following topics:
1. Understanding the dynamics of the Emio platform.
2. Implementing a first PID controller.
3. Identifying a transfer function of the dynamics of the end of the leg.
4. Design an optimal PID controller based on the identified transfer function.
5. Testing the PID controller on the Emio platform.
