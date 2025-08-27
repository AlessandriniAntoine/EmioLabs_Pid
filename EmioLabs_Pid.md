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
the <span style="color:grey">*grey mass*</span>, and one <span style="color:green">*green marker*</span>. We also use the **FEM** modeling of the leg to simulate the dynamics of the system.

![](assets/data/images/labPid-setup-emio.png){width=75% .center}
:::

#include(assets/labs/EmioLabs_Pid/sections/1_openloopcontrol.md)
#include(assets/labs/EmioLabs_Pid/sections/2_handmadepid.md)
#include(assets/labs/EmioLabs_Pid/sections/3_transferfunctionidentification.md)
#include(assets/labs/EmioLabs_Pid/sections/4_pidoptimization.md)
#include(assets/labs/EmioLabs_Pid/sections/5_closedloopcontrol.md)
#include(assets/labs/EmioLabs_Pid/sections/6_antiwindup.md)
