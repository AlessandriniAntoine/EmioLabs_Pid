::::: collapse Closed Loop Control

## Closed Loop Control

**Closed Loop Control.**

This setup will first be implemented in SOFA, which provides the system dynamics and sensor measurements. It enables evaluation of the full closed-loop system without requiring physical hardware, while still respecting realistic sensing and estimation constraints.

#runsofa-button("assets/labs/EmioLabs_Pid/lab_pid.py" "--controller" "closedloop" "--framerate" "fps" "--motorCutoffFreq" "cutoffFreq" "--motorInit" "motorInit" "--motorMin" "motorMin" "--motorMax" "motorMax" "--nb_zeros" "nb_zeros" "--nb_poles" "nb_poles" "--optimal" "1")

In a second step, the same control law can be deployed on the real robot. The observer will estimate the state based on measured outputs (e.g., marker positions), and the computed control input $u$ will be applied to the motor.

```bash
$ python scripts/hardware.py --motorCutoffFreq cutoffFreq --motorInit motorInit --motorMin motorMin --motorMax motorMax --nb_zeros nb_zeros --nb_poles nb_poles
```

:::::
