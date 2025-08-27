from .baseController import *


ControlMode = {"Open Loop": False, "State Feedback": True}


class ClosedLoopController(BaseController):
    def __init__(self, leg, motor, markers, load, motorInit, motorMin, motorMax, cutoffFreq, nb_zeros, nb_poles, optimal, proportionalGain, integralGain, derivativeGain, backCalculationGain):
        super().__init__(leg, motor, markers, load, motorInit, motorMin, motorMax, cutoffFreq)

        # add mechanical object for reference
        self.refMo = self.guiNode.addObject("MechanicalObject",
            name="refMo",
            template="Vec3d",
            position=[[0, 0, 0]],
            showObject=True,
            showObjectScale=3,
            drawMode=1,
            showColor=[0, 0, 1, 1]
        )

        self.setup_additional_variables(nb_zeros, nb_poles, optimal, proportionalGain, integralGain, derivativeGain, backCalculationGain)
        self.setup_additional_gui()


    def setup_additional_variables(self, nb_zeros, nb_poles, optimal, proportionalGain, integralGain, derivativeGain, backCalculationGain):

        # pid gains
        if not optimal:
            self.Kp = proportionalGain
            self.Ki = integralGain
            self.Kd = derivativeGain

        else:
            pid_path = os.path.join(data_path, f"pid_{nb_zeros}zeros_{nb_poles}poles.npz")
            pid_data = np.load(pid_path)
            self.Kp = pid_data["proportionalGain"]
            self.Ki = pid_data["integralGain"]
            self.Kd = pid_data["derivativeGain"]
        self.kp_exposant = int(np.floor(np.log10(abs(self.Kp))))-1
        self.ki_exposant = int(np.floor(np.log10(abs(self.Ki))))-1
        self.kd_exposant = int(np.floor(np.log10(abs(self.Kd))))-1
        self.kp_init, self.ki_init, self.kd_init = self.Kp, self.Ki, self.Kd

        self.Kb = backCalculationGain
        self.use_antiwindup = (self.Kb > 0)
        if self.use_antiwindup:
            self.kb_exposant = int(np.floor(np.log10(abs(self.Kb))))-1
            self.kb_init = self.Kb

        # states for closed-loop control
        self.dt = self.root.dt.value
        self.integral = 0
        self.error_prev = 0
        self.reference = np.zeros((1,))
        self.command = np.zeros((1,))
        self.command_sat = np.zeros((1,))

        # additional data storage
        self.commandModeList = []


    def execute_control_at_camera_frame(self):
        # observer
        desiredMotorPos = self.currentMotorPos.copy()

        if self.guiNode.active.value:
            self.reference = np.array([self.guiNode.reference.value])
            markersPos = self.markers.position.value.flatten()
            self.refMo.position.value = np.array([[self.initRefMo[0], markersPos[1], self.initRefMo[2] + self.reference[0]]])

        # control
        if self.guiNode.controlMode.value == ControlMode["State Feedback"]:
            measure = self.markersPos[1, 0] + np.random.normal(0, self.guiNode.noise.value, 1)[0]
            error = self.reference[0] - measure

            # === Step 1: Euler explicite integration ===
            # TODO: Compute integral and derivative using Euler explicit integration
            if self.use_antiwindup:
                antiwidup_term = self.Kb * (self.command_sat - self.command)
                self.integral += error * self.dt + antiwidup_term
            else:
                self.integral += error * self.dt
            derivative = (error - self.error_prev) / self.dt
            self.error_prev = error

            # === Step 2: Implement PID control law ===
            # TODO: Implement PID control law
            desiredMotorPos = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
            if self.use_antiwindup:
                self.command = desiredMotorPos
                desiredMotorPos = np.clip(desiredMotorPos, self.motorMin, self.motorMax)
                self.command_sat = desiredMotorPos

            self.motor.position.value = desiredMotorPos.flatten()[0] * 1e2
        else:
            if self.guiNode.active.value:
                desiredMotorPos[0] = self.motor.position.value * 1e-2

        self.command = self.filter(
            desiredMotorPos, self.command,
            cutoffFreq=self.cutoffFreq, samplingFreq=self.samplingFreq)


    def setup_additional_gui(self):
        # Specific gui variables
        self.guiNode.addData(name="noise", type="float", value=0.)
        self.guiNode.addData(name="reference", type="float", value=0.)
        self.guiNode.addData(name="output", type="float", value=0.)
        self.guiNode.addData(name="controlMode", type="bool", value=ControlMode["Open Loop"])
        self.guiNode.addData(name="Kp", type="float", value=self.Kp/(10**self.kp_exposant))
        self.guiNode.addData(name="Ki", type="float", value=self.Ki/(10**self.ki_exposant))
        self.guiNode.addData(name="Kd", type="float", value=self.Kd/(10**self.kd_exposant))
        if self.use_antiwindup:
            self.guiNode.addData(name="Kb", type="float", value=self.Kb/(10**self.kb_exposant))
        self.guiNode.addData(name="reset", type="int", value=0)
        self.guiNode.addData(name="reset_gain", type="int", value=0)

        # specific gui data
        MyGui.MyRobotWindow.addSettingInGroup("Reference (mm)", self.guiNode.reference, -50, 50, "Control Law")
        MyGui.MyRobotWindow.addSettingInGroup("Noise (mm)", self.guiNode.noise, 0, 3, "Control Law")
        MyGui.MyRobotWindow.addSettingInGroup("Control Mode", self.guiNode.controlMode, 0, 1, "Buttons")
        MyGui.MyRobotWindow.addSettingInGroup(f"Kp (10^{self.kp_exposant})", self.guiNode.Kp, 0, 100, "PID")
        MyGui.MyRobotWindow.addSettingInGroup(f"Ki (10^{self.ki_exposant})", self.guiNode.Ki, 0, 100, "PID")
        MyGui.MyRobotWindow.addSettingInGroup(f"Kd (10^{self.kd_exposant})", self.guiNode.Kd, 0, 100, "PID")
        if self.use_antiwindup:
            MyGui.MyRobotWindow.addSettingInGroup(f"Kb (10^{self.kb_exposant})", self.guiNode.Kb, 0, 100, "PID")
        MyGui.MyRobotWindow.addSettingInGroup("Reset Gains", self.guiNode.reset_gain, 0, 1, "PID")
        MyGui.MyRobotWindow.addSettingInGroup("Reset", self.guiNode.reset, 0, 1, "Buttons")

        # Plotting data
        MyGui.PlottingWindow.addData("Reference", self.guiNode.reference)
        MyGui.PlottingWindow.addData("Output", self.guiNode.output)


    def execute_control_at_simu_frame(self):
        super().execute_control_at_simu_frame()
        self.guiNode.output.value = self.markersPos[1, 0]
        if self.guiNode.reset_gain.value:
            self.guiNode.Kp.value = self.kp_init / (10**self.kp_exposant)
            self.guiNode.Ki.value = self.ki_init / (10**self.ki_exposant)
            self.guiNode.Kd.value = self.kd_init / (10**self.kd_exposant)
            if self.use_antiwindup:
                self.guiNode.Kb.value = self.kb_init / (10**self.kb_exposant)
            self.guiNode.reset_gain.value = False
        if self.guiNode.reset.value:
            self.integral = 0
            self.error_prev = 0
            self.guiNode.reset.value = False


    def record_data(self):
        super().record_data()
        self.commandModeList.append(self.guiNode.controlMode.value)


    def initialize_simulation(self):
        super().initialize_simulation()
        markerPos = self.markers.position.value.flatten()
        self.refMo.position.value = np.array([[markerPos[0]-10, markerPos[1], markerPos[2]]])
        self.initRefMo = self.refMo.position.value.flatten()


    def save(self):
        print("Saving data...")
        np.savez(
            os.path.join(data_path, "sofa", "closedLoop.npz"),
            legVel=np.array(self.legVelList).reshape(len(self.legVelList), self.legVelList[0].shape[0]),
            legPos=np.array(self.legPosList).reshape(len(self.legPosList), self.legPosList[0].shape[0]),
            markersPos=np.array(self.markersPosList).reshape(len(self.markersPosList), self.markersPosList[0].shape[0]),
            motorPos=np.array(self.motorPosList).reshape(len(self.motorPosList), self.motorPosList[0].shape[0]),
            commandMode=np.array(self.commandModeList).reshape(len(self.commandModeList), 1),
            fps=1 / self.root.dt.value,
        )
