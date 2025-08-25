from .baseController import *


ControlMode = {"Open Loop": False, "State Feedback": True}


class ClosedLoopController(BaseController):
    def __init__(self, leg, motor, markers, load, motorInit, motorMin, motorMax, cutoffFreq, nb_zeros, nb_poles, optimal, proportionalGain, integralGain, derivativeGain):
        super().__init__(leg, motor, markers, load, motorInit, motorMin, motorMax, cutoffFreq)

        # Specific gui setup
        self.guiNode.addData(name="noise", type="float", value=0.)
        self.guiNode.addData(name="reference", type="float", value=0.)
        self.guiNode.addData(name="output", type="float", value=0.)
        self.guiNode.addData(name="controlMode", type="bool", value=ControlMode["Open Loop"])
        MyGui.MyRobotWindow.addSettingInGroup("Reference", self.guiNode.reference, -150, 150, "Control Law")
        MyGui.MyRobotWindow.addSettingInGroup("Noise", self.guiNode.noise, 0, 3, "Control Law")
        MyGui.MyRobotWindow.addSettingInGroup("Control Mode", self.guiNode.controlMode, 0, 1, "Buttons")

        # Plotting data
        MyGui.PlottingWindow.addData("Reference", self.guiNode.reference)
        MyGui.PlottingWindow.addData("Output", self.guiNode.output)

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

        pid_path = os.path.join(data_path, f"pid_{nb_zeros}zeros_{nb_poles}poles.npz")
        pid_data = np.load(pid_path)
        self.setup_additional_variables(pid_data, optimal, proportionalGain, integralGain, derivativeGain)


    def setup_additional_variables(self, pid_data, optimal, proportionalGain, integralGain, derivativeGain):

        # additional states for closed-loop control
        if not optimal:
            self.Kp = proportionalGain
            self.Ki = integralGain
            self.Kd = derivativeGain

        else:
            self.Kp = pid_data["proportionalGain"]
            self.Ki = pid_data["integralGain"]
            self.Kd = pid_data["derivativeGain"]

        self.dt = self.root.dt.value
        self.integral = 0
        self.error_prev = 0
        self.reference = np.zeros((1,))

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
            self.integral += error * self.dt
            derivative = (error - self.error_prev) / self.dt
            self.error_prev = error

            # === Step 2: Implement PID control law ===
            # TODO: Implement PID control law
            desiredMotorPos = self.Kp * error + self.Ki * self.integral + self.Kd * derivative

            self.motor.position.value = desiredMotorPos.flatten()[0]
        else:
            if self.guiNode.active.value:
                desiredMotorPos[0] = self.motor.position.value

        self.command = self.filter(
            desiredMotorPos, self.command,
            cutoffFreq=self.cutoffFreq, samplingFreq=self.samplingFreq)


    def execute_control_at_simu_frame(self):
        super().execute_control_at_simu_frame()
        self.guiNode.output.value = self.markersPos[1, 0]


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
