import wpilib
from commands2 import Command, Subsystem
from commands2.sysid import SysIdRoutine
import math
from phoenix6 import SignalLogger, swerve, units, utils
from typing import Callable, overload
from wpilib import DriverStation, Notifier, RobotController
from wpilib.sysid import SysIdRoutineLog
from wpimath.geometry import Rotation2d
import photonlibpy
from constants import Apritag_heights
from src.constants import CAM_MOUNT_HEIGHT

class PhotonCommand(wpilib.TimedRobot):
    def robotInit(self) -> None:
        self.controller = wpilib.XboxController(0)
        self.cam = PhotonCamera("photonvision") # type: ignore

    def robotPeriodic(self) -> None:
        self.swerve.updateOdometry()
        self.swerve.log()

    def teleopPeriodic(self) -> None:
        # Get information from the camera
        targetYaw = 0.0
        targetRange = 0.0
        targetVisible = False
        results = self.cam.getAllUnreadResults()
        if len(results) > 0:
            result = results[-1]  # take the most recent result the camera had
            # At least one apriltag was seen by the camera
            for target in result.getTargets():
                if target.getFiducialId() == 7:
                    # Found tag 7, record its information
                    targetVisible = True
                    targetYaw = target.getYaw()
                    heightDelta = CAM_MOUNT_HEIGHT - tag_heights(7)
                    angleDelta = math.radians(CAM_MOUNT_PITCH_deg - target.getPitch())
                    targetRange = heightDelta / math.tan(angleDelta)