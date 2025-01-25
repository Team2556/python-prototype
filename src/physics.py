import wpilib
import wpilib.simulation
from wpimath.geometry import Transform2d

from phoenix6.unmanaged import feed_enable
from phoenix6.hardware import TalonFX
import typing
import wpimath.system.plant

import constants
from constants import ElevatorConstants

if typing.TYPE_CHECKING:
    from robot import MyRobot

# import generated.tuner_constants as tc
# from wpimath.units import inchesToMeters, feetToMeters, metersToFeet
# from pyfrc.physics.drivetrains import four_motor_swerve_drivetrain


class PhysicsEngine:
    def __init__(self, physics_controller, robot: "MyRobot"):
        self.physics_controller = physics_controller
        self.robot = robot
        self.container = self.robot.container
        self.drivetrain = self.container.drivetrain
        self.prev_pose = self.drivetrain.get_state().pose
        self.elevatorGearbox = wpimath.system.plant.DCMotor.krakenX60FOC(2)

        """
        :param physics_controller: `pyfrc.physics.core.Physics` object
                                   to communicate simulation effects to
        :param robot: your robot object
        """




        #doesn't work ... temp = self.container.drivetrain.get_module(0).sim_state
        # for each motor in each module in the drivetrain, create a simulated motor and encoder
        # for module in self.drivetrain.modules:
        #     for motor in module:
        #         temp = motor.sim_state

        '''# Front Left
        _front_left_drive_motor_id = 10
        _front_left_steer_motor_id = 11
        _front_left_encoder_id = 12
        _front_left_encoder_offset: units.rotation = 0.45361328125
        _front_left_steer_motor_inverted = False
        _front_left_encoder_inverted = False

        _front_left_x_pos: units.meter = inchesToMeters(11.75)
        _front_left_y_pos: units.meter = inchesToMeters(11.75)

        # Front Right
        _front_right_drive_motor_id = 7
        _front_right_steer_motor_id = 8
        _front_right_encoder_id = 9
        _front_right_encoder_offset: units.rotation = 0.43505859375
        _front_right_steer_motor_inverted = False
        _front_right_encoder_inverted = False

        _front_right_x_pos: units.meter = inchesToMeters(11.75)
        _front_right_y_pos: units.meter = inchesToMeters(-11.75)

        # Back Left
        _back_left_drive_motor_id = 4
        _back_left_steer_motor_id = 5
        _back_left_encoder_id = 6
        _back_left_encoder_offset: units.rotation = 0.187255859375
        _back_left_steer_motor_inverted = False
        _back_left_encoder_inverted = False

        _back_left_x_pos: units.meter = inchesToMeters(-11.75)
        _back_left_y_pos: units.meter = inchesToMeters(11.75)

        # Back Right
        _back_right_drive_motor_id = 1
        _back_right_steer_motor_id = 2
        _back_right_encoder_id = 3
        _back_right_encoder_offset: units.rotation = 0.35498046875
        _back_right_steer_motor_inverted = False
        _back_right_encoder_inverted = False

        _back_right_x_pos: units.meter = inchesToMeters(-11.75)
        _back_right_y_pos: units.meter = inchesToMeters(-11.75)

        #simulated motors and encoders
        self.sim_front_left_drive = TalonFX(_front_left_drive_motor_id).sim_state
        self.sim_front_left_drive.set_supply_voltage(12.0)
        self.sim_front_left_steer = TalonFX(_front_left_steer_motor_id).sim_state
        self.sim_front_left_steer.set_supply_voltage(12.0)
        # sim_front_left_encoder = CANCoder(hardware._front_left_encoder_id).sim_state

        self.sim_front_right_drive = TalonFX(_front_right_drive_motor_id).sim_state
        self.sim_front_right_steer = TalonFX(_front_right_steer_motor_id).sim_state
        # sim_front_right_encoder = CANCoder(_front_right_encoder_id).sim_state

        self.sim_back_left_drive = TalonFX(_back_left_drive_motor_id).sim_state
        self.sim_back_left_steer = TalonFX(_back_left_steer_motor_id).sim_state
        # sim_back_left_encoder = CANCoder(_back_left_encoder_id).sim_state

        self.sim_back_right_drive = TalonFX(_back_right_drive_motor_id).sim_state
        self.sim_back_right_steer = TalonFX(_back_right_steer_motor_id).sim_state
        # sim_back_right_encoder = CANCoder(_back_right_encoder_id).sim_state

        swerve_kinematics = SwerveDrive4Kinematics(Translation2d(_front_left_x_pos, _front_left_y_pos),
                                                   Translation2d(_front_right_x_pos, _front_right_y_pos),
                                                   Translation2d(_back_left_x_pos, _back_left_y_pos),
                                                   Translation2d(_back_right_x_pos, _back_right_y_pos))
        
        self.swerve_drivetrain_speeds = four_motor_swerve_drivetrain(
            self.sim_front_left_drive, sim_front_left_steer,
            sim_front_right_drive, sim_front_right_steer,
            sim_back_left_drive, sim_back_left_steer,
            sim_back_right_drive, sim_back_right_steer,
            swerve_kinematics, 0.381, 0.381, 0.381, 0.381
        )'''
        self.elevator = constants.ElevatorConstants(
            self.elevatorGearbox,
            ElevatorConstants.kElevatorGearing,
            ElevatorConstants.kCarriageMass,
            ElevatorConstants.kElevatorDrumRadius,
            ElevatorConstants.kMinElevatorHeight,
            ElevatorConstants.kMaxElevatorHeight,
            True,
            0,
            [0.01, 0.0],
        )
        self.elevencoder = wpilib.Encoder(robot.elevencoder)
        self.leftelevmotor = TalonFX(robot.leftelevmotor.get())
        self.rightelevmotor = TalonFX(robot.rightelevmotor.get())
        # Create a Mechanism2d display of an elevator
        self.mech2d = wpilib.Mechanism2d(20, 50)
        self.elevatorRoot = self.mech2d.getRoot("Elevator Root", 10, 0)
        self.elevatorMech2d = self.elevatorRoot.appendLigament(
            "Elevator", self.elevator.getPositionInches(), 90
        )

        # Put Mechanism to SmartDashboard
        wpilib.SmartDashboard.putData("Elevator", self.mech2d)

            
        
    def update_sim(self, now, tm_diff):
        # Currently, the Python API for CTRE doesn't automatically detect the the
        # Sim driverstation status and enable the signals. So, for now, manually
        # feed the enable signal for double the set robot period.
        feed_enable(0.020 * 2)

        #refer to https://api.ctr-electronics.com/phoenix6/release/python/autoapi/phoenix6/sim/talon_fx_sim_state/index.html

        # drivetrainKinematics = SwerveDrive4Kinematics([module.pose for module in self.drivetrain.modules])
        # self.physics_controller.setPose(self.drivetrain.pose)
        # print("------------------------------------------------\n",f'{self.sim_front_left_drive.motor_voltage=}',
        #       "------------------------------------------------\n",f'{self.sim_front_left_steer.motor_voltage=}/n/n')
        # for module in self.drivetrain.modules:
        #     for module_locations in module.module_locations:
        #         print("------------------------------------------------\n",f'{module}at{module_locations=}',
        #         "------------------------------------------------\n")
        # print("------------------------------------------------\n",f'{feetToMeters(self.drivetrain.get_state().speeds)=}',"------------------------------------------------\n",f'{(self.drivetrain.get_state().speeds)=}')  
        #may need unit conversion, may not..? metersToFeet or feetToMeters
        # self.physics_controller.drive(-(self.drivetrain.get_state().speeds), 0.020)
        # self.physics_controller.drive(-(self.drivetrain.get_state().speeds), 0.020)
        self.elevator.setInput(
            0, self.leftelevmotor.getSpeed() * wpilib.RobotController.getInputVoltage()
        )
        self.elevator.setInput(
            1, self.rightelevmotor.getSpeed() * wpilib.RobotController.getInputVoltage()
        )

        # Next, we update it
        self.elevator.update(tm_diff)

        # Finally, we set our simulated encoder's readings and simulated battery
        # voltage
        self.elevencoder.getDistance(self.elevator.getPosition())
        # SimBattery estimates loaded battery voltage
        # wpilib.simulation.RoboRioSim.setVInVoltage(
        #     wpilib.simulation.BatterySim
        # )

        # Update the Elevator length based on the simulated elevator height
        self.elevatorMech2d.setLength(self.elevator.getPositionInches())

        do_nothing = True
        if do_nothing:
            pass
        else:
            self.physics_controller.move_elevator(Transform2d(self.prev_pose, self.drivetrain.get_state().pose))
            self.prev_pose = self.drivetrain.get_state().pose

            

