import wpilib
from wpilib import SmartDashboard
import wpilib.simulation
from wpimath.kinematics import SwerveDrive4Kinematics
from wpimath.geometry import Pose2d, Rotation2d, Translation2d, Transform2d, Pose3d, Rotation3d, Transform3d
from wpimath.units import degrees, radians, degreesToRadians, radiansToDegrees, inchesToMeters, inches, meters, feetToMeters, metersToFeet
import robotpy_apriltag as apriltag
import ntcore
from photonlibpy import PhotonCamera
from photonlibpy.simulation import VisionSystemSim, VisionTargetSim, PhotonCameraSim
from constants import AprilTags, AprilTagField
# TODO: add the camera location to constants and import


from phoenix6.unmanaged import feed_enable
from phoenix6.hardware import TalonFX
# import generated.tuner_constants as tc
# from wpimath.units import inchesToMeters, feetToMeters, metersToFeet
# from pyfrc.physics.drivetrains import four_motor_swerve_drivetrain


class PhysicsEngine:
    def __init__(self, physics_controller, robot: "RobotContainer"):
        self.physics_controller = physics_controller
        self.robot = robot
        self.container = self.robot.container
        self.drivetrain = self.container.drivetrain
        self.prev_pose = self.drivetrain.get_state().pose

        self.field = self.robot.field #wpilib.Field2d()#already put a field on smartdashboard from robot or container..
        # SmartDashboard.putData("Field-Vision", self.field)

        #init photon camera
        self.photon_camera_left = PhotonCamera('photonvision_left')
        self.photon_camera_right = PhotonCamera('photonvision_right')
        #setup photon camera physical location on robot
        #init simvision 
        self.photon_camera_sim_left =PhotonCameraSim(self.photon_camera_left)
        self.photon_camera_sim_right =PhotonCameraSim(self.photon_camera_right)
        self.photon_camera_sim_left.setMaxSightRange(4)#meters
        self.photon_camera_sim_right.setMaxSightRange(4)#meters
        self.sim_vision = VisionSystemSim("SimPhoton_system")
        self.sim_vision.addCamera(self.photon_camera_sim_left, Transform3d(x=inchesToMeters(4), 
                                                                           y=inchesToMeters((28/2)-2), 
                                                                           z=inchesToMeters(8), 
                                                                           rotation=Rotation3d(0, 0, degreesToRadians(90))))
        self.sim_vision.addCamera(self.photon_camera_sim_right, Transform3d(x=inchesToMeters(4), 
                                                                            y=inchesToMeters(-((28/2)-2)), 
                                                                            z=inchesToMeters(8), 
                                                                            rotation=Rotation3d(0, 0, degreesToRadians(-90))))
        self.sim_vision.addAprilTags(AprilTagField)

        
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

            
        
    def update_sim(self, now, tm_diff):
        # Currently, the Python API for CTRE doesn't automatically detect the the
        # Sim driverstation status and enable the signals. So, for now, manually
        # feed the enable signal for double the set robot period.
        feed_enable(0.020 * 2)

        robot_pose = self.drivetrain.get_state().pose
        self.field.setRobotPose(robot_pose) 
        # self.photon_camera_sim.  .setRobotPose(robot_pose)
        self.sim_vision.update(robotPose=robot_pose)


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
        do_nothing = True
        if do_nothing:
            pass
        else:
            self.physics_controller.move_robot(Transform2d(self.prev_pose, self.drivetrain.get_state().pose))
            self.prev_pose = self.drivetrain.get_state().pose

            

