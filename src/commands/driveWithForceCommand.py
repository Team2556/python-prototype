

import wpilib
from commands2 import Command
from wpimath.geometry import Pose2d, Translation2d
from wpimath.kinematics import ChassisSpeeds
from subsystems.command_swerve_drivetrain import CommandSwerveDrivetrain
# from robotcontainer import drivetrain  # Replace with your actual drivetrain subsystem name

class DriveToPointForce(Command):
    def __init__(self, drivetrain:CommandSwerveDrivetrain, target: Pose2d, obstacle: Translation2d):
        super().__init__()
        # Store inputs
        self.drivetrain = drivetrain
        self.target = target
        self.obstacle = obstacle
        # Tuning constants
        self.obstacle_radius = 0.95  # meters (size of obstacle)
        self.influence_radius = 1.25  # meters (range of repulsive force)
        self.k_attr = 0.5  # Attractive force gain
        self.k_rep = 2.0   # Repulsive force gain
        self.max_speed = 4.0  # m/s (maximum speed of robot)
        self.tolerance = 0.1  # meters (how close to target to stop)

        # Require the drivetrain to prevent other commands from using it
        self.addRequirements(drivetrain)

    def execute(self):
        # Get current robot position from odometry
        current_pose: Pose2d = self.drivetrain.get_state().pose  # Ensure your drivetrain has this method
        robot_pos: Translation2d = current_pose.translation()
        target_pos: Translation2d = self.target.translation()

        # Calculate attractive force (pull toward target)
        # print(f'robot_pos: {robot_pos}  target_pos: {target_pos}, obstacle: {self.obstacle}')
        f_attr =  target_pos - robot_pos
        f_attr = Translation2d(f_attr.x * self.k_attr, f_attr.y * self.k_attr)

        # Calculate repulsive force (push away from obstacle)
        delta_obstacle = robot_pos - self.obstacle
        dist = delta_obstacle.norm() 
        print(f'dist: {dist} {dist < self.influence_radius=} Is this correct???/n===/n===/n===')
        f_rep = Translation2d(0, 0)  # Default: no repulsion
        if dist < self.influence_radius:
            # Repulsive force scales inversely with distance
            magnitude = self.k_rep * (1 / dist - 1 / self.influence_radius)
            f_rep = delta_obstacle * (magnitude / dist)

        # Combine forces
        f_total = f_attr + f_rep
        norm = f_total.norm()
        print(f'/nf_total: {f_total} norm: {norm} --/n--/n')
        if norm > 0:
            # Normalize and scale to max speed
            velocity = f_total * (self.max_speed / norm)
            print(f'/nf_total: {f_total} norm: {norm} 000--/n--{velocity}/n-00-{velocity.x=}  {velocity.y=}/n')
            # Convert to chassis speeds (no rotation for simplicity)
            chassis_speeds = ChassisSpeeds(velocity.x, velocity.y, 0.0)
            # Send speeds to drivetrain
            # self.drivetrain.set_speeds(chassis_speeds)  # Adjust method name if different
            self.drivetrain.apply_request(
                lambda: (
                    self._drive.with_velocity_x(
                        1000*velocity.x)
                    .with_velocity_y(
                        -1000*velocity.y)
                )
            )
                    # shift the center of rotation to opposite front corner, if the driver pulls down on the right stick in addition to the side.
                    # This should allow some nice defensive roll-off maneuvers
                # )
            

    def isFinished(self):
        # End when close enough to the target
        current_pose = self.drivetrain.get_state().pose
        distance = current_pose.translation().distance(self.target.translation())
        return distance < self.tolerance

    def end(self, interrupted: bool):
        # Stop the robot when the command finishes or is interrupted
        # self.drivetrain.set_speeds(ChassisSpeeds(0, 0, 0))
        
        self.drivetrain.apply_request(
            lambda: (
                self._drive.with_velocity_x(0)
                .with_velocity_y(0)
                .with_rotational_rate(0)
            )
        )