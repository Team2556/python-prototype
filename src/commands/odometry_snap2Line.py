from commands2 import Command
from wpimath.geometry import Translation2d, Pose2d, Translation3d, Pose3d, Rotation2d, Transform2d, Transform3d, Rotation3d
from wpimath.units import degrees, radians, degreesToRadians, radiansToDegrees
import limelight
import limelightresults
from phoenix6 import swerve
# from phoenix6.swerve.utility import phoenix_pid_controller,geometry,kinematics
import numpy as np

class SnapToLineCommand(Command):
    '''This command snaps the robot to the closest point on a line segment.
    The line segments will be the hard walls that the robot could potentially push against.'''
    def __init__(self, drivetrain, line_start: Translation2d, line_end: Translation2d):
        super().__init__()
        self.drivetrain = drivetrain
        self.line_start = line_start
        self.line_end = line_end
        self.addRequirements(drivetrain)
        self.segments = [(Translation2d(1,1), Translation2d(0,0)), (Translation2d(0,0), Translation2d(1,0)),
            (Translation2d(1,0), Translation2d(1,1)), (Translation2d(1,1), Translation2d(0,1))] #TODO: make into real segments, import form constants(could use apriltaglayout to help generate)

    def execute(self):
        current_pose = self.drivetrain.get_state_copy().pose2d
        (trust_snap, snap_dist, closest_point) = self.get_closest_point_on_segments(current_pose.translation(), segments=self.segments)
        #TODO: add way to address the rotation of the robot or use self.drivetrain.reset_translation()
        if trust_snap:
            self.drivetrain.reset_translation(closest_point)
        else:
            pass #TODO add feedback to the driver that the snap was not trusted
        # closest_pose = Pose2d(closest_point, current_pose.rotation())
        # self.drivetrain.reset_pose(closest_pose)

    def get_closest_point_on_segments(self, point: Translation2d, segments: list) -> Translation2d:
        min_snap_dist = float('inf')
        closest_point = None
        for start, end in segments:
            line_vec = end - start
            point_vec = point - start
            line_len = line_vec.norm()
            line_unitvec = line_vec / line_len
            projection_length = np.dot(point_vec.toVector(), line_unitvec.toVector())
            projection_length = max(0, min(line_len, projection_length))
            candidate_point = start + line_unitvec * projection_length
            snap_dist = (point - candidate_point).norm()
            if snap_dist < min_snap_dist:
                min_snap_dist = snap_dist
                closest_point = candidate_point
        snap_trust_limit = 1.0
        trust_snap = min_snap_dist < snap_trust_limit
        return (trust_snap, min_snap_dist, closest_point)
    


    


    def isFinished(self):
        return True

    def end(self):
        pass

    def interrupted(self):
        self.end()