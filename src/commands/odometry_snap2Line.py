from commands2 import Command
from wpimath.geometry import Translation2d, Pose2d, Translation3d, Pose3d, Rotation2d, Transform2d, Transform3d, Rotation3d
from wpimath.units import degrees, radians, degreesToRadians, radiansToDegrees
from constants import AprilTagField, AprilTags


import numpy as np
from phoenix6.utils import get_current_time_seconds
class SnapToLineCommand(Command):
    '''This command snaps the robot to the closest point on a line segment.
    The line segments will be the hard walls that the robot could potentially push against.'''
    def __init__(self, drivetrain, segments: list[tuple[Translation2d, Translation2d]]=[(Translation2d(2,0), Translation2d(0,2)), (Translation2d(0,10), Translation2d(2,12)),
            (Translation2d(10,0), Translation2d(10,10))]):
        super().__init__()
        self.drivetrain = drivetrain
        self.addRequirements(drivetrain)
        self.segments = segments #TODO: make into real segments, import form constants(could use apriltaglayout to help generate)

    def execute(self):
        current_pose = self.drivetrain.get_state_copy().pose
        offset_segments = [self.rightHand_offset_segment(start, end, 0.1, 0.1) for start, end in self.segments]
        print(f'offset_segments: {offset_segments}')
        (trust_snap, snap_dist, closest_point) = self.get_closest_point_on_segments(current_pose.translation(), segments=self.segments)
        #TODO: add way to address the rotation of the robot or use self.drivetrain.reset_translation()
        if trust_snap:
            # self.drivetrain.reset_translation(closest_point)
            self.drivetrain.add_vision_measurement(Pose2d(closest_point, current_pose.rotation()), timestamp=get_current_time_seconds(), vision_measurement_std_devs=(snap_dist, snap_dist, 3.14))
            '''add_vision_measurement(vision_robot_pose: Pose2d, timestamp: phoenix6.units.second, vision_measurement_std_devs: tuple[float, float, float] | None = None)'''
            ''':param vision_measurement_std_devs: Standard deviations of the vision pose
                                            measurement (x position in meters, y
                                            position in meters, and heading in radians).
                                            Increase these numbers to trust the vision
                                            pose measurement less.'''
        else:
            print(f'Not trusting snap, snap_dist: {snap_dist} \n-\n-\n-\n--------------------------------')
            #TODO add feedback to the driver that the snap was not trusted
        # closest_pose = Pose2d(closest_point, current_pose.rotation())
        # self.drivetrain.reset_pose(closest_pose)

    def get_closest_point_on_segments(self, point: Translation2d, segments: list) -> tuple[bool, float, Translation2d]:
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

    
    def rightHand_offset_segment(self, start: Translation2d, end: Translation2d, offset: float, shorten: float) -> tuple[Translation2d, Translation2d]:
        line_vec = end - start
        line_len = line_vec.norm()
        line_unitvec = line_vec / line_len
        # print(f'{line_unitvec=}')
        perp_unitvec = line_unitvec.rotateBy(Rotation2d(degreesToRadians(-90)))
        # print(f'{perp_unitvec=}')
        offset_vec = perp_unitvec * offset
        shorten_vec = line_unitvec * shorten
        new_start = start + offset_vec + shorten_vec
        new_end = end + offset_vec - shorten_vec
        return (new_start, new_end)



    


    def isFinished(self):
        return True

    def end(self, somthingelse):
        (print(f' ------------ \n        ----------------\n        {somthingelse=} \n        ???????????????\n        ????????????????????????\n'))
        pass

    def interrupted(self):
        self.end()