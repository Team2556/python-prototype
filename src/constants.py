import pheonix6
import numpy

VISION_DES_ANGLE_deg = 25
VISION_TURN_kP = 0
VISION_STRAFE_kP = 26
CAM_MOUNT_HEIGHT = 12
CAM_MOUNT_PITCH = 25

class AprilTags_height:
    def tag_heights(): #height of apriltags by order of number, in centimeters
        heights = numpy.array([135, 135, 117, 178, 178, 17, 17, 17, 17, 17, 17, 135, 135, 117, 178, 178, 17, 17, 17, 17, 17, 17])
        return heights