#lifted from https://github.com/LimelightVision/limelightlib-python/tree/main

# look into making a subsystem out of this. Or just use ll imported.

# results specification: https://docs.limelightvision.io/docs/docs-limelight/apis/json-results-specification
# basic use: https://docs.limelightvision.io/docs/docs-limelight/apis/complete-networktables-api

# command based... floating subsystem.



import limelight
import limelightresults
# import limelightHelpers (only in Java/C++ so far; could write it up)
import json
import time
from commands2 import Subsystem
from wpimath.geometry import Pose2d, Transform2d, Translation2d, Rotation2d
from wpimath.units import degreesToRadians
import numpy as np
from phoenix6.utils import get_current_time_seconds

class LimelightSubsystem(Subsystem):
    def __init__(self):
        super().__init__()
        self.discovered_limelights = limelight.discover_limelights()#debug=True)
        print("discovered limelights:", self.discovered_limelights)

        if self.discovered_limelights:
            limelight_address = self.discovered_limelights[0] 
            self.ll = limelight.Limelight(limelight_address)
            results = self.ll.get_results()
            status = self.ll.get_status()
            print("-----")
            print("targeting results:", results)
            print("-----")
            print("status:", status)
            print("-----")
            print("temp:", self.ll.get_temp())
            print("-----")
            print("name:", self.ll.get_name())
            print("-----")
            print("fps:", self.ll.get_fps())
            print("-----")
            print("hwreport:", self.ll.hw_report())

            self.ll.enable_websocket()
            print(self.ll.get_pipeline_atindex(0))
            

    #update python (on limelight) inputs
    def update_python_inputs(self, inputs):
        return self.ll.update_python_inputs(inputs)

    def get_results(self):
        return self.ll.get_results()
    
    # get latest results
    def get_latest_results(self):
        return self.ll.get_latest_results()

    def get_status(self):
        return self.ll.get_status()

    def get_temp(self):
        return self.ll.get_temp()

    def get_name(self):
        return self.ll.get_name()

    def get_fps(self):
        return self.ll.get_fps()

    def hw_report(self):
        return self.ll.hw_report()

    def get_pipeline_atindex(self, index):
        return self.ll.get_pipeline_atindex(index)

    def update_pipeline(self, pipeline_update, flush):
        return self.ll.update_pipeline(pipeline_update, flush)

    def pipeline_switch(self, index):
        return self.ll.pipeline_switch(index)

    def update_python_inputs(self, inputs):
        return self.ll.update_python_inputs(inputs)

    def get_latest_results(self):
        return self.ll.get_latest_results()

    def disable_websocket(self):
        return self.ll.disable_websocket()

    def enable_websocket(self):
        return self.ll.enable_websocket()
    
    #section specialty utilities
    def trust_target(self, robot_pose: Pose2d, residual_threshold=1000000000, override_and_trust=False): #TODO: fix threshold
        #calculate distance to the detected target
        #will the results hav only one target's results? assume one for now
        # Done: check unit consistency: botpose is in meters, robot_pose is in meters
        trust_vision_data = True
        if self.discovered_limelights:
            latest_parsed_result = limelightresults.parse_results(self.ll.get_latest_results())
            self.ll 
            detector_confidences = [detector.confidence for detector in latest_parsed_result.detectorResults]
            print("detector confidences:", detector_confidences)
            if latest_parsed_result:
                validity = latest_parsed_result.validity #what units are validity in ?
                trust_vision_data *= (validity==1) 
                vision_bot_pose_to_use = latest_parsed_result.botpose_wpiblue #botpose

                delta_x  = vision_bot_pose_to_use[0] - robot_pose.x
                delta_y = vision_bot_pose_to_use[1] - robot_pose.y
                residual = np.sqrt(delta_x**2 + delta_y**2)
                latency = latest_parsed_result.targeting_latency
                time_of_measurement = get_current_time_seconds() - .001 * latency #only acounting for json unpacking 
 
                trust_vision_data *= (residual < residual_threshold) 
                '''General results do not have stddev -- need to use MegaTag2
                if latest_parsed_result.stddevs:
                    detect_stdDev = latest_parsed_result.stddevs
                    detect_stdDev_x = detect_stdDev[0]
                    detect_stdDev_y = detect_stdDev[1]
                    max_stdDev = max(detect_stdDev_x, detect_stdDev_y)
                    trust_vision_data *= (residual < 2 * max_stdDev) 
                    #if the residual is within 2 standard deviations, 
                    # and the residual is less than 1 meter, 
                    '''
                # trust the target
                detect_stdDev_x = None
                detect_stdDev_y = None
                trust_telemetry = [validity, residual, detect_stdDev_x, detect_stdDev_y]
                # print(f"trust_telemetry:{trust_vision_data} -> {trust_telemetry}     IIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIII")
                
                if override_and_trust: trust_vision_data = True
                viz_pose = Pose2d(Translation2d(vision_bot_pose_to_use[0], vision_bot_pose_to_use[1]),
                                    Rotation2d(degreesToRadians(vision_bot_pose_to_use[5])))
        else:
            trust_vision_data = False
            viz_pose = None
            latest_parsed_result= None
            time_of_measurement = get_current_time_seconds()

        return (trust_vision_data , viz_pose, latest_parsed_result, time_of_measurement)


    #endsection specialty utilities

    def periodic(self):
        if self.discovered_limelights:
            try:
                result = self.ll.get_latest_results()
                _parsed_result = limelightresults.parse_results(result)
                if _parsed_result is not None:
                    print("valid targets: ", _parsed_result.validity, ", pipelineIndex: ", _parsed_result.pipeline_id,", Targeting Latency: ", _parsed_result.targeting_latency)
                    self.parsed_result = _parsed_result

            except KeyboardInterrupt:
                print("Program interrupted by user, shutting down.")


    def end(self):
        if self.discovered_limelights: self.ll.disable_websocket()
    