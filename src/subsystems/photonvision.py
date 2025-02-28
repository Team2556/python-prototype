import cv2
import numpy as np
from limelight import Limelight
import time   
import  ntcore

# Replace '10.TE.AM.11' with the actual IP address of your Limelight
limelight = Limelight(address='10.TE.AM.11')

while True:
    def Results():
    # Get the latest results from the Limelight
        results = limelight.get_results()

        # Check if a target is valid
        if results.target_valid:
            # Print target information
            print("Target X:", results.target_x)
            print("Target Y:", results.target_y)
            print("Target Area:", results.target_area)
        else:
            print("No target found")

    time.sleep(0.1)

    def center_Robot():
        Results()
        result= ntcore.NetworkTable.getTable("limelight").getNumber('tx');
        centerVal = result/ 2
        if centerVal >= 360:


            #pull results
            #divide tX by 2, 
            #If greater than half of camera space, turn to left until within 15 pixels
            #If less than half, turn to the right until within 15 pixels