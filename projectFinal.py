print("loading libraries for color tracking...")
import argparse         # For fetching user arguments
import numpy as np      # Kernel

print("loading rcpy.")
import rcpy                 # Import rcpy library
import rcpy.motor as motor  # Import rcpy motor module
print("finished loading libraries.")

print("loading scuttle libraries")
import L2_track_target as trackTarget
import L2_log as log

print("finished loading scuttle libraries")


color_range = ((0, 75, 0), (5, 255, 255))

def trackingDrive():
    
    width  = 240   # Resized image width. This is the image width in pixels.
    size_h = 160	# Resized image height. This is the image height in pixels.
    
    tc = 40     # Too Close     - Maximum pixel size of object to track
    tf = 15      # Too Far       - Minimum pixel size of object to track
    tp = 25     # Target Pixels - Target size of object to track

    band = 25   #range of x considered to be centered

    x = 0  # will describe target location left to right
    y = 0  # will describe target location bottom to top

    radius = 0  # estimates the radius of the detected target

    duty_l = 0 # initialize motor with zero duty cycle
    duty_r = 0 # initialize motor with zero duty cycle
    
    print("initializing rcpy...")
    rcpy.set_state(rcpy.RUNNING)        # initialize rcpy
    print("finished initializing rcpy.")

    try:

        while rcpy.get_state() != rcpy.EXITING:

            if rcpy.get_state() == rcpy.RUNNING:

                scale_t = 1 	# a scaling factor for speeds
                scale_d = 1.3	# a scaling factor for speeds

                motor_r = 2 	# Right Motor assigned to #2
                motor_l = 1 	# Left Motor assigned to #1

                x, y, radius = trackTarget.colorTarget(color_range)    # Get properties of circle around shape

                if x != None:   # If more than 0 closed shapes exist

                    
                    

                    # handle centered condition
                    if x > ((width/2)-(band/2)) and x < ((width/2)+(band/2)):       # If center point is centered

                        dir = "driving"

                        if (radius-tp) >= 1:    # Too Close

                            case = "too close"

                            duty = -1 * ((radius-tp)/(tc-tp))
                            dutyF = 0


                        elif (radius - tp) < -1:   # Too Far

                            case = "too far"

                            duty = 1 - ((radius - tf)/(tp - tf))
                            duty = scale_d * duty
                            dutyF = 0
                        
                        else:
                            case = "good"
                            duty = 0
                            dutyF = -0.5

                        duty_r = duty
                        duty_l = duty
                        
                        

                    else:
                        case = "turning"
                        
                        dutyF = 0

                        duty_l = round((x-0.5*width)/(0.5*width),2)     # Duty Left
                        duty_l = duty_l*scale_t

                        duty_r = round((0.5*width-x)/(0.5*width),2)     # Duty Right
                        duty_r = duty_r*scale_t

                    # Keep duty cycle within range

                    if duty_r > 1:
                        duty_r = 1

                    elif duty_r < -1:
                        duty_r = -1

                    if duty_l > 1:
                        duty_l = 1

                    elif duty_l < -1:
                        duty_l = -1
                        
                    if duty_r < .3 and duty_r > 0:
                        duty_r = .3

                    elif duty_r < 0 and duty_r > -.3:
                        duty_r = -.3

                    if duty_l < .3 and duty_l > 0:
                        duty_l = .3

                    elif duty_l < 0 and duty_l > -0.3:
                        duty_l = -0.3

                    # Round duty cycles
                    duty_l = round(duty_l,2)
                    duty_r = round(duty_r,2)

                    print(case, "\tradius: ", round(radius,1), "\tx: ", round(x,0), "\t\tL: ", duty_l, "\tR: ", duty_r)

                    # Set motor duty cycles
                    motor.set(motor_l, duty_l)
                    motor.set(motor_r, duty_r)
                    motor.set(3, dutyF)
                    
                    log.stringTmpFile(str(duty_l),"leftMotor.txt")
                    log.stringTmpFile(str(duty_r),"rightMotor.txt")
                    log.stringTmpFile(str(radius),"radiusOfComputerVision.txt")
                    log.stringTmpFile(str(x),"xValue.txt")


                # Set motor duty cycles
                motor.set(motor_l, duty_l)
                motor.set(motor_r, duty_r)
                motor.set(3, dutyF)
                print(duty_r)

            elif rcpy.get_state() == rcpy.PAUSED:
                pass

    except KeyboardInterrupt: # condition added to catch a "Ctrl-C" event and exit cleanly
        rcpy.set_state(rcpy.EXITING)
        pass

    finally:

    	rcpy.set_state(rcpy.EXITING)
    	print("Exiting Color Tracking.")
    	
trackingDrive()