from adafruit_servokit import ServoKit
from time import sleep
from Calculation import getAngle
from Vector import VectorCalculation
import numpy as np
import math
import keyboard

#Function to set angles for all PWM channels
def setAngle(angle, channel, kit):

    kit.servo[channel].angle = angle

def main():
    kit = ServoKit(channels=16)
    
    #Calibrated each pwm width range to ensure 0-180 degree movement. If using other servos this may need to be changed
    kit.servo[0].set_pulse_width_range(600, 2450)
    kit.servo[1].set_pulse_width_range(600, 2500)
    kit.servo[2].set_pulse_width_range(550, 2400)
    kit.servo[3].set_pulse_width_range(600, 2400)
    kit.servo[4].set_pulse_width_range(600, 2400)
    kit.servo[5].set_pulse_width_range(600, 2500)

    #Sets initial position
    home=np.array([0, 0, 280])
    translation = home
    yaw = 0
    roll = 0
    pitch = 0

    # Angle for each servo with regard to X axis, do not change unless altering the base
    beta = [0, math.pi, math.pi*2/3, -math.pi/3, -math.pi*2/3, math.pi / 3]

    # Points of platform, do not change unless altering the platform
    p = np.array([[35.07, 24.75, 7.55],
                  [3.9, 42.75, 7.55],
                  [-38.97, 18, 7.55],
                  [-38.97, -18, 7.55],
                  [3.9, -42.75, 7.55],
                  [35.07, -24.75, 7.55]])

    # Points of base, do not change unless altering the base
    servo_positions = np.array([[42.5, 73.24, 60.75],
                                [-42.5, 73.24, 60.75],
                                [-86.68, 0.18, 60.75],
                                [-42.18, -73.43, 60.75],
                                [42.18, 73.43, 60.75],
                                [84.68, 0.18, 60.75]])
    
    #Initialize dictionary of servo objects
    servoDict={}
    for i in range(0,6):
        servoDict[i]=VectorCalculation(p[i], beta[i], servo_positions[i])
    
    counter=0

    """Loop of movement, incremental change of translation and rotation gives new angles in getAngle()
    and sends PWM signals through setAngle(). To change movement alter translation in line 73"""
    try:
        while True:
            
            counter+=1

            #Gets list of alpha angles
            alpha = getAngle(translation, p, servoDict, yaw, roll, pitch)

            #Sets angles for servos, 180 - angle for odd servos to mirror movement.
            for i in range(len(alpha)):
                if (i % 2) != 0:
                    alpha[i]=180-alpha[i]
                #print(alpha[i])
                setAngle(alpha[i], i, kit)

            sleep(0.01)
            
            """Incremental change of pitch yaw and roll. Change these to change rotation (Use counter for incremental change)
            A pitch or yaw higher than 20 degrees or roll higher than 45 degrees will most likley exceed maximum angles."""
            yaw = 0
            roll = 0
            pitch = 0
            rotations=([yaw, roll, pitch])

            """Incremental change of x, y and z coordinates. Change these to change path of the platform. To avoid maximum angles
            keep x and y within +/- 20 and z within 300 +/- 20"""
            translation = home + np.array([home, 20*math.cos(math.radians(counter-math.pi/2)) , 20*math.sin(math.radians(counter-math.pi/2))])

            #Cancel movement by pressing s, moves the platform to home position
            if keyboard.is_pressed("s"):
                raise Exception

    except Exception as e:

        while not np.array_equal(translation, home):
            print(e)

            #Moves coordinates incrementally back to home position
            #For z-coordinate
            if translation[2] > home[2]+1:
                z=-1
            elif translation[2] < home[2]-1:
                z=1
            else:
                translation[2]=home[2]
                z=0
            
            #For x and y coordinate
            direction = np.sign(translation)
            incrementXY = np.where(translation[:2] == 0, 0, -direction[:2])
            increment = np.concatenate((incrementXY, [z]))
            
            translation += increment
            #Check if any x or y distance is close to zero, if so sets this to 0
            minimumDistance = np.where(abs(translation) < 1)
            translation[minimumDistance]=0

            #Moves rotations back to zero
            rotations=np.array([yaw, roll, pitch])
            lessThanOne = np.where(abs(rotations) < 0.05)
            rotations[lessThanOne]=0
            

            angularChange = np.sign(rotations)
            change = np.where(rotations == 0, 0,-angularChange*0.01)
            change = change.astype(rotations.dtype)

            rotations += change

            #Updates rotations
            yaw= rotations[0]
            roll=rotations[1]
            pitch=rotations[2]

            #Calculates new angles to move to home position
            alpha = getAngle(translation, p, servoDict, yaw, roll, pitch)

            #Sends information to servos
            for i in range(len(alpha)):
                if (i % 2) != 0:
                    alpha[i]=180-alpha[i]
                setAngle(alpha[i], i, kit)

            sleep(0.01)

if __name__ == "__main__":
    main()
