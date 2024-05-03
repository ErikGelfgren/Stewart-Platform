from Vector import VectorCalculation
import numpy as np
import math

def rotationMatrix(gamma, phi, theta):

    #Rotational matrix, do not change this    
    rx = np.array([math.cos(gamma)*math.cos(theta), -math.sin(gamma)*math.cos(phi)+math.cos(gamma)*math.sin(theta)*math.sin(phi), math.sin(gamma)*math.sin(phi)+math.cos(gamma)*math.sin(theta)*math.cos(phi)])
    ry = np.array([math.sin(gamma)*math.cos(theta), math.cos(gamma)*math.cos(phi)+math.sin(gamma)*math.sin(theta)*math.sin(phi), -math.cos(gamma)*math.sin(phi)+math.sin(gamma)*math.sin(theta)*math.cos(phi)])
    rz = np.array([-math.sin(theta), math.cos(theta)*math.sin(phi), math.cos(theta)*math.cos(phi)])

    return np.array([rx, ry, rz])

def getAngle(translation, servo, yaw=0, roll=0, pitch=0):
    """Insert desired translation vector, pitch, yaw and roll, returns needed angles for all 6 servos.
    Ensure that all parameters are predefined"""

    #Rotational matrix, yaw (z-axis), roll (x-axis), pitch (y-axis)
    rotation=rotationMatrix(yaw,roll,pitch)

    #Initialize each object (servo) return and append each servos required angle in degreeAngle list
    degreeAngle=[]

    for i in range(0,6):
        degreeAngle.append(servo[i].calcAngle(rotation, translation))
    
    return(degreeAngle)
