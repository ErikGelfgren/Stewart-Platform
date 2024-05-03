import numpy as np
import math

class VectorCalculation:
    """Class for each servos angular calculation, contains all predefined constants"""
    def __init__(self, p, beta, B):

        #B and P Vectors
        self.B=B
        self.p=p

        #Points of servo on base
        self.xb=self.B[0]
        self.yb=self.B[1]
        self.zb=self.B[2]

        #Length of pivotal arm on servo, change this if changing the length of pivotal arm
        self.a=30

        #Length of rod between base and platform, change this if changing the length of rod length
        self.s=220

        #Angle of servo with respect to x-axis
        self.beta=beta

    def calcAngle(self, rotation, translation):

        length, pi=self.calcLength(rotation, translation)

        #p in x, y, z coordinates
        self.xp=pi[0] + translation[0]
        self.yp=pi[1] + translation[1]
        self.zp=pi[2] + translation[2]
        
        #Calculations, see documentation for more info
        L=length**2-(self.s**2-self.a**2)
        N=2*self.a*(math.cos(self.beta)*(self.xp-self.xb)+ math.sin(self.beta)*(self.yp-self.yb))
        M=2*self.a*(self.zp-self.zb)
        alpha=math.asin(L/(math.sqrt(M**2+N**2)))-math.atan(N/M)
        return alpha*180/(math.pi)+90
    
    
    def calcLength(self, rotation, translation):

        #Vector calculation of distance between servo and platform, see documentation for more info
        length = translation + np.dot(rotation, self.p) - self.B

        return np.linalg.norm(length), np.dot(rotation, self.p)