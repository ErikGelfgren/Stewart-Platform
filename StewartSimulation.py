import matplotlib.pyplot as plt
import numpy as np
import math

# Desired position and rotation for the platform
px = 0.0
py = 0.0
pz = 3.0

# Positional vector base->platform
p = np.array([[px], [py], [pz]])


def rotationMatrix(gamma, phi, theta):
    
    rx = np.array([math.cos(gamma)*math.cos(theta), -math.sin(gamma)*math.cos(phi)+math.cos(gamma)*math.sin(theta)*math.sin(phi), math.sin(gamma)*math.sin(phi)+math.cos(gamma)*math.sin(theta)*math.cos(phi)])
    ry = np.array([math.sin(gamma)*math.cos(theta), math.cos(gamma)*math.cos(phi)+math.sin(gamma)*math.sin(theta)*math.sin(phi), -math.cos(gamma)*math.sin(phi)+math.sin(gamma)*math.sin(theta)*math.cos(phi)])
    rz = np.array([-math.sin(theta), math.cos(theta)*math.sin(phi), math.cos(theta)*math.cos(phi)])


    # Rotational matrix
    return np.array([rx, ry, rz])

# Points of servos for arms in System 0 (Base platform)
arm1=[1.0,6.0,0]
arm1x = 1.0
arm1y = 6.0
arm1z = 0

arm2=[-1.0,6.0,0]
arm2x = -1.0
arm2y = 6.0
arm2z = 0

arm3=[5.0,-7.0,0]
arm3x = 5.0
arm3y = -7.0
arm3z = 0

arm4=[7.0,-5.0,0]
arm4x = 7.0
arm4y = -5.0
arm4z = 0

arm5=[-5.0,-7.0,0]
arm5x = -5.0
arm5y = -7.0
arm5z = 0

arm6=[-7.0,-5.0,0]
arm6x = -7.0
arm6y = -5.0
arm6z = 0

#Matrice for arms
arm1 = np.array([[arm1x], [arm1y], [arm1z]])
arm2 = np.array([[arm2x], [arm2y], [arm2z]])
arm3 = np.array([[arm3x], [arm3y], [arm3z]])
arm4 = np.array([[arm4x], [arm4y], [arm4z]])
arm5 = np.array([[arm5x], [arm5y], [arm5z]])
arm6 = np.array([[arm6x], [arm6y], [arm6z]])

# Points for arms in system 1 (connected platform)
b1x = arm1x / 2
b1y = arm1y / 2
b1z = 0.0

b2x = arm2x / 2
b2y = arm2y / 2
b2z = 0.0

b3x = arm3x / 2
b3y = arm3y / 2
b3z = 0.0

b4x = arm4x / 2
b4y = arm4y / 2
b4z = 0.0

b5x = arm5x / 2
b5y = arm5y / 2
b5z = 0.0

b6x = arm6x / 2
b6y = arm6y / 2
b6z = 0.0

#matrices for these points
b1 = np.array([[b1x], [b1y], [b1z]])
b2 = np.array([[b2x], [b2y], [b2z]])
b3 = np.array([[b3x], [b3y], [b3z]])
b4 = np.array([[b4x], [b4y], [b4z]])
b5 = np.array([[b5x], [b5y], [b5z]])
b6 = np.array([[b6x], [b6y], [b6z]])

# Initilize plot outside loop
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Plot for connections between arms and platform
ax.scatter(arm1[0, 0], arm1[1, 0], arm1[2, 0], marker='o', label='arm1', color='r')
ax.scatter(arm2[0, 0], arm2[1, 0], arm2[2, 0], marker='o', label='arm2', color='g')
ax.scatter(arm3[0, 0], arm3[1, 0], arm3[2, 0], marker='o', label='arm3', color='b')
ax.scatter(arm4[0, 0], arm4[1, 0], arm4[2, 0], marker='o', label='arm4', color='y')
ax.scatter(arm5[0, 0], arm5[1, 0], arm5[2, 0], marker='o', label='arm5', color='purple')
ax.scatter(arm6[0, 0], arm6[1, 0], arm6[2, 0], marker='o', label='arm6', color='pink')

# Initialize lines for arms
lineS1, = ax.plot(
    [arm1[0, 0], b1[0, 0]],
    [arm1[1, 0], b1[1, 0]],
    [arm1[2, 0], b1[2, 0]], label='s1', color='r')

lineS2, = ax.plot(
    [arm2[0, 0], b2[0, 0]],
    [arm2[1, 0], b2[1, 0]],
    [arm2[2, 0], b2[2, 0]], label='s2', color='g')

lineS3, = ax.plot(
    [arm3[0, 0], b3[0, 0]],
    [arm3[1, 0], b3[1, 0]],
    [arm3[2, 0], b3[2, 0]], label='s3', color='b')

lineS4, = ax.plot(
    [arm4[0, 0], b4[0, 0]],
    [arm4[1, 0], b4[1, 0]],
    [arm4[2, 0], b4[2, 0]], label='s4', color='y')

lineS5, = ax.plot(
    [arm5[0, 0], b5[0, 0]],
    [arm5[1, 0], b5[1, 0]],
    [arm5[2, 0], b5[2, 0]], label='s5', color='purple')

lineS6, = ax.plot(
    [arm6[0, 0], b6[0, 0]],
    [arm6[1, 0], b6[1, 0]],
    [arm6[2, 0], b6[2, 0]], label='s6', color='pink')

# Initialize endpoints for arms
line_endpoints, = ax.plot([], [], [], label='Platform Vågkraftvärk', color='blue')

ax.set_xlabel('X-axis')
ax.set_ylabel('Y-axis')
ax.set_zlabel('Z-axis')
ax.legend()

# Lim for axis plot
ax.set_xlim([-10, 10])
ax.set_ylim([-10, 10])
ax.set_zlim([-10, 10])

endpointsX = []
endpointsY = []
endpointsZ = []

plt.ion()  # Interactive to see movement
i = 0

while i < 721:

    degrees=math.radians(i)
    pitch=-(math.sin(degrees)*math.pi/8)
    yaw=0
    roll=0
    #Loop of rotation, insert pitch yaw and roll below to test movement
    r=rotationMatrix(yaw, roll, pitch)

    # Calculating new arm length
    s1 = p + np.dot(r, b1) - arm1
    s2 = p + np.dot(r, b2) - arm2
    s3 = p + np.dot(r, b3) - arm3
    s4 = p + np.dot(r, b4) - arm4
    s5 = p + np.dot(r, b5) - arm5
    s6 = p + np.dot(r, b6) - arm6

    # Uppdates lines from base to platform
    lineS1.set_xdata([arm1[0, 0], s1[0, 0] + arm1[0, 0]])
    lineS1.set_ydata([arm1[1, 0], s1[1, 0] + arm1[1, 0]])
    lineS1.set_3d_properties([arm1[2, 0], s1[2, 0] + arm1[2, 0]])

    lineS2.set_xdata([arm2[0, 0], s2[0, 0] + arm2[0, 0]])
    lineS2.set_ydata([arm2[1, 0], s2[1, 0] + arm2[1, 0]])
    lineS2.set_3d_properties([arm2[2, 0], s2[2, 0] + arm2[2, 0]])

    lineS3.set_xdata([arm3[0, 0], s3[0, 0] + arm3[0, 0]])
    lineS3.set_ydata([arm3[1, 0], s3[1, 0] + arm3[1, 0]])
    lineS3.set_3d_properties([arm3[2, 0], s3[2, 0] + arm3[2, 0]])

    lineS4.set_xdata([arm4[0, 0], s4[0, 0] + arm4[0, 0]])
    lineS4.set_ydata([arm4[1, 0], s4[1, 0] + arm4[1, 0]])
    lineS4.set_3d_properties([arm4[2, 0], s4[2, 0] + arm4[2, 0]])

    lineS5.set_xdata([arm5[0, 0], s5[0, 0] + arm5[0, 0]])
    lineS5.set_ydata([arm5[1, 0], s5[1, 0] + arm5[1, 0]])
    lineS5.set_3d_properties([arm5[2, 0], s5[2, 0] + arm5[2, 0]])

    lineS6.set_xdata([arm6[0, 0], s6[0, 0] + arm6[0, 0]])
    lineS6.set_ydata([arm6[1, 0], s6[1, 0] + arm6[1, 0]])
    lineS6.set_3d_properties([arm6[2, 0], s6[2, 0] + arm6[2, 0]])

    # Updates lines endpoints to encapsulate and draw platform
    endpointsX = [s1[0, 0]+arm1[0,0], s2[0, 0]+arm2[0,0], s6[0, 0]+arm6[0,0], s5[0, 0]+arm5[0,0], s3[0, 0]+arm3[0,0], s4[0, 0]+arm4[0,0], s1[0, 0]+arm1[0,0]]
    endpointsY = [s1[1, 0]+arm1[1,0], s2[1, 0]+arm2[1,0], s6[1, 0]+arm6[1,0], s5[1, 0]+arm5[1,0], s3[1, 0]+arm3[1,0], s4[1, 0]+arm4[1,0], s1[1, 0]+arm1[1,0]]
    endpointsZ = [s1[2, 0]+arm1[2,0], s2[2, 0]+arm2[2,0], s6[2, 0]+arm6[2,0], s5[2, 0]+arm5[2,0], s3[2, 0]+arm3[2,0], s4[2, 0]+arm4[2,0], s1[2, 0]+arm1[2,0]]

    # plots these lines
    line_endpoints.set_xdata(endpointsX)
    line_endpoints.set_ydata(endpointsY)
    line_endpoints.set_3d_properties(endpointsZ)

    plt.draw()
    plt.pause(0.01)

    radians=math.radians(i)
    #Calculates a circular path, change these to test path movements
    #x-coordinate
    p[0] = p[0] 
    #y-coordinate
    p[1] = math.cos(radians)*3
    #z-coordinate
    p[2] = math.sin(radians)*3+3
    i += 1

plt.ioff()
plt.show()