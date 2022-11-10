#!/usr/bin/python

# Import Libraries for Linear Algebra and Plotting
import numpy as np
from numpy.linalg import inv
import matplotlib.pyplot as plt

# Function to generate noisy measurements
def getMeasurement(updateNumber):
    # Initialize the function variables to align with true values
    if updateNumber == 1:
      getMeasurement.currentPosition = 0
      getMeasurement.currentVelocity = 60 # m/s

    # If the measurement is updated 10 times a second, the time difference
    # between updates will be 0.1 seconds
    dt = 0.1

    # Compute random noise to add to position and velocity
    # Note: np.random.randn(1) will return 1 value on the standard normal
    # distribution with mean of 0. The "8" multiplier serves as a way to scale
    # that error. The number 8 was chosen at random to add a visible amount of
    # of noise.
    w = 8 * np.random.randn(1)
    v = 8 * np.random.randn(1)

    # z = pos + vel*dt + noise error
    z = getMeasurement.currentPosition + getMeasurement.currentVelocity*dt + v

    # Reset true position to be the current position minus the noise error
    getMeasurement.currentPosition = z - v
    # Update velocity to account for real changes in accelerations
    getMeasurement.currentVelocity = 60 + w
    return [z, getMeasurement.currentPosition, getMeasurement.currentVelocity]

def filter(z, updateNumber):
    dt = 0.1
    # Step 1: Initialize Filter Parameters:
    # System Model Variables, State Estimate, and State Covariance
    if updateNumber == 1:
        filter.x = np.array([[0],
                            [20]])
        filter.P = np.array([[5, 0],
                                 [0, 5]])

        filter.A = np.array([[1, dt],
                             [0, 1]])
        filter.H = np.array([[1, 0]])
        filter.HT = np.array([[1],
                              [0]])
        filter.R = 10
        filter.Q = np.array([[1, 0],
                             [0, 3]])

    # Predict State Forward
    x_prime = filter.A.dot(filter.x)
    # Predict Covariance Forward
    P_prime = filter.A.dot(filter.P).dot(filter.A.T) + filter.Q
    # Compute Kalman Gain
    S = filter.H.dot(P_prime).dot(filter.HT) + filter.R
    K = P_prime.dot(filter.HT)*(1/S)

    # Estimate State
    residual = z - filter.H.dot(x_prime)
    filter.x = x_prime + K*residual

    # Estimate Covariance
    filter.P = P_prime - K.dot(filter.H).dot(P_prime)

    return [filter.x[0], filter.x[1], filter.P, K];



def testFilter():
    # Define Range of Measurements to Loop Through
    dt = 0.1
    t = np.linspace(0, 10, num=300)
    numOfMeasurements = len(t)

    # Initialize arrays to save off data so it could be plotted
    measTime = []
    measPos = []
    measDifPos = []
    estDifPos = []
    estPos = []
    estVel = []
    posBound3Sigma = []
    posGain = []
    velGain = []

    # Loop through each measurement
    for k in range(1,numOfMeasurements):
        # Generate the latest measurement
        z = getMeasurement(k)
        # Call Filter and return new State
        f = filter(z[0], k)
        # Save off that state so that it could be plotted
        measTime.append(k)
        measPos.append(z[0])
        measDifPos.append(z[0]-z[1])
        estDifPos.append(f[0]-z[1])
        estPos.append(f[0])
        estVel.append(f[1])
        posVar = f[2]
        posBound3Sigma.append(3*np.sqrt(posVar[0][0]))
        K = f[3]
        posGain.append(K[0][0])
        velGain.append(K[1][0])

    return [measTime, measPos, estPos, estVel, \
           measDifPos, estDifPos, posBound3Sigma, \
           posGain, velGain];


# Execute Test Filter Function
t = testFilter()
# Plot Results
plot1 = plt.figure(1)
plt.scatter(t[0], t[1])
plt.plot(t[0], t[2])
plt.ylabel('Position')
plt.xlabel('Time')
plt.grid(True)

plot2 = plt.figure(2)
plt.plot(t[0], t[3])
plt.ylabel('Velocity (meters/seconds)')
plt.xlabel('Update Number')
plt.title('Velocity Estimate On Each Measurement Update \n', fontweight="bold")
plt.legend(['Estimate'])
plt.grid(True)

plot3 = plt.figure(3)
plt.scatter(t[0], t[4], color = 'red')
plt.plot(t[0], t[5])
plt.legend(['Estimate', 'Measurement'])
plt.title('Position Errors On Each Measurement Update \n', fontweight="bold")
plt.ylabel('Position Error (meters)')
plt.xlabel('Update Number')
plt.grid(True)
plt.xlim([0, 300])

plot4 = plt.figure(4)
plt.plot(t[0], t[7])
plt.plot(t[0], t[8])
plt.ylabel('Gain')
plt.xlabel('Update Number')
plt.grid(True)
plt.xlim([0, 100])
plt.legend(['Position Gain', 'Velocity Gain'])
plt.title('Position and Velocity Gains \n', fontweight="bold")
plt.show()
