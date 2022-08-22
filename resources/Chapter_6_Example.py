#!/usr/bin/python

import numpy as np
from numpy.linalg import inv
import matplotlib.pyplot as plt

#----------------------------------------------------------------
def getMeasurements():
    # Measurements are taken 1 time a second
    t = np.linspace(0, 25, num=25)
    numOfMeasurements = len(t)
    # Define x and y initial points -> Range = 4100 meters or 2.5 miles
    x = 2900
    y = 2900
    # Velocity magnitude = 22 meters per second OR 50 miles per hour
    vel = 22.0
    # Create storage arrays for true position data
    t_time = []
    t_x = []
    t_y = []
    t_r = []
    t_b = []
    # Compute the Real Position Data - This simulation use the cartesian
    # coordinate frame for computing new positional points and then converts
    # them to the polar coordinates.
    for i in range(0,numOfMeasurements):
        # Set the delta time to 1 second
        dT = 1.0
        # Store off the update time for this position update
        t_time.append(t[i])
        # Compute the new x and y position data with the assumption all of the
        # velocity motion is in the x direction i.e. linear motion
        x = x+dT*vel
        y = y
        # Store off the computed x and y data
        t_x.append(x)
        t_y.append(y)
        # Compute the sum of the squares of x and y as an intermediate step
        # before computing the range and storing it off
        temp = x*x + y*y
        r = np.sqrt(temp)
        t_r.append(r)
        # Compute the azimuth (or bearing) with the arctan2 function and convert
        # it to degrees. Then store this azimuth data
        b = np.arctan2(x, y) * 180/np.pi
        t_b.append(b)

    # Create storage containers for polar measurement data
    m_r = []
    m_b = []
    m_cov = []
    # Bearing standard deviation = 9 milliradians (in degrees)
    sig_b = 0.009*180/np.pi
    sig_r = 30 # Range Standard Deviation = 30 meters
    # Storage containers for cartesian measurements - for analysis purposes
    m_x = []
    m_y = []
    for ii in range(0, len(t_time)):
        # Compute the error for each measurement
        # By taking the max between .25 of the defined standard deviation and
        # the randomly generated normal error, it guarantees an error
        temp_sig_b = np.maximum(sig_b * np.random.randn(), 0.25*sig_b)
        temp_sig_r = np.maximum(sig_r * np.random.randn(), 0.25*sig_r)
        # Save off the measurement values for bearing and range as a Function
        # of the true value + the error generated above
        temp_b = t_b[ii] + temp_sig_b
        temp_r = t_r[ii] + temp_sig_r
        # Save off the measurement data
        m_b.append(temp_b)
        m_r.append(temp_r)
        m_cov.append(np.array([[temp_sig_r*temp_sig_r, 0],
                      [0, temp_sig_b*temp_sig_b]]))
        m_x.append(temp_r*np.sin(temp_b*np.pi/180))
        m_y.append(temp_r*np.cos(temp_b*np.pi/180))

    return [m_r, m_b, m_cov, t_r, t_b, t_time, t_x, t_y, m_x, m_y]
    #        0    1     2      3    4    5      6   7    8    9


#----------------------------------------------------------------
def ekfilter(z, updateNumber): # z = [r, b]
    dt = 1.0
    j = updateNumber
    # Initialize State
    if updateNumber == 0: # First Update
        # compute position values from measurements
        temp_x = z[0][j]*np.sin(z[1][j]*np.pi/180) # x = r*sin(b)
        temp_y = z[0][j]*np.cos(z[1][j]*np.pi/180) # y = r*cos(b)
        # State vector - initialize position values
        ekfilter.x = np.array([[temp_x],
                            [temp_y],
                            [0],
                            [0]])
        # State covariance matrix - initialized to zero for first update
        ekfilter.P = np.array([[0, 0, 0, 0],
                                 [0, 0, 0, 0],
                                 [0, 0, 0, 0],
                                 [0, 0, 0, 0]])
        # State transistion matrix - linear extrapolation assuming constant velocity
        ekfilter.A = np.array([[1, 0, dt, 0],
                             [0, 1, 0, dt],
                             [0, 0, 1, 0],
                             [0, 0, 0, 1]])
        # Measurement covariance matrix
        ekfilter.R = z[2][j]
        # System error matrix - initialized to zero matrix for first update
        ekfilter.Q = np.array([[0, 0, 0, 0],
                                 [0, 0, 0, 0],
                                 [0, 0, 0, 0],
                                 [0, 0, 0, 0]])
        # Residual and kalman gain
        # not computed for first update but initialized so it could be output
        residual = np.array([[0, 0],
                      [0, 0]])
        K = np.array([[0, 0],
                      [0, 0],
                      [0, 0],
                      [0, 0]])

    # Reinitialize State
    if updateNumber == 1: # Second Update
        # Get previous state vector values
        prev_x = ekfilter.x[0][0]
        prev_y = ekfilter.x[1][0]
        temp_x = z[0][j]*np.sin(z[1][j]*np.pi/180) # x = r*sin(b)
        temp_y = z[0][j]*np.cos(z[1][j]*np.pi/180) # y = r*cos(b)
        #  Compute velocity - vel = (pos2 - pos1)/deltaTime
        temp_xv = (temp_x - prev_x)/dt
        temp_yv = (temp_y - prev_y)/dt
        # State vector - reinitialized with new position and computed velocity
        ekfilter.x = np.array([[temp_x],
                            [temp_y],
                            [temp_xv],
                            [temp_yv]])
        # state covariance matrix - initialized to large values
        ekfilter.P = np.array([[100, 0, 0, 0],
                                 [0, 100, 0, 0],
                                 [0, 0, 250, 0],
                                 [0, 0, 0, 250]])
        # State transistion matrix - linear extrapolation assuming constant velocity
        ekfilter.A = np.array([[1, 0, dt, 0],
                             [0, 1, 0, dt],
                             [0, 0, 1, 0],
                             [0, 0, 0, 1]])
        # Measurement covariance matrix - provided by the measurment source
        ekfilter.R = z[2][j]
        # System error matrix
        # adds 4.5 m std dev in position and 2 m/s std dev in velocity
        ekfilter.Q = np.array([[20, 0, 0, 0],
                                 [0, 20, 0, 0],
                                 [0, 0, 4, 0],
                                 [0, 0, 0, 4]])
        # Residual and kalman gain-  initialized so it could be output
        residual = np.array([[0, 0],
                      [0, 0]])
        K = np.array([[0, 0],
                      [0, 0],
                      [0, 0],
                      [0, 0]])

    if updateNumber > 1: # Third Update and Subsequent Updates
      # Predict state and state covariance forward
      x_prime = ekfilter.A.dot(ekfilter.x)
      P_prime = ekfilter.A.dot(ekfilter.P).dot(ekfilter.A.T) + ekfilter.Q
      # Form state to measurement transition matrix
      x1 = x_prime[0][0]
      y1 = x_prime[1][0]
      x_sq = x1*x1
      y_sq = y1*y1
      den = x_sq+y_sq
      den1 = np.sqrt(den)
      ekfilter.H = np.array([[  x1/den1,    y1/den1, 0, 0],
                           [y1/den, -x1/den, 0, 0]])
      ekfilter.HT = np.array([[x1/den1, y1/den],
                              [y1/den1, -x1/den],
                              [0, 0],
                              [0, 0]])
      # Measurement covariance matrix
      ekfilter.R = z[2][j]
      # Compute Kalman Gain
      S = ekfilter.H.dot(P_prime).dot(ekfilter.HT) + ekfilter.R
      K = P_prime.dot(ekfilter.HT).dot(np.linalg.inv(S))
      # Estimate State
      temp_z = np.array([[z[0][j]],
                         [z[1][j]]])
      # Convert the predicted cartesian state to polar range and azimuth
      pred_x = x_prime[0][0]
      pred_y = x_prime[1][0]
      sumSquares = pred_x*pred_x + pred_y*pred_y
      pred_r = np.sqrt(sumSquares)
      pred_b = np.arctan2(pred_x, pred_y) * 180/np.pi
      h_small = np.array([[pred_r],
                       [pred_b]])
      # Compute residual difference between state and measurement for data time
      residual = temp_z - h_small
      # Compute new estimate for state vector using the Kalman Gain
      ekfilter.x = x_prime + K.dot(residual)
      # Compute new estimate for state covariance using the Kalman Gain
      ekfilter.P = P_prime - K.dot(ekfilter.H).dot(P_prime)
    return [ekfilter.x[0], ekfilter.x[1], ekfilter.P, ekfilter.x[2], ekfilter.x[3], K, residual];
#               0             1              2           3               4          5     6
# return [ekfilter.x[0], ekfilter.x[1], ekfilter.x[2], ekfilter.x[3], ekfilter.P, K, residual, updateNumber];
#----------------------------------------------------------------


#for k in range(1,numOfMeasurements):
#    z = getMeasurement(k)
#    # Call Filter and return new State
#    f = filter(z[0], k)

#--------------------------------------
# create measurements by adding noise
# create covariance matrix
#--------------------------------------



f_x = []
f_y = []
f_x_sig = []
f_y_sig =[]
f_xv = []
f_yv = []
f_xv_sig = []
f_yv_sig =[]

z = getMeasurements()
for iii in range(0, len(z[0])):
  f = ekfilter(z, iii)
  f_x.append(f[0])
  f_y.append(f[1])
  f_xv.append(f[3])
  f_yv.append(f[4])
  f_x_sig.append(np.sqrt(f[2][0][0]))
  f_y_sig.append(np.sqrt(f[2][1][1]))

plot1 = plt.figure(1)
plt.grid(True)
plt.plot(z[5], z[3])
plt.scatter(z[5], z[0])
plt.title('Actual Range vs Measured Range')
plt.legend(['Ship Actual Range', 'Ship Measured Range'])
plt.ylabel('Range (meters)')
plt.xlabel('Update Number')

plot2 = plt.figure(2)
plt.grid(True)
plt.plot(z[5], z[4])
plt.scatter(z[5], z[1])
plt.title('Actual Azimuth vs Measured Azimuth')
plt.legend(['Ship Actual Azimuth', 'Ship Measured Azimuth'])
plt.ylabel('Azimuth (degrees)')
plt.xlabel('Update Number')

plot3 = plt.figure(3), plt.grid(True)
plt.plot(z[5], f_xv)
plt.plot(z[5], f_yv)
plt.title('Velocity Estimate On Each Measurement Update \n', fontweight="bold")
plt.legend(['X Velocity Estimate', 'Y Velocity Estimate'])

# Compute Range Error
e_x_err = []
e_x_3sig = []
e_x_3sig_neg = []
e_y_err = []
e_y_3sig = []
e_y_3sig_neg = []
for m in range(0, len(z[0])):
    e_x_err.append(f_x[m]-z[6][m])
    e_x_3sig.append(3*f_x_sig[m])
    e_x_3sig_neg.append(-3*f_x_sig[m])
    e_y_err.append(f_y[m]-z[7][m])
    e_y_3sig.append(3*f_y_sig[m])
    e_y_3sig_neg.append(-3*f_y_sig[m])

plot4 = plt.figure(4), plt.grid(True)
line1 = plt.scatter(z[5], e_x_err)
line2, = plt.plot(z[5], e_x_3sig, color='green')
plt.plot(z[5], e_x_3sig_neg, color='green')
plt.ylabel('Position Error (meters)')
plt.xlabel('Update Number')
plt.title('X Position Estimate Error Containment \n', fontweight="bold")
plt.legend([line1, line2,], ['X Position Error', '3 Sigma Error Bound'])

plot5 = plt.figure(5), plt.grid(True)
yline1 = plt.scatter(z[5], e_y_err)
yline2, = plt.plot(z[5], e_y_3sig, color='green')
plt.plot(z[5], e_y_3sig_neg, color='green')
plt.ylabel('Position Error (meters)')
plt.xlabel('Update Number')
plt.title('Y Position Estimate Error Containment \n', fontweight="bold")
plt.legend([yline1, yline2,], ['Y Position Error', '3 Sigma Error Bound'])
plt.show()
