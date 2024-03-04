# script to collect serial data from arduino
# IMPORTANT: ensure Arduino serial monitor is closed before running this script
import csv
import os
import serial # pip install pyserial
import numpy as np
import matplotlib.pyplot as plt

# set up comport:
COMPORT = '/dev/cu.usbmodem-14201' # adjust this
arduino = serial.Serial(COMPORT, 115200, bytesize=8)
print(f"Connected to port {arduino.name}.")

# get name of file to save data to:
FILENAME = './test_data'
NEWFILE = './test_data'
INDEX = 1

while os.path.exists(NEWFILE+'.csv'):
    print(f"File {NEWFILE}.csv already exists.")
    NEWFILE = FILENAME + '_' + str(INDEX)
    INDEX += 1
print(f"Saving data to {NEWFILE}.csv")


# open corresponding file and record data:
with open(NEWFILE+'.csv', 'w', newline='') as csvfile:
    datawriter = csv.writer(csvfile, delimiter='\n')

    ############################################################################
    # Read in inital values:
    # cols = arduino.readline().decode('utf-8').rstrip().split('\n')
    # datawriter.writerow([cols]) # Log column headers

    datawriter.writerow(arduino.readline().decode('utf-8').rstrip().split('\n'))
    Freq = float(arduino.readline().decode('utf-8').rstrip())
    datawriter.writerow([Freq]) # Log Sample Frequency in Hz

    datawriter.writerow(arduino.readline().decode('utf-8').rstrip().split('\n'))
    Time = float(arduino.readline().decode('utf-8').rstrip())
    datawriter.writerow([Time]) # Log test duration in seconds

    datawriter.writerow(arduino.readline().decode('utf-8').rstrip().split('\n'))
    H_Gain = float(arduino.readline().decode('utf-8').rstrip())
    datawriter.writerow([H_Gain]) # Log Feedback gain

    datawriter.writerow(arduino.readline().decode('utf-8').rstrip().split('\n'))
    cnt_max = int(arduino.readline().decode('utf-8').rstrip())
    datawriter.writerow([cnt_max]) # Log total number of timesteps

    datawriter.writerow(arduino.readline().decode('utf-8').rstrip().split('\n'))
    pressure_offset = float(arduino.readline().decode('utf-8').rstrip())
    datawriter.writerow([pressure_offset]) # Log pressure offset
    ############################################################################

    # Initialize Vectors
    T = np.linspace(0, Time, cnt_max) # Setup Time Vector
    R = np.zeros(cnt_max)             # Initialize Reference Input Vector
    Y = np.zeros(cnt_max)             # Initialize Output Vector

    print("Initialization complete.")
    print("Starting data capture...")

    # read in output data line by line:
    # time, input, output
    for i in range(cnt_max):
        R[i] = float(arduino.readline().decode('utf-8').rstrip())
        datawriter.writerow([R[i]])
        Y[i] = float(arduino.readline().decode('utf-8').rstrip())
        datawriter.writerow([Y[i]])

print('Data capture complete.')


# Convert Y from encoder counts to rad, where H_Gain is the Feedback gain in encoder counts/rad
# Y = Y/H_Gain

# Plot captured data
fig, (ax1, ax2) = plt.subplots(2, 1)

ax1.plot(T, R)
ax1.set_title('Reference Input vs. Time')
ax1.set_ylabel('Reference Input')
ax1.set_xlabel('Time (seconds)')

ax2.plot(T, Y)
ax2.set_title('Output vs. Time')
ax2.set_ylabel('Angular Position (radians)')
ax2.set_xlabel('Time (seconds)')
plt.savefig(NEWFILE+'_plot.png')
plt.show()