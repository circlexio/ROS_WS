from distutils.log import error
import readline
from termios import CSIZE
import time
import sys


import VL53L0X
import RPi.GPIO as GPIO
import json
import signal

import pandas as pd
import numpy as np
import math



'''

3---1
|. .|       Senesors orientation and numbering when
|. .|       PrintHead Front, Facing you.
|. .|         
4---2

'''

ERROR_TOF1 = 601
ERROR_TOF2 = 602
ERROR_TOF3 = 603
ERROR_TOF4 = 604

# GPIO for Sensor 1 shutdown pin
sensor1_shutdown = 8
# GPIO for Sensor 2 shutdown pin
sensor2_shutdown = 10
# GPIO for Sensor 1 shutdown pin
sensor3_shutdown = 11
# GPIO for Sensor 2 shutdown pin
sensor4_shutdown = 12

# Create one object per VL53L0X passing the address to give to
# each.
tof1 = VL53L0X.VL53L0X(address=0x30)
tof2 = VL53L0X.VL53L0X(address=0x31)
tof3 = VL53L0X.VL53L0X(address=0x32)
tof4 = VL53L0X.VL53L0X(address=0x33)

# define sensor pos on printHead
sensorVDist = 130   # in mm
sensorHDist = 76    # in mm

safetyDist = 0 # in mm

def handler(signum, frame):
    res = input("\nCtrl-c was pressed. Do you really want to exit? y/n ")
    if res == 'y':
        tof1.stop_ranging()
        tof2.stop_ranging()
        tof3.stop_ranging()
        tof4.stop_ranging()

        exit(1)

def setupBoard():
    GPIO.setwarnings(False)

    # Setup GPIO for shutdown pins on each VL53L0X
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(sensor1_shutdown, GPIO.OUT)
    GPIO.setup(sensor2_shutdown, GPIO.OUT)
    GPIO.setup(sensor3_shutdown, GPIO.OUT)
    GPIO.setup(sensor4_shutdown, GPIO.OUT)

    # Set all shutdown pins low to turn off each VL53L0X
    GPIO.output(sensor1_shutdown, GPIO.LOW)
    GPIO.output(sensor2_shutdown, GPIO.LOW)
    GPIO.output(sensor3_shutdown, GPIO.LOW)
    GPIO.output(sensor4_shutdown, GPIO.LOW)

    # Keep all low for 500 ms or so to make sure they reset
    time.sleep(0.50)

    signal.signal(signal.SIGINT, handler)
    # Set shutdown pin high for the first VL53L0X then
    # call to start ranging
    GPIO.output(sensor1_shutdown, GPIO.HIGH)
    time.sleep(0.50)
    tof1.start_ranging(VL53L0X.VL53L0X_BETTER_ACCURACY_MODE)

    # Set shutdown pin high for the second VL53L0X then
    # call to start ranging
    GPIO.output(sensor2_shutdown, GPIO.HIGH)
    time.sleep(0.50)
    tof2.start_ranging(VL53L0X.VL53L0X_BETTER_ACCURACY_MODE)

    # Set shutdown pin high for the third VL53L0X then
    # call to start ranging
    GPIO.output(sensor3_shutdown, GPIO.HIGH)
    time.sleep(0.50)
    tof3.start_ranging(VL53L0X.VL53L0X_BETTER_ACCURACY_MODE)

    # Set shutdown pin high for the fourth VL53L0X then
    # call to start ranging
    GPIO.output(sensor4_shutdown, GPIO.HIGH)
    time.sleep(0.50)
    tof4.start_ranging(VL53L0X.VL53L0X_BETTER_ACCURACY_MODE)

def getTOFData(tof, errorNum):
    distance = tof.get_distance()
    error = 0
    if (distance < 0):
        error = errorNum
    return distance, error

def kalmanFilter(measurement, predVal, kf_gain):
    predVal = predVal + (1/kf_gain) * (measurement - predVal)
    return predVal,kf_gain

def calib(loop=100,calib1=0,calib2=0,calib3=0,calib4=0):
    CalibTOF1_predVal, CalibTOF2_predVal, CalibTOF3_predVal, CalibTOF4_predVal = 0,0,0,0
    kf_gain = 1
    i=0
    while(i<loop):
        distance1, error1 = getTOFData(tof1, ERROR_TOF1)
        distance2, error2 = getTOFData(tof2, ERROR_TOF2)
        distance3, error3 = getTOFData(tof3, ERROR_TOF3)
        distance4, error4 = getTOFData(tof4, ERROR_TOF4)
        CalibTOF1_predVal, kf_gain = kalmanFilter(distance1-calib1, CalibTOF1_predVal, kf_gain)
        CalibTOF2_predVal, kf_gain = kalmanFilter(distance2-calib2, CalibTOF2_predVal, kf_gain)
        CalibTOF3_predVal, kf_gain = kalmanFilter(distance3-calib3, CalibTOF3_predVal, kf_gain)
        CalibTOF4_predVal, kf_gain = kalmanFilter(distance4-calib4, CalibTOF4_predVal, kf_gain)
        kf_gain += 1
        i+=1
        CalibList = [CalibTOF1_predVal, CalibTOF2_predVal, CalibTOF3_predVal, CalibTOF4_predVal]
        CalibList = [round(i,1) for i in CalibList]
        # print(i)
        # print(CalibList)
        CalibList1 = [round(i) for i in CalibList]
        # print(CalibList1)
    calib = np.array(CalibList1)
    return CalibList1


def main():

    setupBoard()

    timing = tof1.get_timing()
    if (timing < 20000):
        timing = 20000
    print("Timing %d ms" % (timing/1000))

    try:
        from config import successCalib
    except:
        successCalib = -1
        input("about to caliberate... press ANY kEY to continue....")
        while(successCalib == -1):
            successCalib = calib()
            if successCalib == -1:
                print("Caliberation Unsuccessful...")
        print("Caliberation Successful !!!")

        with open("./config.py",'w') as f:
            f.writelines(["successCalib="+str(successCalib)])
            f.close()

    print(successCalib)
    CalibTOF1_predVal, CalibTOF2_predVal, CalibTOF3_predVal, CalibTOF4_predVal = successCalib
    dataJson = {}
    kf_gain = 1
    intiVal = 265
    TOF2_predVal = intiVal
    TOF1_predVal = intiVal
    TOF3_predVal = intiVal
    TOF4_predVal = intiVal

def orientation_angle():
    # apply kalmanFilter
    res = calib(loop=20,  
                calib1 = CalibTOF1_predVal - safetyDist,
                calib2 = CalibTOF2_predVal - safetyDist,
                calib3 = CalibTOF3_predVal - safetyDist,
                calib4 = CalibTOF4_predVal - safetyDist)
    print("filtered:")
    print(res) 
    #avg values
    avgDist = [ (res[0] + res[2])/2, 
                (res[2] + res[3])/2, 
                (res[1] + res[3])/2, 
                (res[0] + res[1])/2 ]              
    print("top,right,bot,left :")
    print(avgDist)

    top,right,bot,left = avgDist
    yawAngle = np.arctan((right-left)/sensorHDist)
    print(yawAngle)

    pitchAngle = np.arctan((top-bot)/sensorVDist)
    print(pitchAngle)

    return [pitchAngle, yawAngle]

