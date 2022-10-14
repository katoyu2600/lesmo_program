#!/usr/bin/env python
#-----------------------------------------------------------------------------
# qwiic_gps_ex1.py
#
# Simple Example for SparkFun GPS Breakout - XA1110
# In this example NMEA data is requested from the GPS module and the recieved Latitude,
# Longitude, and Time data is printed to the screen.  
#------------------------------------------------------------------------
#
# Written by  SparkFun Electronics, October 2019
#
#
# More information on qwiic is at https://www.sparkfun.com/qwiic
#
# Do you like this library? Help support SparkFun. Buy a board!
#
#==================================================================================
# Copyright (c) 2019 SparkFun Electronics
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
#==================================================================================
# Example 1
#

from __future__ import print_function
import time
import sys
import qwiic_titan_gps

def run_example():

    print("SparkFun GPS Breakout - XA1110!")
    qwiicGPS = qwiic_titan_gps.QwiicTitanGps()

    if qwiicGPS.connected is False:
        print("Could not connect to to the SparkFun GPS Unit. Double check that\
              it's wired correctly.", file=sys.stderr)
        return

    qwiicGPS.begin()

    while True:
        if qwiicGPS.get_nmea_data() is True:
            print("Latitude: {}, Longitude: {}, Time: {}".format(
                qwiicGPS.gnss_messages['Latitude'],
                qwiicGPS.gnss_messages['Longitude'],
                qwiicGPS.gnss_messages['Time']))

        time.sleep(1)

GPS_Realtime_txt_PATH = "gps_realtime.txt"

def read_rgps() :
    global GPS_Realtime_txt_PATH 
    qwiicGPS = qwiic_titan_gps.QwiicTitanGps()
    
    if qwiicGPS.connected is False:
        print("Could not connect to to the SparkFun GPS Unit. Double check that\
              it's wired correctly.", file=sys.stderr)
        return

    qwiicGPS.begin()
    
    if qwiicGPS.get_nmea_data() is True:
        GPS_Array = [qwiicGPS.gnss_messages['Latitude'], qwiicGPS.gnss_messages['Longitude']]
        if GPS_Array != 0:
            Main_State = t_TXT_Write_module(GPS_Array,GPS_Realtime_txt_PATH)
            time.sleep(0.9)
            return Main_State
        
def t_TXT_Write_module(GPS_Array,GPS_txt_PATH):
    if GPS_Array != 0:
        fp = open(GPS_txt_PATH , "w")
        GPS_Array = str(GPS_Array)
        fp.writelines(GPS_Array[1:len(GPS_Array)-1:1])
        fp.close()
    return GPS_Array    

#if __name__ == '__main__':
    #try:
    #    run_example()
    #except (KeyboardInterrupt, SystemExit) as exErr:
    #    print("Ending Basic Example.")
    #    sys.exit(0)
        
if __name__ == "__main__":
    GPS_Realtime_txt_PATH = "gps_goalpoint.txt"
    i = 1000
    try:
        while(True):
            print(read_rgps())
            i-=1
            time.sleep(0.1)
    except KeyboardInterrupt:
        sys.exit("KeyboardInterrupt")