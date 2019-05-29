#!/usr/bin/env python
# -*- coding: utf-8 -*-
# for GPS sensor (Lab 1)

import sys
import lcm
# import time
import serial
from lab3lcm import gps_t
import utm


class gps(object):
    def __init__(self, port_name):
        self.port = serial.Serial(port_name, 4800, timeout=1.)  # 4800-N-8-1
        self.lcm = lcm.LCM()
        self.packet = gps_t()

    def readloop(self):
        while True:
            line = self.port.readline()  # read each line from GPS
            vals = [x for x in line.split(',')]  # store GPS line data in list, values separated by commas
            if vals[0] == "$GPGGA":  # check if GPS line data is GPGGA
                try:
                    self.packet.timestamp = float(vals[1])  # parse vals data for time and store to packet

                    latVals = vals[2]  # parse vals list for latitude data
                    latDeg = float(latVals[:2])  # parse latitude data for degrees info
                    latMin = float(latVals[2:])  # parse lat data for minutes info
                    lat = latDeg + latMin/60  # store latitude in decimal degrees

                    self.packet.lat_dir = vals[3]  # store latitude directional data in packet
                    if vals[3] == "N":  # check for direction. +/- input important for utm
                        self.packet.lat = lat  # store positive lat for North direction
                    else:
                        self.packet.lat = -lat  # store negative lat for South direction

                    lonVals = vals[4]  # parse vals for longitude data
                    lonDeg = float(lonVals[:3])  # parse longitude data for degrees info
                    lonMin = float(lonVals[3:])  # parse lon data for minutes info
                    lon = lonDeg + lonMin/60  # store longitude

                    self.packet.lon_dir = vals[5]  # store longitude directional data in packet
                    if vals[5] == "E":  # check for direction. +/- input important for utm
                        self.packet.lon = lon  # store positive lon for East direction
                    else:
                        self.packet.lon = -lon  # store negative lon for West direction

                    self.packet.altitude = float(vals[9])  # parse vals for altitude data in meters. store in packet

                    latLon = (self.packet.lat, self.packet.lon)  # store latitude and longitude
                    utmXY = utm.from_latlon(latLon[0], latLon[1])  # convert lat/lon info into UTM coordinates

                    self.packet.utm_x = utmXY[0]  # store x utm coordinate
                    self.packet.utm_y = utmXY[1]  # store y utm coordinate

                    print("timestamp: %.2f" %self.packet.timestamp)
                    print("lat: %.2f" % self.packet.lat)
                    print("lon: %.2f" % self.packet.lon)
                    print("altitude: %.2f" %self.packet.altitude)
                    print("lat dir: %s" %self.packet.lat_dir)
                    print("lon dir: %s" %self.packet.lon_dir)
                    print("utm x: %.2f" %self.packet.utm_x)
                    print("utm y: %.2f" %self.packet.utm_y)
                    print("")

                    self.lcm.publish("GPS", self.packet.encode())  # publish to GPS

                except:
                    print("Error reading GPS data")  # print error message when try fails then continue running
                    print (line)


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print "Usage: %s <serial_port>\n" % sys.argv[0]
        sys.exit(0)
    myGPS = gps(sys.argv[1])
    myGPS.readloop()
