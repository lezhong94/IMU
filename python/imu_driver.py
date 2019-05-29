#!/usr/bin/env python
# -*- coding: utf-8 -*-
# for GPS sensor (Lab 1)

import sys
import lcm
# import time
import serial
from lab3lcm import imu_t


class gps(object):
    def __init__(self, port_name):
        self.port = serial.Serial(port_name, 115200, timeout=1.)  # 115200-N-8-1
	self.lcm = lcm.LCM()
        #self.lcm = lcm.LCM("udpm://239.255.76.67:7667?ttl=1")
        self.packet = imu_t()

    def readloop(self):
        while True:
            line = self.port.readline()  # read each line from GPS
            vals = [x for x in line.split(',')]  # store GPS line data in list, values separated by commas
            if vals[0] == "$VNYMR":  # check if GPS line data is GPGGA
                try:
                    self.packet.yaw = float(vals[1])  # parse vals for yaw and store to packet
                    self.packet.pitch = float(vals[2])  # parse vals for pitch and store to packet
                    self.packet.roll = float(vals[3])  # parse vals for roll and store to packet

                    self.packet.mag_x = float(vals[4])  # parse vals for magnetic x data and store to packet
                    self.packet.mag_y = float(vals[5])  # parse vals for magnetic y data and store to packet
                    self.packet.mag_z = float(vals[6])  # parse vals for magnetic z data and store to packet

                    self.packet.accel_x = float(vals[7])  # parse vals for acceleration x data and store to packet
                    self.packet.accel_y = float(vals[8])  # parse vals for acceleration y data and store to packet
                    self.packet.accel_z = float(vals[9])  # parse vals for acceleration z data and store to packet

                    self.packet.gyro_x = float(vals[10])  # parse vals for acceleration x data and store to packet
                    self.packet.gyro_y = float(vals[11])  # parse vals for acceleration y data and store to packet

                    gyro_z = vals[12]  # parse vals for acceleration z data and store to packet
                    gyro_z_dec = float(gyro_z[:10])  # extract decimal portion of gyro z
                    gyro_z_hex = int(gyro_z[11:], 16)  # extract hex portion of gyro z
                    gyro_z = gyro_z_dec*gyro_z_hex
                    self.packet.gyro_z = gyro_z

                    print("yaw: %f" % self.packet.yaw)
                    print("pitch: %f" % self.packet.pitch)
                    print("roll: %f" % self.packet.roll)
                    print("mag_x: %f" % self.packet.mag_x)
                    print("mag_y: %f" % self.packet.mag_y)
                    print("mag_z: %f" % self.packet.mag_z)
                    print("accel_x: %f" %self.packet.accel_x)
                    print("accel_y: %f" %self.packet.accel_y)
                    print("accel_z: %f" %self.packet.accel_z)
                    print("gyro_x: %f" %self.packet.gyro_x)
                    print("gyro_y: %f" %self.packet.gyro_y)
                    print("gyro_z: %f" % self.packet.gyro_z)
                    print("")

                    self.lcm.publish("IMU", self.packet.encode())  # publish to IMU

                except:
                    print("Error reading IMU data")  # print error message when try fails then continue running
                    print (line)


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print "Usage: %s <serial_port>\n" % sys.argv[0]
        sys.exit(0)
    myGPS = gps(sys.argv[1])
    myGPS.readloop()
