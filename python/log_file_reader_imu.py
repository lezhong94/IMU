#!/usr/bin/env python

import sys
import lcm

from lab3lcm import imu_t

if len(sys.argv) < 2:
    sys.stderr.write("usage: log_file_reader.py <logfile>\n")
    sys.exit(1)

log = lcm.EventLog(sys.argv[1], "r")  # set log from log file specified
c = 1
for event in log:  # look at all events logged
    if event.channel == "IMU":  # check for events logged on channel name "GPS"
        msg = imu_t.decode(event.data)  # read event data from GPS events
        c += 1
        print("IMU Data:")
        print("   yaw   = %s" % str(msg.yaw))
        print("   pitch = %s" % str(msg.pitch))
        print("   roll  = %s" % str(msg.roll))
        print("   mag x = %s" % str(msg.mag_x))
        print("   mag y = %s" % str(msg.mag_y))
        print("   mag z = %s" % str(msg.mag_z))
        print("count: %d" % c)
        print("")