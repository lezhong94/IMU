#!/usr/bin/env python

import sys
import lcm

from lab3lcm import gps_t

if len(sys.argv) < 2:
    sys.stderr.write("usage: log_file_reader.py <logfile>\n")
    sys.exit(1)

log = lcm.EventLog(sys.argv[1], "r")  # set log from log file specified
c = 1
for event in log:  # look at all events logged
    if event.channel == "GPS":  # check for events logged on channel name "GPS"
        msg = gps_t.decode(event.data)  # read event data from GPS events
        c += 1
        print("GPS Data:")
        print("   timestamp   = %s" % str(msg.timestamp))
        print("   latitude    = %s" % str(msg.lat))
        print("   longitude = %s" % str(msg.lon))
        print("   altitude: %s" % str(msg.altitude))
        print("   utm x: %s" % str(msg.utm_x))
        print("   utm y: %s" % str(msg.utm_y))
        print("count: %d" % c)