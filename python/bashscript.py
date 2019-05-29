#/usr/bin/env bash
sudo chmod 666 /dev/ttyUSB0
sudo chmod 666 /dev/ttyUSB1
export CLASSPATH=$PWD
# if using lcm over multicast in local network use belowexport
#export LCM_DEFAULT_URL=udpm://230.0.0.0:8080?ttl=1
lcm-logger -s .././log_files/lcm-log-%F-%T &
../lcm-spy/./runspy.sh &
python imu_driver.py  '/dev/ttyUSB1' &
python gps_driver.py '/dev/ttyUSB0'

# SIGINT        2       Term    Interrupt from keyboard
kill %1 %2 %3 %4