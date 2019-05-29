#!/bin/sh

lcm-gen -j ../types/*.lcm

javac -cp ~/Downloads/lcm-1.3.1/lcm-java/lcm.jar lab3lcm/*.java

jar cf my_types.jar lab3lcm/*.class
