#!/bin/bash

for device in 1 2 3 4 5 6 7 8 9 10 11 12; do
    echo $device
    (echo "conf set servo.flux_brake_min_voltage 20.3";
     sleep 0.05;
     echo "conf set servo.flux_brake_resistance_ohm 0.05";
     sleep 0.2;
     echo "conf write";
     sleep 0.5) | ($1 -t $device -c)
done
