#!/bin/sh
./Release/FastMarkerLocalizer /home/pi/fml/FastMarkerLocalizer/config/ config_rauschen.xml 1>> output.txt 2>> error.txt &
sleep 3
rm running.txt
