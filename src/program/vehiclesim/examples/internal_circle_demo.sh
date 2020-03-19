#!/bin/bash

#generate the .ini files for all vehicles

export LC_NUMERIC=en_US.utf8

#startup script for the remote computer 
# running centrals and simulated traffic

Y=50

RUN ()
{
  xterm -geometry 80x4+0+$Y -T "RUN $1" -e /bin/bash -l -c "$1 $2 $3 $4 $5" &
sleep 1 
 Y=$[ $Y + 120 ]
}


cd ~/sci/workspace/hgdriving/bin

killall central
sleep 2

./central >/dev/null&

sleep 1

./param_server `printf ~/sci/workspace/hgdriving/src/program/vehiclesim/examples/internal_circle.ini` &

sleep 1

RUN ./fake_estop run &

#./aw_planner circledemo
