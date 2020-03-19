#!/bin/bash

#generate the .ini files for all vehicles

export LC_NUMERIC=en_US.utf8

j=1
cat circle_positions.txt | while read LINE; do
  cp circle_base.ini `printf ./vehicle%02d.ini $j`
  printf "sim_vehicle_start_latitude\t%lf\nsim_vehicle_start_longitude\t%lf\nsim_vehicle_start_theta\t%lf\n" $LINE >> `printf ./vehicle%02d.ini $j`
  ((j++))
done 


#startup script for the remote computer 
# running centrals and simulated traffic

Y=50

RUN ()
{
  xterm -geometry 80x4+0+$Y -T "RUN $1" -e /bin/bash -l -c "$1 $2 $3 $4 $5" &
sleep 1 
 Y=$[ $Y + 120 ]
}


cd ~/sci/workspace/bin


NUM=$1

killall central
sleep 2

for i in $(seq 1381 $(( 1380 + $NUM ))); do
# central -p selects port to listen. default is 1381
echo "port $i"
./central -p$i &
done


sleep 2

for i in $(seq 1 $(( $NUM ))); do
sleep 2
export CENTRALHOST=localhost:$(( 1380 + $i ))
./param_server `printf ~/sci/workspace/driving/development/src/program/vehiclesim_dev/example/vehicle%02d.ini $i` &
RUN ./fake_localize &
RUN ./fake_estop run &
done

sleep 1
RUN ~/sci/workspace/bin/multisim_dev -localn $NUM

sleep 1

for i in $(seq 1381 $(( 1380 + $NUM ))); do
export CENTRALHOST=localhost:$i
#RUN ./controller
#RUN ./planner_wpo
#RUN ./aw_planner
#sleep 2
RUN ./controller
done


#sleep 2

export CENTRALHOST=localhost:1381
#RUN ./perception_view_dev

#sleep 5 #enable to view star messages
