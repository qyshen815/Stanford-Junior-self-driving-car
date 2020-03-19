#!/bin/bash

#generate the .ini files for all vehicles

export LC_NUMERIC=en_US.utf8

j=1
cat vail_positions.txt | while read LINE; do
  cp vail_base.ini `printf ./vehicle%02d.ini $j`
  printf "sim_vehicle_start_latitude\t%lf\nsim_vehicle_start_longitude\t%lf\nsim_vehicle_start_theta\t%lf\t%s\n" $LINE >> `printf ./vehicle%02d.ini $j`
  ((j++))
done 


#startup script for the remote computer 
# running centrals and simulated traffic

Y=50

RUN ()
{
 xterm -geometry 80x4+0+$Y -T "RUN $1" -e /bin/bash -l -c "$1 $2 $3 $4 $5; sleep 5" &
# sleep 1 
 Y=$[ $Y + 120 ]
}


if [ $# -eq 0 ]
then
 NUM=1
else
 NUM=$1
fi

killall -9 central
sleep 2

cd $VLR_ROOT/bin

for i in $(seq 1381 $(( 1380 + $NUM ))); do
echo "starting central on port $i"
./central -u -s -p$i &
done


for i in $(seq 1 $(( $NUM ))); do
sleep 1
export CENTRALHOST=localhost:$(( 1380 + $i ))
./param_server `printf $VLR_ROOT/src/program/vehiclesim/examples/vehicle%02d.ini $i` &
RUN ./fake_localize &
RUN ./fake_estop pause &
done

RUN ./multisim -localn $NUM

for i in $(seq 1381 $(( 1380 + $NUM ))); do
export CENTRALHOST=localhost:$i
RUN ./aw_planner nowaitforcar
RUN ./controller
done


#sleep 2

#export CENTRALHOST=localhost:1381
#RUN ./perception_view
