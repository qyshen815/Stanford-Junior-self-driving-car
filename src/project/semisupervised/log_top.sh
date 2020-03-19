#!/bin/bash

while [ 1 ]
do
    top -n1 > log_top_`date +'%F_%T'`.txt;
    sleep $1
done
