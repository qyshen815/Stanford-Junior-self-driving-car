#!/bin/bash

while [ 1 ]
do
    free -m > log_free_`date +'%F_%T'`.txt;
    sleep $1
done
