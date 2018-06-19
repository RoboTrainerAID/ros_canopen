#!/bin/bash


echo "start"
COUNTER=1
while [  $COUNTER -lt 5 ]; do
   echo $COUNTER
   rosrun canopen_led_node send 0 $COUNTER 1 100 100 100 
   let COUNTER=COUNTER+1
done

echo "end"

echo "start GlobalMapping"
