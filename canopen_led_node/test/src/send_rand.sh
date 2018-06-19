#!/bin/bash

RANGE=100
RANGECHANNEL=15
RANGEBANK=4

echo "start"
COUNTER=0
while [  $COUNTER -lt 200 ]; do

number1=$RANDOM
let "number1 %= $RANGE"
number1+=50

number2=$RANDOM
let "number2 %= $RANGE"
number2+=50

number3=$RANDOM
let "number3 %= $RANGE"
number3+=50

channel=$RANDOM
let "channel %= $RANGECHANNEL"
channel=$channel+1

bank=$RANDOM
let "bank %= $RANGEBANK"
bank=$bank+1
   rosrun canopen_led_node send 0 $bank $channel $number1
   let COUNTER=COUNTER+1
done

echo "end"

echo "start GlobalMapping"
