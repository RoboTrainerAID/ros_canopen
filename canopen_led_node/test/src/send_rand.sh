#!/bin/bash

RANGE=150
RANGECHANNEL=15
RANGEBANK=4

echo "start"
COUNTER=0
while [  $COUNTER -lt 200 ]; do

number1=$RANDOM
let "number1 %= $RANGE"

number2=$RANDOM
let "number2 %= $RANGE"

number3=$RANDOM
let "number3 %= $RANGE"

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
