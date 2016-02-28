int16 group 	#set non zero to address a group
int16 bank 	#set non zero to address a bank
int16 led 	#set non zero to address specific led/channel
uint16[] data 	#values according to group/bank/led config

# either group or bank have to be non zero
# group has priority over bank

# group_brightness: group set, led not set 
# bank_brightness: bank set, led and group not set

#modes:
# group != 0 
# data[1]:
# 	led == 0: group_brightness 
# 	led != 0: set one channel
# data[group_size]: direct mapping

# bank != 0 
# data[1]:
# 	led == 0: bank_brightness 
# 	led != 0: set one channel
# data[bank_size]: direct mapping

# data[3]: for 3channel rgb leds
# 	led == 0: all leds to same rgb-color
# 	led != 0: set 3 channels starting at led