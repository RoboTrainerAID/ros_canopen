#either group or bank have to be non zero
#group has priority over bank

int16 group 	#set non zero to address a group
int16 bank 	#set non zero to address a bank
int16 led 	#set non zero to address specific led/channel
uint16[] data 	#value according to group/bank/led config
#e.g. group = 1, led = 1: first group, first subindex 
# => 3 values for rgb of first channel
#
# group = 0, bank = 2, led = 0: second bank, all subindices
# => either 3 values to set whole bank to those values
# or [bank_size] values, to set different values to more than 1 led at a time
