define reset
	mon reset halt
end

define stopd 
	monitor shutdown
	kill
	quit
end

define reloadd
	file bms.elf
	load
	thbreak main
end

target extended-remote localhost:3333
monitor reset halt
mon flash erase_address 0x08000000 0x00100000
mon reset halt
monitor flash protect 0 0 7 off
monitor flash info 0
monitor reset halt
file bms.elf
load
source test_breaks
continue

#target extended-remote localhost:3333
#mon reset halt
#mon flash erase_address 0x08000000 0x00100000
#mon reset halt
#monitor halt
#monitor flash protect 0 0 7 off
#monitor flash info 0
#file bms.elf
#load
#mon reset halt
#thbreak main
#source break_list
#continue


