#!/bin/bash
#./make_test.sh debug &&
#arm-none-eabi-gdb --command="debug.cmd"
gnome-terminal -- /bin/bash -c "cv bms/test; ./make_test.sh clean; ./make_test.sh ;./make_test.sh debug" & arm-none-eabi-gdb
#nemiver --remote=localhost:3333 --gdb-binary=/mnt/workspace/SeleX.JSC/Firmware/toolchain/gcc-arm-none-eabi-7-2017-q4-major/bin/arm-none-eabi-gdb bms.elf
#./make_test.sh debug &
#/mnt/workspace/SeleX.JSC/Firmware/toolchain/fedora19-arm-none-eabi-insight/insight/bin/arm-none-eabi-insight --eval-command="target remote localhost:3333" bms.elf
