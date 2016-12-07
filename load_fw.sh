#!/bin/bash
$1/JLinkGDBServer &
/usr/bin/arm-none-eabi-gdb -x loadme.gdb
pkill JLinkGDBServer
