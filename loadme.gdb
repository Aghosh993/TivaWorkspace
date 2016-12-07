target remote localhost:2331
monitor speed 1000
monitor flash device =  LM4F232H5QD
monitor halt
monitor reset
file /home/aghosh/TM4C_Workspace/gcc/project.axf
load 
monitor reg pc = (0x00000004)
monitor reset
set confirm off
quit
