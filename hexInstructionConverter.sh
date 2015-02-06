#!/bin/bash

##############################################################################
#	Script file used to convert binary program to hexadecimal instruction format
# ( used for debug purpose )
###############################################################################

if [ "$#" -ne 1 ]; then
    echo "Syntax error, please tape : ./hexInstructionConverter.sh mybinaryFile.bin"
    exit
fi 

PROG=$(cat $1)

for inst in $PROG;
do
		printf '0x%x\n' "$((2#$inst))"
    #echo $i
done    
