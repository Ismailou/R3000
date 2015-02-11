#!/bin/bash

##############################################################################
#	Script file used to convert binary program to hexadecimal instruction format
# ( used for debug purpose )
###############################################################################

if [ "$#" -ne 2 ]; then
    echo "Syntax error, please tape : ./hexInstructionConverter.sh myProgram.asm mybinary.bin"
    exit
fi 

IFS=$'\n'
arr=$(cat $1)
count=0
instruction[0]=010

for i in ${arr[@]};
do
	instruction[$count]=$i;
	count=$(( $count + 1 ))
done

PROG=$(cat $2)
count=0

for inst in $PROG;
do
	printf '@CP %d: ' "${count}"
		printf '%s\t|' "${instruction[$count]}"
		printf '0x%x\n' "$((2#$inst))"
    count=$(( $count + 1 ))
done    
