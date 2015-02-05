


proc c  {} {
    vcom cpu_package.2.vhd
		vcom memory.1.vhd
		vcom registres.1.vhd
		vcom risc.0.vhd
    vcom test_risc.0.vhd  
    echo "---------------COMPILATION IS OVER--------------------------"
    #vsim work.test_risc.0
    vsim -voptargs=+acc work.test_risc(behavior)
    echo "---------------STARTED SIMULATION---------------------------"
    add wave -r -radix hexadecimal sim:/test_risc/*
    run 100ps #reset
    force -freeze sim:/test_risc/r3k/regf/REGS(1) 00000000000000000000000000000001 0
    force -freeze sim:/test_risc/r3k/regf/REGS(2) 00000000000000000000000000000010 0
    force -freeze sim:/test_risc/r3k/regf/REGS(3) 00000000000000000000000000000011 0
    force -freeze sim:/test_risc/r3k/regf/REGS(4) 00000000000000000000000000000100 0
    force -freeze sim:/test_risc/r3k/regf/REGS(5) 00000000000000000000000000000101 0
    force -freeze sim:/test_risc/r3k/regf/REGS(6) 00000000000000000000000000000110 0
    force -freeze sim:/test_risc/r3k/regf/REGS(7) 00000000000000000000000000000111 0
    force -freeze sim:/test_risc/r3k/regf/REGS(8) 00000000000000000000000000001000 0
    force -freeze sim:/test_risc/r3k/regf/REGS(9) 00000000000000000000000000001001 0
    force -freeze sim:/test_risc/r3k/regf/REGS(10) 00000000000000000000000000001010 0
}
 
proc r  {} {
    vcom cpu_package.2.vhd
		vcom memory.1.vhd
		vcom registres.1.vhd
		vcom risc.0.vhd
    vcom test_risc.0.vhd   
    echo "---------------COMPILATION IS OVER--------------------------"
    restart
    echo "---------------SIMULATION IS RESTARTED----------------------"    
    run 100ps#reset
}
proc q  {} {
 quit -sim
}
 
proc Q  {} {
 quit -force
}
