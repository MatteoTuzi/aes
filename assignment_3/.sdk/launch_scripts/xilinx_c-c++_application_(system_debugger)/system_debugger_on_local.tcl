connect -url tcp:127.0.0.1:3121
source C:/Users/super/OneDrive/Desktop/Universita/aes/assignment_3/audio_lab.sdk/design_1_wrapper_hw_platform_1/ps7_init.tcl
targets -set -nocase -filter {name =~"APU*" && jtag_cable_name =~ "Digilent Zybo Z7 210351B0FC9BA"} -index 0
rst -system
after 3000
targets -set -filter {jtag_cable_name =~ "Digilent Zybo Z7 210351B0FC9BA" && level==0} -index 1
fpga -file C:/Users/super/OneDrive/Desktop/Universita/aes/assignment_3/audio_lab.sdk/design_1_wrapper_hw_platform_1/design_1_wrapper.bit
targets -set -nocase -filter {name =~"APU*" && jtag_cable_name =~ "Digilent Zybo Z7 210351B0FC9BA"} -index 0
loadhw -hw C:/Users/super/OneDrive/Desktop/Universita/aes/assignment_3/audio_lab.sdk/design_1_wrapper_hw_platform_1/system.hdf -mem-ranges [list {0x40000000 0xbfffffff}]
configparams force-mem-access 1
targets -set -nocase -filter {name =~"APU*" && jtag_cable_name =~ "Digilent Zybo Z7 210351B0FC9BA"} -index 0
ps7_init
ps7_post_config
targets -set -nocase -filter {name =~ "ARM*#0" && jtag_cable_name =~ "Digilent Zybo Z7 210351B0FC9BA"} -index 0
dow C:/Users/super/OneDrive/Desktop/Universita/aes/assignment_3/master/Debug/master.elf
targets -set -nocase -filter {name =~ "ARM*#1" && jtag_cable_name =~ "Digilent Zybo Z7 210351B0FC9BA"} -index 0
dow C:/Users/super/OneDrive/Desktop/Universita/aes/assignment_3/worker/Debug/worker.elf
configparams force-mem-access 0
targets -set -nocase -filter {name =~ "ARM*#0" && jtag_cable_name =~ "Digilent Zybo Z7 210351B0FC9BA"} -index 0
con
targets -set -nocase -filter {name =~ "ARM*#1" && jtag_cable_name =~ "Digilent Zybo Z7 210351B0FC9BA"} -index 0
con
