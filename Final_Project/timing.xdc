create_clock -period 13.000 -name clk -waveform {0.000 6.500} -add [get_ports Clk]
set_input_delay -clock [get_clocks *] -add_delay 6.500 [get_ports {flagChannelorData in_valid {InData[0]} {InData[1]} {InData[2]} {InData[3]} {InData[4]} {InData[5]} {InData[6]} {InData[7]} {InData[8]} {InData[9]} {InData[10]} {InData[11]} {InData[12]} {InData[13]} {InData[14]} {InData[15]} {InData[16]} {InData[17]} {InData[18]} {InData[19]} {InData[20]} {InData[21]} {InData[22]} {InData[23]} {InData[24]} {InData[25]} {InData[26]} {InData[27]} {InData[28]} {InData[29]} {InData[30]} {InData[31]} {InData[32]} {InData[33]} {InData[34]} {InData[35]} {InData[36]} {InData[37]} {InData[38]} {InData[39]} {InData[40]} {InData[41]} {InData[42]} {InData[43]} {InData[44]} {InData[45]} {InData[46]} {InData[47]} {InData[48]} {InData[49]} {InData[50]} {InData[51]} {InData[52]} {InData[53]} {InData[54]} {InData[55]} {InData[56]} {InData[57]} {InData[58]} {InData[59]} {InData[60]} {InData[61]} {InData[62]} {InData[63]} {InData[64]} {InData[65]} {InData[66]} {InData[67]} {InData[68]} {InData[69]} {InData[70]} {InData[71]} {InData[72]} {InData[73]} {InData[74]} {InData[75]} {InData[76]} {InData[77]} {InData[78]} {InData[79]} Reset}]
set_output_delay -clock [get_clocks *] -add_delay 1.000 [get_ports -filter { NAME =~  "*" && DIRECTION == "OUT" }]








