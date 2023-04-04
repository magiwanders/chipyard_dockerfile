###############################################################################
# Created by write_sdc
# Tue Apr  4 21:55:06 2023
###############################################################################
current_design serv_top
###############################################################################
# Timing Constraints
###############################################################################
create_clock -name clk -period 10.0000 [get_ports {clk}]
set_clock_uncertainty 0.0000 clk
###############################################################################
# Environment
###############################################################################
set_load -pin_load 0.0231 [get_ports {o_dbus_cyc}]
set_load -pin_load 0.0231 [get_ports {o_dbus_we}]
set_load -pin_load 0.0231 [get_ports {o_ibus_cyc}]
set_load -pin_load 0.0231 [get_ports {o_mdu_valid}]
set_load -pin_load 0.0231 [get_ports {o_rf_rreq}]
set_load -pin_load 0.0231 [get_ports {o_rf_wreq}]
set_load -pin_load 0.0231 [get_ports {o_wdata0}]
set_load -pin_load 0.0231 [get_ports {o_wdata1}]
set_load -pin_load 0.0231 [get_ports {o_wen0}]
set_load -pin_load 0.0231 [get_ports {o_wen1}]
set_load -pin_load 0.0231 [get_ports {o_dbus_adr[31]}]
set_load -pin_load 0.0231 [get_ports {o_dbus_adr[30]}]
set_load -pin_load 0.0231 [get_ports {o_dbus_adr[29]}]
set_load -pin_load 0.0231 [get_ports {o_dbus_adr[28]}]
set_load -pin_load 0.0231 [get_ports {o_dbus_adr[27]}]
set_load -pin_load 0.0231 [get_ports {o_dbus_adr[26]}]
set_load -pin_load 0.0231 [get_ports {o_dbus_adr[25]}]
set_load -pin_load 0.0231 [get_ports {o_dbus_adr[24]}]
set_load -pin_load 0.0231 [get_ports {o_dbus_adr[23]}]
set_load -pin_load 0.0231 [get_ports {o_dbus_adr[22]}]
set_load -pin_load 0.0231 [get_ports {o_dbus_adr[21]}]
set_load -pin_load 0.0231 [get_ports {o_dbus_adr[20]}]
set_load -pin_load 0.0231 [get_ports {o_dbus_adr[19]}]
set_load -pin_load 0.0231 [get_ports {o_dbus_adr[18]}]
set_load -pin_load 0.0231 [get_ports {o_dbus_adr[17]}]
set_load -pin_load 0.0231 [get_ports {o_dbus_adr[16]}]
set_load -pin_load 0.0231 [get_ports {o_dbus_adr[15]}]
set_load -pin_load 0.0231 [get_ports {o_dbus_adr[14]}]
set_load -pin_load 0.0231 [get_ports {o_dbus_adr[13]}]
set_load -pin_load 0.0231 [get_ports {o_dbus_adr[12]}]
set_load -pin_load 0.0231 [get_ports {o_dbus_adr[11]}]
set_load -pin_load 0.0231 [get_ports {o_dbus_adr[10]}]
set_load -pin_load 0.0231 [get_ports {o_dbus_adr[9]}]
set_load -pin_load 0.0231 [get_ports {o_dbus_adr[8]}]
set_load -pin_load 0.0231 [get_ports {o_dbus_adr[7]}]
set_load -pin_load 0.0231 [get_ports {o_dbus_adr[6]}]
set_load -pin_load 0.0231 [get_ports {o_dbus_adr[5]}]
set_load -pin_load 0.0231 [get_ports {o_dbus_adr[4]}]
set_load -pin_load 0.0231 [get_ports {o_dbus_adr[3]}]
set_load -pin_load 0.0231 [get_ports {o_dbus_adr[2]}]
set_load -pin_load 0.0231 [get_ports {o_dbus_adr[1]}]
set_load -pin_load 0.0231 [get_ports {o_dbus_adr[0]}]
set_load -pin_load 0.0231 [get_ports {o_dbus_dat[31]}]
set_load -pin_load 0.0231 [get_ports {o_dbus_dat[30]}]
set_load -pin_load 0.0231 [get_ports {o_dbus_dat[29]}]
set_load -pin_load 0.0231 [get_ports {o_dbus_dat[28]}]
set_load -pin_load 0.0231 [get_ports {o_dbus_dat[27]}]
set_load -pin_load 0.0231 [get_ports {o_dbus_dat[26]}]
set_load -pin_load 0.0231 [get_ports {o_dbus_dat[25]}]
set_load -pin_load 0.0231 [get_ports {o_dbus_dat[24]}]
set_load -pin_load 0.0231 [get_ports {o_dbus_dat[23]}]
set_load -pin_load 0.0231 [get_ports {o_dbus_dat[22]}]
set_load -pin_load 0.0231 [get_ports {o_dbus_dat[21]}]
set_load -pin_load 0.0231 [get_ports {o_dbus_dat[20]}]
set_load -pin_load 0.0231 [get_ports {o_dbus_dat[19]}]
set_load -pin_load 0.0231 [get_ports {o_dbus_dat[18]}]
set_load -pin_load 0.0231 [get_ports {o_dbus_dat[17]}]
set_load -pin_load 0.0231 [get_ports {o_dbus_dat[16]}]
set_load -pin_load 0.0231 [get_ports {o_dbus_dat[15]}]
set_load -pin_load 0.0231 [get_ports {o_dbus_dat[14]}]
set_load -pin_load 0.0231 [get_ports {o_dbus_dat[13]}]
set_load -pin_load 0.0231 [get_ports {o_dbus_dat[12]}]
set_load -pin_load 0.0231 [get_ports {o_dbus_dat[11]}]
set_load -pin_load 0.0231 [get_ports {o_dbus_dat[10]}]
set_load -pin_load 0.0231 [get_ports {o_dbus_dat[9]}]
set_load -pin_load 0.0231 [get_ports {o_dbus_dat[8]}]
set_load -pin_load 0.0231 [get_ports {o_dbus_dat[7]}]
set_load -pin_load 0.0231 [get_ports {o_dbus_dat[6]}]
set_load -pin_load 0.0231 [get_ports {o_dbus_dat[5]}]
set_load -pin_load 0.0231 [get_ports {o_dbus_dat[4]}]
set_load -pin_load 0.0231 [get_ports {o_dbus_dat[3]}]
set_load -pin_load 0.0231 [get_ports {o_dbus_dat[2]}]
set_load -pin_load 0.0231 [get_ports {o_dbus_dat[1]}]
set_load -pin_load 0.0231 [get_ports {o_dbus_dat[0]}]
set_load -pin_load 0.0231 [get_ports {o_dbus_sel[3]}]
set_load -pin_load 0.0231 [get_ports {o_dbus_sel[2]}]
set_load -pin_load 0.0231 [get_ports {o_dbus_sel[1]}]
set_load -pin_load 0.0231 [get_ports {o_dbus_sel[0]}]
set_load -pin_load 0.0231 [get_ports {o_ext_funct3[2]}]
set_load -pin_load 0.0231 [get_ports {o_ext_funct3[1]}]
set_load -pin_load 0.0231 [get_ports {o_ext_funct3[0]}]
set_load -pin_load 0.0231 [get_ports {o_ext_rs1[31]}]
set_load -pin_load 0.0231 [get_ports {o_ext_rs1[30]}]
set_load -pin_load 0.0231 [get_ports {o_ext_rs1[29]}]
set_load -pin_load 0.0231 [get_ports {o_ext_rs1[28]}]
set_load -pin_load 0.0231 [get_ports {o_ext_rs1[27]}]
set_load -pin_load 0.0231 [get_ports {o_ext_rs1[26]}]
set_load -pin_load 0.0231 [get_ports {o_ext_rs1[25]}]
set_load -pin_load 0.0231 [get_ports {o_ext_rs1[24]}]
set_load -pin_load 0.0231 [get_ports {o_ext_rs1[23]}]
set_load -pin_load 0.0231 [get_ports {o_ext_rs1[22]}]
set_load -pin_load 0.0231 [get_ports {o_ext_rs1[21]}]
set_load -pin_load 0.0231 [get_ports {o_ext_rs1[20]}]
set_load -pin_load 0.0231 [get_ports {o_ext_rs1[19]}]
set_load -pin_load 0.0231 [get_ports {o_ext_rs1[18]}]
set_load -pin_load 0.0231 [get_ports {o_ext_rs1[17]}]
set_load -pin_load 0.0231 [get_ports {o_ext_rs1[16]}]
set_load -pin_load 0.0231 [get_ports {o_ext_rs1[15]}]
set_load -pin_load 0.0231 [get_ports {o_ext_rs1[14]}]
set_load -pin_load 0.0231 [get_ports {o_ext_rs1[13]}]
set_load -pin_load 0.0231 [get_ports {o_ext_rs1[12]}]
set_load -pin_load 0.0231 [get_ports {o_ext_rs1[11]}]
set_load -pin_load 0.0231 [get_ports {o_ext_rs1[10]}]
set_load -pin_load 0.0231 [get_ports {o_ext_rs1[9]}]
set_load -pin_load 0.0231 [get_ports {o_ext_rs1[8]}]
set_load -pin_load 0.0231 [get_ports {o_ext_rs1[7]}]
set_load -pin_load 0.0231 [get_ports {o_ext_rs1[6]}]
set_load -pin_load 0.0231 [get_ports {o_ext_rs1[5]}]
set_load -pin_load 0.0231 [get_ports {o_ext_rs1[4]}]
set_load -pin_load 0.0231 [get_ports {o_ext_rs1[3]}]
set_load -pin_load 0.0231 [get_ports {o_ext_rs1[2]}]
set_load -pin_load 0.0231 [get_ports {o_ext_rs1[1]}]
set_load -pin_load 0.0231 [get_ports {o_ext_rs1[0]}]
set_load -pin_load 0.0231 [get_ports {o_ext_rs2[31]}]
set_load -pin_load 0.0231 [get_ports {o_ext_rs2[30]}]
set_load -pin_load 0.0231 [get_ports {o_ext_rs2[29]}]
set_load -pin_load 0.0231 [get_ports {o_ext_rs2[28]}]
set_load -pin_load 0.0231 [get_ports {o_ext_rs2[27]}]
set_load -pin_load 0.0231 [get_ports {o_ext_rs2[26]}]
set_load -pin_load 0.0231 [get_ports {o_ext_rs2[25]}]
set_load -pin_load 0.0231 [get_ports {o_ext_rs2[24]}]
set_load -pin_load 0.0231 [get_ports {o_ext_rs2[23]}]
set_load -pin_load 0.0231 [get_ports {o_ext_rs2[22]}]
set_load -pin_load 0.0231 [get_ports {o_ext_rs2[21]}]
set_load -pin_load 0.0231 [get_ports {o_ext_rs2[20]}]
set_load -pin_load 0.0231 [get_ports {o_ext_rs2[19]}]
set_load -pin_load 0.0231 [get_ports {o_ext_rs2[18]}]
set_load -pin_load 0.0231 [get_ports {o_ext_rs2[17]}]
set_load -pin_load 0.0231 [get_ports {o_ext_rs2[16]}]
set_load -pin_load 0.0231 [get_ports {o_ext_rs2[15]}]
set_load -pin_load 0.0231 [get_ports {o_ext_rs2[14]}]
set_load -pin_load 0.0231 [get_ports {o_ext_rs2[13]}]
set_load -pin_load 0.0231 [get_ports {o_ext_rs2[12]}]
set_load -pin_load 0.0231 [get_ports {o_ext_rs2[11]}]
set_load -pin_load 0.0231 [get_ports {o_ext_rs2[10]}]
set_load -pin_load 0.0231 [get_ports {o_ext_rs2[9]}]
set_load -pin_load 0.0231 [get_ports {o_ext_rs2[8]}]
set_load -pin_load 0.0231 [get_ports {o_ext_rs2[7]}]
set_load -pin_load 0.0231 [get_ports {o_ext_rs2[6]}]
set_load -pin_load 0.0231 [get_ports {o_ext_rs2[5]}]
set_load -pin_load 0.0231 [get_ports {o_ext_rs2[4]}]
set_load -pin_load 0.0231 [get_ports {o_ext_rs2[3]}]
set_load -pin_load 0.0231 [get_ports {o_ext_rs2[2]}]
set_load -pin_load 0.0231 [get_ports {o_ext_rs2[1]}]
set_load -pin_load 0.0231 [get_ports {o_ext_rs2[0]}]
set_load -pin_load 0.0231 [get_ports {o_ibus_adr[31]}]
set_load -pin_load 0.0231 [get_ports {o_ibus_adr[30]}]
set_load -pin_load 0.0231 [get_ports {o_ibus_adr[29]}]
set_load -pin_load 0.0231 [get_ports {o_ibus_adr[28]}]
set_load -pin_load 0.0231 [get_ports {o_ibus_adr[27]}]
set_load -pin_load 0.0231 [get_ports {o_ibus_adr[26]}]
set_load -pin_load 0.0231 [get_ports {o_ibus_adr[25]}]
set_load -pin_load 0.0231 [get_ports {o_ibus_adr[24]}]
set_load -pin_load 0.0231 [get_ports {o_ibus_adr[23]}]
set_load -pin_load 0.0231 [get_ports {o_ibus_adr[22]}]
set_load -pin_load 0.0231 [get_ports {o_ibus_adr[21]}]
set_load -pin_load 0.0231 [get_ports {o_ibus_adr[20]}]
set_load -pin_load 0.0231 [get_ports {o_ibus_adr[19]}]
set_load -pin_load 0.0231 [get_ports {o_ibus_adr[18]}]
set_load -pin_load 0.0231 [get_ports {o_ibus_adr[17]}]
set_load -pin_load 0.0231 [get_ports {o_ibus_adr[16]}]
set_load -pin_load 0.0231 [get_ports {o_ibus_adr[15]}]
set_load -pin_load 0.0231 [get_ports {o_ibus_adr[14]}]
set_load -pin_load 0.0231 [get_ports {o_ibus_adr[13]}]
set_load -pin_load 0.0231 [get_ports {o_ibus_adr[12]}]
set_load -pin_load 0.0231 [get_ports {o_ibus_adr[11]}]
set_load -pin_load 0.0231 [get_ports {o_ibus_adr[10]}]
set_load -pin_load 0.0231 [get_ports {o_ibus_adr[9]}]
set_load -pin_load 0.0231 [get_ports {o_ibus_adr[8]}]
set_load -pin_load 0.0231 [get_ports {o_ibus_adr[7]}]
set_load -pin_load 0.0231 [get_ports {o_ibus_adr[6]}]
set_load -pin_load 0.0231 [get_ports {o_ibus_adr[5]}]
set_load -pin_load 0.0231 [get_ports {o_ibus_adr[4]}]
set_load -pin_load 0.0231 [get_ports {o_ibus_adr[3]}]
set_load -pin_load 0.0231 [get_ports {o_ibus_adr[2]}]
set_load -pin_load 0.0231 [get_ports {o_ibus_adr[1]}]
set_load -pin_load 0.0231 [get_ports {o_ibus_adr[0]}]
set_load -pin_load 0.0231 [get_ports {o_rreg0[5]}]
set_load -pin_load 0.0231 [get_ports {o_rreg0[4]}]
set_load -pin_load 0.0231 [get_ports {o_rreg0[3]}]
set_load -pin_load 0.0231 [get_ports {o_rreg0[2]}]
set_load -pin_load 0.0231 [get_ports {o_rreg0[1]}]
set_load -pin_load 0.0231 [get_ports {o_rreg0[0]}]
set_load -pin_load 0.0231 [get_ports {o_rreg1[5]}]
set_load -pin_load 0.0231 [get_ports {o_rreg1[4]}]
set_load -pin_load 0.0231 [get_ports {o_rreg1[3]}]
set_load -pin_load 0.0231 [get_ports {o_rreg1[2]}]
set_load -pin_load 0.0231 [get_ports {o_rreg1[1]}]
set_load -pin_load 0.0231 [get_ports {o_rreg1[0]}]
set_load -pin_load 0.0231 [get_ports {o_wreg0[5]}]
set_load -pin_load 0.0231 [get_ports {o_wreg0[4]}]
set_load -pin_load 0.0231 [get_ports {o_wreg0[3]}]
set_load -pin_load 0.0231 [get_ports {o_wreg0[2]}]
set_load -pin_load 0.0231 [get_ports {o_wreg0[1]}]
set_load -pin_load 0.0231 [get_ports {o_wreg0[0]}]
set_load -pin_load 0.0231 [get_ports {o_wreg1[5]}]
set_load -pin_load 0.0231 [get_ports {o_wreg1[4]}]
set_load -pin_load 0.0231 [get_ports {o_wreg1[3]}]
set_load -pin_load 0.0231 [get_ports {o_wreg1[2]}]
set_load -pin_load 0.0231 [get_ports {o_wreg1[1]}]
set_load -pin_load 0.0231 [get_ports {o_wreg1[0]}]
set_driving_cell -lib_cell sky130_fd_sc_hd__buf_1 -pin {X} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {clk}]
set_driving_cell -lib_cell sky130_fd_sc_hd__buf_1 -pin {X} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {i_dbus_ack}]
set_driving_cell -lib_cell sky130_fd_sc_hd__buf_1 -pin {X} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {i_ext_ready}]
set_driving_cell -lib_cell sky130_fd_sc_hd__buf_1 -pin {X} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {i_ibus_ack}]
set_driving_cell -lib_cell sky130_fd_sc_hd__buf_1 -pin {X} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {i_rdata0}]
set_driving_cell -lib_cell sky130_fd_sc_hd__buf_1 -pin {X} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {i_rdata1}]
set_driving_cell -lib_cell sky130_fd_sc_hd__buf_1 -pin {X} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {i_rf_ready}]
set_driving_cell -lib_cell sky130_fd_sc_hd__buf_1 -pin {X} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {i_rst}]
set_driving_cell -lib_cell sky130_fd_sc_hd__buf_1 -pin {X} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {i_timer_irq}]
set_driving_cell -lib_cell sky130_fd_sc_hd__buf_1 -pin {X} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {i_dbus_rdt[31]}]
set_driving_cell -lib_cell sky130_fd_sc_hd__buf_1 -pin {X} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {i_dbus_rdt[30]}]
set_driving_cell -lib_cell sky130_fd_sc_hd__buf_1 -pin {X} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {i_dbus_rdt[29]}]
set_driving_cell -lib_cell sky130_fd_sc_hd__buf_1 -pin {X} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {i_dbus_rdt[28]}]
set_driving_cell -lib_cell sky130_fd_sc_hd__buf_1 -pin {X} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {i_dbus_rdt[27]}]
set_driving_cell -lib_cell sky130_fd_sc_hd__buf_1 -pin {X} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {i_dbus_rdt[26]}]
set_driving_cell -lib_cell sky130_fd_sc_hd__buf_1 -pin {X} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {i_dbus_rdt[25]}]
set_driving_cell -lib_cell sky130_fd_sc_hd__buf_1 -pin {X} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {i_dbus_rdt[24]}]
set_driving_cell -lib_cell sky130_fd_sc_hd__buf_1 -pin {X} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {i_dbus_rdt[23]}]
set_driving_cell -lib_cell sky130_fd_sc_hd__buf_1 -pin {X} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {i_dbus_rdt[22]}]
set_driving_cell -lib_cell sky130_fd_sc_hd__buf_1 -pin {X} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {i_dbus_rdt[21]}]
set_driving_cell -lib_cell sky130_fd_sc_hd__buf_1 -pin {X} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {i_dbus_rdt[20]}]
set_driving_cell -lib_cell sky130_fd_sc_hd__buf_1 -pin {X} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {i_dbus_rdt[19]}]
set_driving_cell -lib_cell sky130_fd_sc_hd__buf_1 -pin {X} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {i_dbus_rdt[18]}]
set_driving_cell -lib_cell sky130_fd_sc_hd__buf_1 -pin {X} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {i_dbus_rdt[17]}]
set_driving_cell -lib_cell sky130_fd_sc_hd__buf_1 -pin {X} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {i_dbus_rdt[16]}]
set_driving_cell -lib_cell sky130_fd_sc_hd__buf_1 -pin {X} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {i_dbus_rdt[15]}]
set_driving_cell -lib_cell sky130_fd_sc_hd__buf_1 -pin {X} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {i_dbus_rdt[14]}]
set_driving_cell -lib_cell sky130_fd_sc_hd__buf_1 -pin {X} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {i_dbus_rdt[13]}]
set_driving_cell -lib_cell sky130_fd_sc_hd__buf_1 -pin {X} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {i_dbus_rdt[12]}]
set_driving_cell -lib_cell sky130_fd_sc_hd__buf_1 -pin {X} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {i_dbus_rdt[11]}]
set_driving_cell -lib_cell sky130_fd_sc_hd__buf_1 -pin {X} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {i_dbus_rdt[10]}]
set_driving_cell -lib_cell sky130_fd_sc_hd__buf_1 -pin {X} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {i_dbus_rdt[9]}]
set_driving_cell -lib_cell sky130_fd_sc_hd__buf_1 -pin {X} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {i_dbus_rdt[8]}]
set_driving_cell -lib_cell sky130_fd_sc_hd__buf_1 -pin {X} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {i_dbus_rdt[7]}]
set_driving_cell -lib_cell sky130_fd_sc_hd__buf_1 -pin {X} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {i_dbus_rdt[6]}]
set_driving_cell -lib_cell sky130_fd_sc_hd__buf_1 -pin {X} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {i_dbus_rdt[5]}]
set_driving_cell -lib_cell sky130_fd_sc_hd__buf_1 -pin {X} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {i_dbus_rdt[4]}]
set_driving_cell -lib_cell sky130_fd_sc_hd__buf_1 -pin {X} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {i_dbus_rdt[3]}]
set_driving_cell -lib_cell sky130_fd_sc_hd__buf_1 -pin {X} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {i_dbus_rdt[2]}]
set_driving_cell -lib_cell sky130_fd_sc_hd__buf_1 -pin {X} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {i_dbus_rdt[1]}]
set_driving_cell -lib_cell sky130_fd_sc_hd__buf_1 -pin {X} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {i_dbus_rdt[0]}]
set_driving_cell -lib_cell sky130_fd_sc_hd__buf_1 -pin {X} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {i_ext_rd[31]}]
set_driving_cell -lib_cell sky130_fd_sc_hd__buf_1 -pin {X} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {i_ext_rd[30]}]
set_driving_cell -lib_cell sky130_fd_sc_hd__buf_1 -pin {X} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {i_ext_rd[29]}]
set_driving_cell -lib_cell sky130_fd_sc_hd__buf_1 -pin {X} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {i_ext_rd[28]}]
set_driving_cell -lib_cell sky130_fd_sc_hd__buf_1 -pin {X} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {i_ext_rd[27]}]
set_driving_cell -lib_cell sky130_fd_sc_hd__buf_1 -pin {X} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {i_ext_rd[26]}]
set_driving_cell -lib_cell sky130_fd_sc_hd__buf_1 -pin {X} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {i_ext_rd[25]}]
set_driving_cell -lib_cell sky130_fd_sc_hd__buf_1 -pin {X} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {i_ext_rd[24]}]
set_driving_cell -lib_cell sky130_fd_sc_hd__buf_1 -pin {X} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {i_ext_rd[23]}]
set_driving_cell -lib_cell sky130_fd_sc_hd__buf_1 -pin {X} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {i_ext_rd[22]}]
set_driving_cell -lib_cell sky130_fd_sc_hd__buf_1 -pin {X} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {i_ext_rd[21]}]
set_driving_cell -lib_cell sky130_fd_sc_hd__buf_1 -pin {X} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {i_ext_rd[20]}]
set_driving_cell -lib_cell sky130_fd_sc_hd__buf_1 -pin {X} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {i_ext_rd[19]}]
set_driving_cell -lib_cell sky130_fd_sc_hd__buf_1 -pin {X} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {i_ext_rd[18]}]
set_driving_cell -lib_cell sky130_fd_sc_hd__buf_1 -pin {X} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {i_ext_rd[17]}]
set_driving_cell -lib_cell sky130_fd_sc_hd__buf_1 -pin {X} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {i_ext_rd[16]}]
set_driving_cell -lib_cell sky130_fd_sc_hd__buf_1 -pin {X} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {i_ext_rd[15]}]
set_driving_cell -lib_cell sky130_fd_sc_hd__buf_1 -pin {X} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {i_ext_rd[14]}]
set_driving_cell -lib_cell sky130_fd_sc_hd__buf_1 -pin {X} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {i_ext_rd[13]}]
set_driving_cell -lib_cell sky130_fd_sc_hd__buf_1 -pin {X} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {i_ext_rd[12]}]
set_driving_cell -lib_cell sky130_fd_sc_hd__buf_1 -pin {X} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {i_ext_rd[11]}]
set_driving_cell -lib_cell sky130_fd_sc_hd__buf_1 -pin {X} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {i_ext_rd[10]}]
set_driving_cell -lib_cell sky130_fd_sc_hd__buf_1 -pin {X} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {i_ext_rd[9]}]
set_driving_cell -lib_cell sky130_fd_sc_hd__buf_1 -pin {X} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {i_ext_rd[8]}]
set_driving_cell -lib_cell sky130_fd_sc_hd__buf_1 -pin {X} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {i_ext_rd[7]}]
set_driving_cell -lib_cell sky130_fd_sc_hd__buf_1 -pin {X} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {i_ext_rd[6]}]
set_driving_cell -lib_cell sky130_fd_sc_hd__buf_1 -pin {X} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {i_ext_rd[5]}]
set_driving_cell -lib_cell sky130_fd_sc_hd__buf_1 -pin {X} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {i_ext_rd[4]}]
set_driving_cell -lib_cell sky130_fd_sc_hd__buf_1 -pin {X} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {i_ext_rd[3]}]
set_driving_cell -lib_cell sky130_fd_sc_hd__buf_1 -pin {X} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {i_ext_rd[2]}]
set_driving_cell -lib_cell sky130_fd_sc_hd__buf_1 -pin {X} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {i_ext_rd[1]}]
set_driving_cell -lib_cell sky130_fd_sc_hd__buf_1 -pin {X} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {i_ext_rd[0]}]
set_driving_cell -lib_cell sky130_fd_sc_hd__buf_1 -pin {X} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {i_ibus_rdt[31]}]
set_driving_cell -lib_cell sky130_fd_sc_hd__buf_1 -pin {X} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {i_ibus_rdt[30]}]
set_driving_cell -lib_cell sky130_fd_sc_hd__buf_1 -pin {X} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {i_ibus_rdt[29]}]
set_driving_cell -lib_cell sky130_fd_sc_hd__buf_1 -pin {X} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {i_ibus_rdt[28]}]
set_driving_cell -lib_cell sky130_fd_sc_hd__buf_1 -pin {X} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {i_ibus_rdt[27]}]
set_driving_cell -lib_cell sky130_fd_sc_hd__buf_1 -pin {X} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {i_ibus_rdt[26]}]
set_driving_cell -lib_cell sky130_fd_sc_hd__buf_1 -pin {X} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {i_ibus_rdt[25]}]
set_driving_cell -lib_cell sky130_fd_sc_hd__buf_1 -pin {X} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {i_ibus_rdt[24]}]
set_driving_cell -lib_cell sky130_fd_sc_hd__buf_1 -pin {X} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {i_ibus_rdt[23]}]
set_driving_cell -lib_cell sky130_fd_sc_hd__buf_1 -pin {X} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {i_ibus_rdt[22]}]
set_driving_cell -lib_cell sky130_fd_sc_hd__buf_1 -pin {X} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {i_ibus_rdt[21]}]
set_driving_cell -lib_cell sky130_fd_sc_hd__buf_1 -pin {X} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {i_ibus_rdt[20]}]
set_driving_cell -lib_cell sky130_fd_sc_hd__buf_1 -pin {X} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {i_ibus_rdt[19]}]
set_driving_cell -lib_cell sky130_fd_sc_hd__buf_1 -pin {X} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {i_ibus_rdt[18]}]
set_driving_cell -lib_cell sky130_fd_sc_hd__buf_1 -pin {X} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {i_ibus_rdt[17]}]
set_driving_cell -lib_cell sky130_fd_sc_hd__buf_1 -pin {X} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {i_ibus_rdt[16]}]
set_driving_cell -lib_cell sky130_fd_sc_hd__buf_1 -pin {X} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {i_ibus_rdt[15]}]
set_driving_cell -lib_cell sky130_fd_sc_hd__buf_1 -pin {X} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {i_ibus_rdt[14]}]
set_driving_cell -lib_cell sky130_fd_sc_hd__buf_1 -pin {X} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {i_ibus_rdt[13]}]
set_driving_cell -lib_cell sky130_fd_sc_hd__buf_1 -pin {X} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {i_ibus_rdt[12]}]
set_driving_cell -lib_cell sky130_fd_sc_hd__buf_1 -pin {X} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {i_ibus_rdt[11]}]
set_driving_cell -lib_cell sky130_fd_sc_hd__buf_1 -pin {X} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {i_ibus_rdt[10]}]
set_driving_cell -lib_cell sky130_fd_sc_hd__buf_1 -pin {X} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {i_ibus_rdt[9]}]
set_driving_cell -lib_cell sky130_fd_sc_hd__buf_1 -pin {X} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {i_ibus_rdt[8]}]
set_driving_cell -lib_cell sky130_fd_sc_hd__buf_1 -pin {X} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {i_ibus_rdt[7]}]
set_driving_cell -lib_cell sky130_fd_sc_hd__buf_1 -pin {X} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {i_ibus_rdt[6]}]
set_driving_cell -lib_cell sky130_fd_sc_hd__buf_1 -pin {X} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {i_ibus_rdt[5]}]
set_driving_cell -lib_cell sky130_fd_sc_hd__buf_1 -pin {X} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {i_ibus_rdt[4]}]
set_driving_cell -lib_cell sky130_fd_sc_hd__buf_1 -pin {X} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {i_ibus_rdt[3]}]
set_driving_cell -lib_cell sky130_fd_sc_hd__buf_1 -pin {X} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {i_ibus_rdt[2]}]
set_driving_cell -lib_cell sky130_fd_sc_hd__buf_1 -pin {X} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {i_ibus_rdt[1]}]
set_driving_cell -lib_cell sky130_fd_sc_hd__buf_1 -pin {X} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {i_ibus_rdt[0]}]
###############################################################################
# Design Rules
###############################################################################
