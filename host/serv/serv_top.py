import siliconcompiler

chip = siliconcompiler.Chip('serv_top')     # create chip object
chip.load_target('skywater130_demo')        # load predefined target
chip.input('serv_top.v')
chip.input('serv_aligner.v')
chip.input('serv_alu.v')
chip.input('serv_bufreg.v')
chip.input('serv_bufreg2.v')
chip.input('serv_compdec.v')
chip.input('serv_csr.v')
chip.input('serv_ctrl.v')
chip.input('serv_decode.v')
chip.input('serv_immdec.v')
chip.input('serv_mem_if.v')
chip.input('serv_rf_if.v')
chip.input('serv_rf_ram_if.v')
chip.input('serv_rf_ram.v')
chip.input('serv_rf_top.v')
chip.input('serv_state.v')
chip.input('serv_synth_wrapper.v')                    # define list of source files
chip.clock('clk', period=10)                # define clock speed of design
chip.set('option', 'remote', False)          # run remote in the cloud
chip.run()                                  # run chip compilation
chip.summary()                              # print results summary with PNG screenshot and HTML report

