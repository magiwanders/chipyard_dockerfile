import siliconcompiler

chip = siliconcompiler.Chip('chipyard')     # create chip object
chip.load_target('skywater130_demo')        # load predefined target
chip.input('plusarg_reader.v')                    # define list of source files
chip.input('chipyard.TestHarness.RocketConfig.top.v')
chip.input('chipyard.TestHarness.RocketConfig.top.mems.v')
chip.input('chipyard.TestHarness.RocketConfig.harness.v')
chip.input('chipyard.TestHarness.RocketConfig.harness.mems.v')
chip.input('SimUART.v')
chip.input('SimSerial.v')
chip.input('SimJTAG.v')
chip.input('SimDRAM.v')
chip.input('IOCell.v')
chip.input('EICG_wrapper.v')
chip.input('ClockDividerN.sv')
chip.clock('clk', period=10)                # define clock speed of design
chip.set('option', 'remote', False)          # run remote in the cloud
chip.run()                                  # run chip compilation
chip.summary()                              # print results summary with PNG screenshot and HTML report

