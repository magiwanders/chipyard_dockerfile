import siliconcompiler

chip = siliconcompiler.Chip('picorv32')     # create chip object
chip.load_target('skywater130_demo')        # load predefined target
chip.input('picorv32.v')                    # define list of source files
chip.clock('clk', period=10)                # define clock speed of design
chip.set('option', 'remote', False)          # run remote in the cloud
chip.run()                                  # run chip compilation
chip.summary()                              # print results summary with PNG screenshot and HTML report

