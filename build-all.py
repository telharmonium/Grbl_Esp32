#!/usr/bin/env python2

# Note: If you experience random errors running this script within
# VSCode, try running it from a regular terminal window.  Some VSCode
# extensions seem to randomly interfere with the files that platformio
# uses during compilation.

# Compile Grbl_ESP32 for each of the machines defined in Machines/ .
# Add-on files are built on top of a single base.
# This is useful for automated testing, to make sure you haven't broken something

# The output is filtered so that the only lines you see are a single
# success or failure line for each build, plus any preceding lines that
# contain the word "error".  If you need to see everything, for example to
# see the details of an errored build, include -v on the command line.

from builder import buildMachine
import os, sys

cmd = ['platformio','run']

verbose = '-v' in sys.argv

numErrors = 0
adderBase = '3axis_v4.h'
for name in os.listdir('Grbl_Esp32/Machines'):
    if name.startswith('add_'):
        exitCode = buildMachine(adderBase, addName=name, verbose=verbose)
    else:
        exitCode = buildMachine(name, verbose=verbose)
    if exitCode != 0:
        numErrors += 1

sys.exit(numErrors)
