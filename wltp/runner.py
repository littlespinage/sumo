#!/usr/bin/env python
# Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
# Copyright (C) 2008-2018 German Aerospace Center (DLR) and others.
# This program and the accompanying materials
# are made available under the terms of the Eclipse Public License v2.0
# which accompanies this distribution, and is available at
# http://www.eclipse.org/legal/epl-v20.html
# SPDX-License-Identifier: EPL-2.0

# @file    runner.py
# @author  Daniel Krajzewicz
# @author  Michael Behrisch
# @date    2007-10-25
# @version $Id$

from __future__ import absolute_import
from __future__ import print_function

import os
import subprocess
import sys

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

from sumolib import checkBinary  # noqa
import traci  # noqa

def run():
    step = 0
    for x in open("data.txt", "r"):
        traci.simulationStep()
        traci.vehicle.setSpeed("veh0", float(x))
        step = step + 1
    traci.close()
    sys.stdout.flush()

if __name__ == "__main__":
    traci.start(['sumo', "-c", "data/hello.sumocfg"])
    run()
