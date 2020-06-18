/****************************************************************************/
// Eclipse SUMO, Simulation of Urban MObility; see https://eclipse.org/sumo
// Copyright (C) 2001-2018 German Aerospace Center (DLR) and others.
// This program and the accompanying materials
// are made available under the terms of the Eclipse Public License v2.0
// which accompanies this distribution, and is available at
// http://www.eclipse.org/legal/epl-v20.html
// SPDX-License-Identifier: EPL-2.0
/****************************************************************************/
/// @file    HelpersEnergy.cpp
/// @author  Daniel Krajzewicz
/// @author  Jakob Erdmann
/// @author  Michael Behrisch
/// @date    Mon, 10.05.2004
/// @version $Id$
///
// Helper methods for HBEFA-based emission computation
/****************************************************************************/
#include <fstream>

// ===========================================================================
// included modules
// ===========================================================================
#include <config.h>

#include <utils/common/SUMOTime.h>
#include <utils/common/ToString.h>
#include "HelpersEnergy.h"

//============================================================================
//definition--Junjie Gu
//============================================================================
#define wheelRadius 0.32
#define gearRatio 9.0
#define maxTorque 280
#define maxSpeed 150

// ===========================================================================
// method definitions
// ===========================================================================
HelpersEnergy::HelpersEnergy() : PollutantsInterface::Helper("Energy") {
    myEmissionClassStrings.insert("zero", PollutantsInterface::ZERO_EMISSIONS);
    myEmissionClassStrings.insert("unknown", ENERGY_BASE);
    myDefaultParameter[SUMO_ATTR_VEHICLEMASS] = 1595;
    myDefaultParameter[SUMO_ATTR_FRONTSURFACEAREA] = 2.27;
    myDefaultParameter[SUMO_ATTR_AIRDRAGCOEFFICIENT] = 0.28;
    myDefaultParameter[SUMO_ATTR_INTERNALMOMENTOFINERTIA] = 0.0;
    myDefaultParameter[SUMO_ATTR_RADIALDRAGCOEFFICIENT] = 0.0;
    myDefaultParameter[SUMO_ATTR_ROLLDRAGCOEFFICIENT] = 0.0;
    myDefaultParameter[SUMO_ATTR_CONSTANTPOWERINTAKE] = 0.0;
    myDefaultParameter[SUMO_ATTR_PROPULSIONEFFICIENCY] = 0.5;
    myDefaultParameter[SUMO_ATTR_RECUPERATIONEFFICIENCY] = 0.;
    myDefaultParameter[SUMO_ATTR_ANGLE] = 0.;
}


double
HelpersEnergy::compute(const SUMOEmissionClass /* c */, const PollutantsInterface::EmissionType e, const double v, const double a, const double slope, const std::map<int, double>* param) const {
    if (e != PollutantsInterface::ELEC) {
        return 0.;
    }
    if (param == nullptr) {
        param = &myDefaultParameter;
    }
    //@ToDo: All formulas below work with the logic of the euler update (refs #860).
    //       Approximation order could be improved. Refs. #2592.

    const double mass = param->find(SUMO_ATTR_VEHICLEMASS)->second;

    double tractionForce = 0;
    // climbing force 
    tractionForce = mass * 9.81 * sin(DEG2RAD(slope));

    // acceleration force
    tractionForce += mass * a * 1.2425;

    // Energy loss through Air resistance [Ws]
    // Calculate energy losses:
    // EnergyLoss,Air = 1/2 * rho_air [kg/m^3] * myFrontSurfaceArea [m^2] * myAirDragCoefficient [-] * v_Veh^2 [m/s] * s [m]
    //                    ... with rho_air [kg/m^3] = 1,2041 kg/m^3 (at T = 20C)
    //                    ... with s [m] = v_Veh [m/s] * TS [s]
    tractionForce += 0.5 * 1.2041 * param->find(SUMO_ATTR_FRONTSURFACEAREA)->second * param->find(SUMO_ATTR_AIRDRAGCOEFFICIENT)->second * v * v;

    // Energy loss through Roll resistance [Ws]
    //                    ... (fabs(veh.getSpeed())>=0.01) = 0, if vehicle isn't moving
    // EnergyLoss,Tire = c_R [-] * F_N [N] * s [m]
    //                    ... with c_R = ~0.012    (car tire on asphalt)
    //                    ... with F_N [N] = myMass [kg] * g [m/s^2]
    tractionForce += param->find(SUMO_ATTR_ROLLDRAGCOEFFICIENT)->second * 9.81 * mass;
    
    // EnergyLoss,constantConsumers
    // Energy loss through constant loads (e.g. A/C) [Ws]
    double energyDiff = param->find(SUMO_ATTR_CONSTANTPOWERINTAKE)->second;
/*
==========================================================================================
motor efficiency                                                                           
==========================================================================================
*/
    double EfficiencyMatrix[43][26];
    std::ifstream data("efficiencyMap.txt");

    double d;
    for(int i = 0; i < 43; i++){
        for(int j = 0; j < 26; j++){
            data >> d;
            EfficiencyMatrix[i][j] = d;
        }
    }
    data.close();

    double MotorTorque = tractionForce * wheelRadius / gearRatio;
    int TorqueIndex = static_cast<int> (MotorTorque / maxTorque * 21 + 21);
    int SpeedIndex =  static_cast<int> (v * 3.6 / maxSpeed * 25);
    double motorEfficiency = EfficiencyMatrix[TorqueIndex][SpeedIndex] / 100.0;  

    if (tractionForce > 0) {
        // Assumption: Efficiency of myPropulsionEfficiency when accelerating
        energyDiff = tractionForce * SPEED2DIST(v) / motorEfficiency;
    } else {
        // Assumption: Efficiency of myRecuperationEfficiency when recuperating
        energyDiff = tractionForce * SPEED2DIST(v) * motorEfficiency;
    }

    // convert from [Ws] to [Wh] (3600s / 1h):
    return energyDiff/3600;
}


/****************************************************************************/
