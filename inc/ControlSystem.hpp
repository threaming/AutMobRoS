#ifndef CONTROLSYSTEM_HPP_
#define CONTROLSYSTEM_HPP_

#include <eeros/control/TimeDomain.hpp>
#include <eeros/core/Executor.hpp>
#include <eeros/control/PeripheralInput.hpp>
#include <eeros/control/Mux.hpp>
#include <eeros/control/D.hpp>
#include "customBlocks/FwKinOdom.hpp"
#include <eeros/control/DeMux.hpp>
#include <eeros/control/Constant.hpp>
#include "customBlocks/InvKin.hpp"
#include "customBlocks/InvMotMod.hpp"
#include <eeros/control/D.hpp>
#include <eeros/control/PeripheralOutput.hpp>

using namespace eeros::control;

class ControlSystem
{
public:
    ControlSystem(double dt);

    // Define Blocks
    PeripheralInput<> E1, E2;
    Sum<> e;
    Gain<> Kp;
    D<> ed;
    D<> q1d;
    Gain<> Kd;
    Sum<> qdd_c;
    Gain<> M;
    InvMotMod<> invMotMod;
    PeripheralOutput<> motor2;

    TimeDomain timedomain;
};

#endif // CONTROLSYSTEM_HPP