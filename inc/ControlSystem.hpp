#ifndef CONTROLSYSTEM_HPP_
#define CONTROLSYSTEM_HPP_

#include <eeros/control/TimeDomain.hpp>
#include <eeros/core/Executor.hpp>
#include <eeros/control/PeripheralInput.hpp>
#include <eeros/control/Mux.hpp>
#include "customBlocks/FwKinOdom.hpp"
#include <eeros/control/DeMux.hpp>
#include "customBlocks/InvKin.hpp"
#include "customBlocks/Controller.hpp"
#include "customBlocks/InvMotMod.hpp"
#include <eeros/control/PeripheralOutput.hpp>

using namespace eeros::control;

class ControlSystem
{
public:
    ControlSystem(double dt);

    // Define Blocks
    PeripheralInput<> E1, E2;
    Mux<2> E;
    Mux<2> E_d;
    Controller<eeros::math::Vector2> controller;
    InvMotMod<eeros::math::Vector2> invMotMod;
    DeMux<2> U;
    PeripheralOutput<> M1, M2;

    TimeDomain timedomain;
};

#endif // CONTROLSYSTEM_HPP