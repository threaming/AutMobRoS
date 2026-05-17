#include "ControlSystem.hpp"

ControlSystem::ControlSystem(double dt)
    : E1("enc1"),
      E2("enc2"),
      g(21.2 / 2.0 / M_PI),
      VelCtrl(1 / dt, 0.7, 2.3, 3441.0 / 104.0 / 0.04 * 3441.0 / 104.0 / 0.04 * 6.8e-8, 0.1),
      invMotMod(0.1 / 0.04, 21.2 * 0.04, 3441.0 / 104.0 / 0.04, 8.44e-3, 8.0),
      M1("motor1"),
      timedomain("Main time domain", dt, true)
{
    // Name all blocks
    E1.setName("E1");
    E2.setName("E2");
    E1d.setName("E1d");
    g.setName("g");
    VelCtrl.setName("VelocityController");
    invMotMod.setName("invMotMod");
    M1.setName("M1");

    // Name all signals
    E1.getOut().getSignal().setName("q1 [m]");
    E2.getOut().getSignal().setName("q2 [m]");
    E1d.getOut().getSignal().setName("q1d [m/s]");
    g.getOut().getSignal().setName("q1d_d [rad/s]");
    invMotMod.getOut().getSignal().setName("U [V]");

    // Connect signals
    E1d.getIn().connect(E1.getOut());
    g.getIn().connect(E2.getOut());
    VelCtrl.getIn(0).connect(g.getOut());
    VelCtrl.getIn(1).connect(E1d.getOut());
    invMotMod.getIn(0).connect(VelCtrl.getOut(0));
    invMotMod.getIn(1).connect(VelCtrl.getOut(1));
    M1.getIn().connect(invMotMod.getOut());

    // Add blocks to timedomain
    timedomain.addBlock(E1);
    timedomain.addBlock(E2);
    timedomain.addBlock(E1d);
    timedomain.addBlock(g);
    timedomain.addBlock(VelCtrl);
    timedomain.addBlock(invMotMod);
    timedomain.addBlock(M1);

    // Add timedomain to executor
    eeros::Executor::instance().add(timedomain);
}