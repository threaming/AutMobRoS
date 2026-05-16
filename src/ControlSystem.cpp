#include "ControlSystem.hpp"

ControlSystem::ControlSystem(double dt)
    : E1("enc1"),
      E2("enc2"),
      Kp(1.0 / dt / 4.6 / 0.7 * 1.0 / dt / 4.6 / 0.7),
      Kd(2.0 * 0.7 / dt / 4.6 / 0.7),
      M(3441.0 / 104.0 * 3441.0 / 104.0 * 6.8e-8),
      qd_dummy(0.0),
      invMotMod(0.1, 21.2, 3441.0/104.0, 8.44e-3, 8),
      motor2("motor2"),
      timedomain("Main time domain", dt, true)
{
    // Name all blocks
    E1.setName("E1");
    E2.setName("E2");
    e.setName("e");
    Kp.setName("Kp");
    ed.setName("ed");
    Kd.setName("Kd");
    qdd_c.setName("qdd_c");
    M.setName("M");
    qd_dummy.setName("qd_dummy");
    invMotMod.setName("invMotMod");
    motor2.setName("motor2");

    // Name all signals
    E1.getOut().getSignal().setName("q1 [rad]");
    E2.getOut().getSignal().setName("q2 [rad]");
    e.getOut().getSignal().setName("e [rad]");
    Kp.getOut().getSignal().setName("qdd_cp [rad/s^2]");
    ed.getOut().getSignal().setName("ed [rad/s]");
    Kd.getOut().getSignal().setName("qdd_cd [rad/s^2]");
    qdd_c.getOut().getSignal().setName("qdd_c [rad/s^2]");
    M.getOut().getSignal().setName("Q1 [Nm]");
    qd_dummy.getOut().getSignal().setName("qd [rad/s]");
    invMotMod.getOut().getSignal().setName("U1 [V]");

    // Connect signals
    e.getIn(0).connect(E2.getOut());
    e.getIn(1).connect(E1.getOut());
    e.negateInput(1);
    Kp.getIn().connect(e.getOut());
    ed.getIn().connect(e.getOut());
    Kd.getIn().connect(ed.getOut());
    qdd_c.getIn(0).connect(Kp.getOut());
    qdd_c.getIn(1).connect(Kd.getOut());
    M.getIn().connect(qdd_c.getOut());
    invMotMod.getIn(0).connect(M.getOut());
    invMotMod.getIn(1).connect(qd_dummy.getOut());
    motor2.getIn().connect(invMotMod.getOut());

    // Add blocks to timedomain
    timedomain.addBlock(E1);
    timedomain.addBlock(E2);
    timedomain.addBlock(e);
    timedomain.addBlock(Kp);
    timedomain.addBlock(ed);
    timedomain.addBlock(Kd);
    timedomain.addBlock(qdd_c);
    timedomain.addBlock(M);
    timedomain.addBlock(qd_dummy);
    timedomain.addBlock(invMotMod);
    timedomain.addBlock(motor2);

    // Add timedomain to executor
    eeros::Executor::instance().add(timedomain);
}