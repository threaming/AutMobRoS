#include "MyRobotSafetyProperties.hpp"

MyRobotSafetyProperties::MyRobotSafetyProperties(ControlSystem &cs, double dt)
    : cs(cs),
    
      // Define all safety levels
      slSystemOff("System is offline"),
      slShuttingDown("System is shutting down"),
      slBraking("System is braking"),
      slStartingUp("System is starting up"),
      slEmergency("System in emergency state"),
      slEmergencyBreaking("System performing emergency braking"),
      slSystemOn("System is online"),
      slMotorPowerOn("Motor power is enabled"),
      slSystemMoving("System is moving"),

      // Define all events
      abort("Abort operation"),
      shutdown("Shutdown the system"),
      doSystemOn("Startup the system"),
      systemStarted("System has started"),
      emergency("Emergency triggered"),
      resetEmergency("Emergency reset"),
      powerOn("Power motors on"),
      powerOff("Power motors off"),
      startMoving("Start system movement"),
      stopMoving("Stop system movement"),
      motorsHalted("Motors have halted")
{
    eeros::hal::HAL &hal = eeros::hal::HAL::instance();

    // Declare and add critical outputs
    greenLED = hal.getLogicOutput("onBoardLEDgreen");
    redLED = hal.getLogicOutput("onBoardLEDred");

    criticalOutputs = { greenLED, redLED };

    // Declare and add critical inputs
    buttonPause = eeros::hal::HAL::instance().getLogicInput("onBoardButtonPause");
    buttonMode = eeros::hal::HAL::instance().getLogicInput("onBoardButtonMode");

    criticalInputs = { buttonPause, buttonMode };

    // Add all safety levels to the safety system
    addLevel(slSystemOff);
    addLevel(slShuttingDown);
    addLevel(slBraking);
    addLevel(slStartingUp);
    addLevel(slEmergency);
    addLevel(slEmergencyBreaking);
    addLevel(slSystemOn);
    addLevel(slMotorPowerOn);
    addLevel(slSystemMoving);

    // Add events to individual safety levels
    slSystemOff.addEvent(doSystemOn, slStartingUp, kPublicEvent);
    slShuttingDown.addEvent(shutdown, slSystemOff, kPrivateEvent);
    slBraking.addEvent(motorsHalted, slShuttingDown, kPrivateEvent);
    slStartingUp.addEvent(systemStarted, slSystemOn, kPrivateEvent);
    slEmergency.addEvent(resetEmergency, slSystemOn, kPrivateEvent);
    slEmergencyBreaking.addEvent(motorsHalted, slEmergency, kPrivateEvent);
    slSystemOn.addEvent(powerOn, slMotorPowerOn, kPublicEvent);
    slMotorPowerOn.addEvent(startMoving, slSystemMoving, kPublicEvent);
    slMotorPowerOn.addEvent(powerOff, slSystemOn, kPublicEvent);
    slSystemMoving.addEvent(stopMoving, slMotorPowerOn, kPublicEvent);
    slSystemMoving.addEvent(emergency, slEmergencyBreaking, kPublicEvent);
    slSystemMoving.addEvent(abort, slBraking, kPublicEvent);

    // Add events to multiple safety levels
    addEventToAllLevelsBetween(slEmergency, slMotorPowerOn, abort, slShuttingDown, kPublicEvent);
    addEventToAllLevelsBetween(slSystemOn, slMotorPowerOn, emergency, slEmergency, kPublicEvent);

    // Define input actions for all levels
    slSystemOff.setInputActions({           ignore(buttonPause),                    ignore(buttonMode) });
    slShuttingDown.setInputActions({        ignore(buttonPause),                    ignore(buttonMode) });
    slBraking.setInputActions({             ignore(buttonPause),                    ignore(buttonMode) });
    slStartingUp.setInputActions({          ignore(buttonPause),                    ignore(buttonMode) });
    slEmergency.setInputActions({           ignore(buttonPause),                    check(buttonMode, false, resetEmergency) });
    slEmergencyBreaking.setInputActions({    ignore(buttonPause),                    ignore(buttonMode) });
    slSystemOn.setInputActions({            check(buttonPause, false, emergency),   ignore(buttonMode) });
    slMotorPowerOn.setInputActions({        check(buttonPause, false, emergency),   ignore(buttonMode) });
    slSystemMoving.setInputActions({        check(buttonPause, false, emergency),   ignore(buttonMode) });

    // Define output actions for all levels
    slSystemOff.setOutputActions({           set(greenLED, false),   set(redLED, false) });
    slShuttingDown.setOutputActions({        set(greenLED, false),   set(redLED, true) });
    slBraking.setOutputActions({             set(greenLED, false),   set(redLED, true) });
    slStartingUp.setOutputActions({          set(greenLED, true),    set(redLED, false) });
    slEmergency.setOutputActions({           set(greenLED, true),    set(redLED, true) });
    slEmergencyBreaking.setOutputActions({    set(greenLED, true),    set(redLED, true) });
    slSystemOn.setOutputActions({            set(greenLED, true),    set(redLED, false) });
    slMotorPowerOn.setOutputActions({        set(greenLED, true),    set(redLED, false) });
    slSystemMoving.setOutputActions({        set(greenLED, true),    set(redLED, false) });

    // Define and add level actions
    slSystemOff.setLevelAction([&](SafetyContext *privateContext) {
        eeros::Executor::stop();
    });

    slShuttingDown.setLevelAction([&](SafetyContext *privateContext) {
        cs.timedomain.stop();
        privateContext->triggerEvent(shutdown);
    });

    slBraking.setLevelAction([&](SafetyContext *privateContext) {
        // Check if motors are standing sill
        privateContext->triggerEvent(motorsHalted);
    });

    slStartingUp.setLevelAction([&](SafetyContext *privateContext) {
        cs.timedomain.start();
        privateContext->triggerEvent(systemStarted);
    });

    slEmergency.setLevelAction([&](SafetyContext *privateContext) {
        
    });

    slEmergencyBreaking.setLevelAction([&](SafetyContext *privateContext) {
        // Check if motors are standing still
        privateContext->triggerEvent(motorsHalted);
    });

    slSystemOn.setLevelAction([&, dt](SafetyContext *privateContext) {
        cs.signalChecker.reset();
        if (slSystemOn.getNofActivations()*dt >= 1)   // wait 1 sec
        {
            privateContext->triggerEvent(powerOn);
        }
    });

    slMotorPowerOn.setLevelAction([&, dt](SafetyContext *privateContext) {
        if (slMotorPowerOn.getNofActivations()*dt >= 5)   // wait 5 sec
        {
            privateContext->triggerEvent(startMoving);
        }
    });

    slSystemMoving.setLevelAction([&, dt](SafetyContext *privateContext) {
        if (slSystemMoving.getNofActivations()*dt >= 5)   // wait 5 sec
        {
            privateContext->triggerEvent(stopMoving);
        }
    });

    // Define entry level
    setEntryLevel(slSystemOff);

    // Define exit function
    exitFunction = ([&](SafetyContext *privateContext) {
        privateContext->triggerEvent(abort);
    });
}