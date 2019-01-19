package org.whitneyrobotics.ftc.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.whitneyrobotics.ftc.lib.subsys.robot.WHSRobot;
import org.whitneyrobotics.ftc.lib.util.Position;
import org.whitneyrobotics.ftc.subsys.WHSRobotImpl;

@Autonomous(name = "ActuatorTest", group = "tests")
public class ActuatorTest extends OpMode {

    WHSRobot robot;

    /**
     * State definitions
     */
    static final int INIT = 0;
    static final int LIFT = 1;
    static final int MARKER_DROP = 2;
    static final int INTAKE = 3;
    static final int OMNIARM = 4;
    static final int TILERUNNER = 5;

    static final int NUM_STATES = 6;

    boolean[] stateEnabled = new boolean[NUM_STATES];

    int state = INIT;
    int subState = 0;
    String stateDesc;
    String substateDesc;

    //TODO: Connect to FTC Dashboard.
    //TODO: (long-term) Make GUI to operate from driver station.
    /**
     * Determines which states will be run.
     */
    public void defineStateEnabledStatus() {
        stateEnabled[INIT] = true;
        stateEnabled[LIFT] = true;
        stateEnabled[MARKER_DROP] = true;
        stateEnabled[INTAKE] = true;
        stateEnabled[OMNIARM] = true;
        stateEnabled[TILERUNNER] = true;
    }

    /**
     * Advances the state, skipping ones that have been disabled.
     */
    public void advanceState() {
        if (stateEnabled[(state + 1)]) {
            state++;
            subState = 0;
        } else {
            state++;
            advanceState();
        }
    }

    @Override
    public void init() {
        robot = new WHSRobotImpl(hardwareMap);
        defineStateEnabledStatus();
    }

    @Override
    public void init_loop() {

    }

    @Override
    public void loop() {
        switch (state) {
            case INIT:
                stateDesc = "Initializing";
                advanceState();
                break;
            case LIFT:
                stateDesc = "Testing Lift";
                break;
            case MARKER_DROP:
                break;
            case INTAKE:
                break;
            case OMNIARM:
                break;
            case TILERUNNER:
                break;
            default: break;
        }

        telemetry.addData("State:", stateDesc);
        telemetry.addData("Substate:", substateDesc);
    }
}
