package org.whitneyrobotics.ftc.autoop;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.whitneyrobotics.ftc.subsys.WHSRobotImpl;

@Autonomous(name="WHSAuto", group="auto")
public class WHSAuto extends OpMode{

    WHSRobotImpl robot;

    //State Definitions
    static final int INIT = 0;
    static final int DROP_FROM_LANDER = 1;
    static final int SAMPLE_PIECE = 2;
    static final int CLAIM_DEPOT = 3;
    static final int DRIVE_TO_CRATER = 4;
    static final int EXIT = 5;

    static final int NUM_OF_STATES = 6;

    boolean[] stateEnabled = new boolean[NUM_OF_STATES];

    public void defineStateEnabledStatus() {
        stateEnabled[INIT] = true;
        stateEnabled[DROP_FROM_LANDER] = true;
        stateEnabled[SAMPLE_PIECE] = true;
        stateEnabled[CLAIM_DEPOT] = true;
        stateEnabled[DRIVE_TO_CRATER] = true;
        stateEnabled[EXIT] = true;
    }

    int currentState;
    String currentStateDesc;

    @Override
    public void init() {
        robot = new WHSRobotImpl(hardwareMap);
        currentState = INIT;
    }

    @Override
    public void loop() {
        robot.estimateHeading();
        robot.estimatePosition();

        switch (currentState) {
            case INIT:
                
                advanceState();
                break;
            case DROP_FROM_LANDER:
                advanceState();
                break;
            case SAMPLE_PIECE:
                advanceState();
                break;
            case CLAIM_DEPOT:
                advanceState();
                break;
            case DRIVE_TO_CRATER:
                advanceState();
                break;
            case EXIT:
                advanceState();
                break;
        }
    }

    public void advanceState() {
        if (stateEnabled[(currentState + 1)]) {
            currentState = currentState + 1;
        } else {
            currentState = currentState + 1;
            advanceState();
        }
    }
}
