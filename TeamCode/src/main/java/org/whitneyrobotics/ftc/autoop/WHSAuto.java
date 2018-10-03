package org.whitneyrobotics.ftc.autoop;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.whitneyrobotics.ftc.lib.util.Coordinate;
import org.whitneyrobotics.ftc.lib.util.Position;
import org.whitneyrobotics.ftc.subsys.WHSRobotImpl;

@Autonomous(name="WHSAuto", group="auto")
public class WHSAuto extends OpMode{

    WHSRobotImpl robot;

    Coordinate[] startingCoordinateArray = new Coordinate[2];
    Position[] landerClearancePositionArray = new Position[2];


    static final int CRATER = 0;
    static final int DEPOT = 1;

    static final int STARTING_POSITION = CRATER;

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
    String subStateDesc;

    boolean performStateEntry;
    boolean performStateExit;

    @Override
    public void init() {
        robot = new WHSRobotImpl(hardwareMap);
        currentState = INIT;

        performStateEntry = true;
        performStateExit = false;

        startingCoordinateArray[CRATER] = new Coordinate(300, 300, 150, 45);
        startingCoordinateArray[DEPOT] = new Coordinate(-300, 300, 150, 135);

        landerClearancePositionArray[CRATER] = new Position(500, 500, 150);
        landerClearancePositionArray[DEPOT] = new Position(-500, 500, 150);
    }

    @Override
    public void loop() {
        robot.estimateHeading();
        robot.estimatePosition();

        switch (currentState) {
            case INIT:
                currentStateDesc = "starting auto";
                robot.setInitialCoordinate(startingCoordinateArray[STARTING_POSITION]);
                advanceState();
                break;
            case DROP_FROM_LANDER:
                if (performStateEntry) {
                    //drop from lander, unlatch
                    performStateEntry = false;
                    subStateDesc = "entry";
                }
                robot.driveToTarget(landerClearancePositionArray[STARTING_POSITION], false);

                if (robot.driveToTargetInProgress() || robot.rotateToTargetInProgress()) {
                    robot.driveToTarget(landerClearancePositionArray[STARTING_POSITION], false);
                } else {
                    performStateExit = true;
                }
                if (performStateExit) {
                    performStateEntry = true;
                    performStateExit = false;
                    advanceState();
                }
                break;
            case SAMPLE_PIECE:
                if (performStateEntry) {
                    //vision stuff
                    performStateEntry = false;
                    subStateDesc = "entry";
                }
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
