package org.whitneyrobotics.ftc.autoop;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.whitneyrobotics.ftc.lib.subsys.goldpositiondetector.GoldPositionDetector;
import org.whitneyrobotics.ftc.lib.util.Coordinate;
import org.whitneyrobotics.ftc.lib.util.Position;
import org.whitneyrobotics.ftc.subsys.WHSRobotImpl;

@Autonomous(name="WHSAuto", group="auto")
public class WHSAuto extends OpMode{

    WHSRobotImpl robot;

    Coordinate[] startingCoordinateArray = new Coordinate[2];
    Position[] landerClearancePositionArray = new Position[2];
    Position[][] goldPositionArray = new Position[2][3];
    Position wallPosition = new Position(-300,1600,150);
    Position depotPosition = new Position(1600,-1600,150);
    Position[] craterPositonArray = new Position[2];


    static final int CRATER = 0;
    static final int DEPOT = 1;

    static final int LEFT = 0;
    static final int CENTER = 1;
    static final int RIGHT = 2;

    static final int STARTING_POSITION = CRATER;

    //State Definitions
    static final int INIT = 0;
    static final int DROP_FROM_LANDER = 1;
    static final int SAMPLE_PIECE = 2;
    static final int CLAIM_DEPOT = 3;
    static final int DRIVE_TO_CRATER = 4;
    static final int END = 5;

    static final int NUM_OF_STATES = 6;

    boolean[] stateEnabled = new boolean[NUM_OF_STATES];

    public void defineStateEnabledStatus() {
        stateEnabled[INIT] = true;
        stateEnabled[DROP_FROM_LANDER] = true;
        stateEnabled[SAMPLE_PIECE] = true;
        stateEnabled[CLAIM_DEPOT] = true;
        stateEnabled[DRIVE_TO_CRATER] = true;
        stateEnabled[END] = true;
    }

    int currentState;
    String currentStateDesc;
    String subStateDesc;

    GoldPositionDetector.GoldPosition goldPosition;

    boolean drivingToGold;
    boolean drivingToWall;

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

        //setting the three different particle positions for the crater side
        goldPositionArray[CRATER][LEFT] = new Position(600,120, 150);
        goldPositionArray[CRATER][CENTER] = new Position(900,900,150);
        goldPositionArray[CRATER][RIGHT] = new Position(1200,600,150);

        //setting the three different particle positions for the depot side
        goldPositionArray[DEPOT][LEFT] = new Position(-1200,600,150);
        goldPositionArray[DEPOT][CENTER] = new Position(-900,900,150);
        goldPositionArray[DEPOT][RIGHT]=  new Position(-600,1200, 150);

        craterPositonArray[CRATER] = new Position(800,1600,150);
        craterPositonArray[DEPOT] = new Position(-1600,-800,150);
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
                    robot.driveToTarget(landerClearancePositionArray[STARTING_POSITION], false);
                    performStateEntry = false;
                    subStateDesc = "entry";
                }

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
                    robot.driveToTarget(goldPositionArray[STARTING_POSITION][goldPosition.ordinal()], true);
                    drivingToGold = true;
                    performStateEntry = false;
                    subStateDesc = "entry";
                }
                if (drivingToGold) {
                    if (robot.driveToTargetInProgress() || robot.rotateToTargetInProgress()) {
                        robot.driveToTarget(goldPositionArray[STARTING_POSITION][goldPosition.ordinal()], true);
                    } else {
                        drivingToGold = false;
                    }
                } else {
                    if (STARTING_POSITION == CRATER) {
                        robot.driveToTarget(landerClearancePositionArray[CRATER], true);
                    }
                    if (robot.driveToTargetInProgress() || robot.rotateToTargetInProgress()) {
                        robot.driveToTarget(landerClearancePositionArray[CRATER], true);
                    }
                }
                advanceState();
                break;
            case CLAIM_DEPOT:
                if (performStateEntry) {
                    if (STARTING_POSITION == CRATER) {
                        robot.driveToTarget(wallPosition, true);
                        drivingToWall = true;
                    } else if (STARTING_POSITION == DEPOT) {
                        robot.driveToTarget(depotPosition, true);
                    }
                    performStateEntry = false;
                    subStateDesc = "entry";
                }
                if (STARTING_POSITION == CRATER) {
                    if (drivingToWall && (robot.driveToTargetInProgress() || robot.rotateToTargetInProgress())) {
                        robot.driveToTarget(wallPosition, true);
                    } else {
                        drivingToWall = false;
                        robot.driveToTarget(depotPosition, true);
                    }

                }
                advanceState();
                break;
            case DRIVE_TO_CRATER:
                advanceState();
                break;
            case END:
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
