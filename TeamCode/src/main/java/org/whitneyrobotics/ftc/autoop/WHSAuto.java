package org.whitneyrobotics.ftc.autoop;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.whitneyrobotics.ftc.lib.subsys.goldpositiondetector.GoldPositionDetector;
import org.whitneyrobotics.ftc.lib.util.Coordinate;
import org.whitneyrobotics.ftc.lib.util.Position;
import org.whitneyrobotics.ftc.lib.util.SimpleTimer;
import org.whitneyrobotics.ftc.subsys.Lift;
import org.whitneyrobotics.ftc.subsys.MarkerDrop;
import org.whitneyrobotics.ftc.subsys.WHSRobotImpl;

@Autonomous(name="WHSAuto", group="auto")
public class WHSAuto extends OpMode{

    WHSRobotImpl robot;

    Coordinate[] startingCoordinateArray = new Coordinate[2];
    Position[] landerClearancePositionArray = new Position[2];
    Position[][] goldPositionArray = new Position[2][3];
    Position wallPosition;
    Position depotCornerPosition;
    Position depotSidePosition;
    Position[] craterPositonArray = new Position[2];
    Position[] depotPositionArray = new Position[2];

    static final int CRATER = 0;
    static final int DEPOT = 1;

    static final int LEFT = 0;
    static final int CENTER = 1;
    static final int RIGHT = 2;

    static final int STARTING_POSITION = DEPOT;

    // state definitions
    static final int INIT = 0;
    static final int DROP_FROM_LANDER = 1;
    static final int DRIVE_FROM_LANDER = 2;
    static final int SAMPLE_PIECE = 3;
    static final int CLAIM_DEPOT = 4;
    static final int DRIVE_TO_CRATER = 5;
    static final int END = 6;

    static final int NUM_OF_STATES = 7;

    boolean[] stateEnabled = new boolean[NUM_OF_STATES];
    private double finishTime;

    public void defineStateEnabledStatus() {
        stateEnabled[INIT] = true;
        stateEnabled[DROP_FROM_LANDER] = true;
        stateEnabled[DRIVE_FROM_LANDER] = true;
        stateEnabled[SAMPLE_PIECE] = true;
        stateEnabled[CLAIM_DEPOT] = true;
        stateEnabled[DRIVE_TO_CRATER] = true;
        stateEnabled[END] = true;
    }

    int currentState;
    int subState;
    String currentStateDesc;
    String subStateDesc;

    SimpleTimer omniArmMoveTimer = new SimpleTimer();
    SimpleTimer storedToDumpedTimer = new SimpleTimer();
    SimpleTimer dumpedToStoredTimer = new SimpleTimer();

    static final double OMNI_ARM_MOVE_DELAY = 1.5;
    static final double MARKER_DROP_DELAY = 0.75;
    static final double DRIVE_FORWARD_SMALL_BIT_DURATION = 0.2;

    private GoldAlignDetector detector;
    GoldPositionDetector.GoldPosition goldPosition;
    double xpos;

    @Override
    public void init() {
        robot = new WHSRobotImpl(hardwareMap);
        robot.lift.liftMotor.setPower(0);
        currentState = INIT;
        subState = 0;

        // from the perspective of blue alliance
        startingCoordinateArray[CRATER] = new Coordinate(350, 350, 150, 45);
        startingCoordinateArray[DEPOT] = new Coordinate(-350, 350, 150, 135);

        landerClearancePositionArray[CRATER] = new Position(650, 650, 150);
        landerClearancePositionArray[DEPOT] = new Position(-650, 650, 150);

        // setting the three different particle positions for the crater side
        goldPositionArray[CRATER][LEFT] = new Position(590, 1190, 150);//(600,1200, 150);
        goldPositionArray[CRATER][CENTER] = new Position(890, 890, 150);//(900,900,150);
        goldPositionArray[CRATER][RIGHT] = new Position(1190, 590, 150);//(1200,600,150);

        // setting the three different particle positions for the depot side
        goldPositionArray[DEPOT][LEFT] = new Position(-1200,600,150);
        goldPositionArray[DEPOT][CENTER] = new Position(-900,900,150);
        goldPositionArray[DEPOT][RIGHT]=  new Position(-600,1200, 150);

        wallPosition = new Position(-150,1425,150);

        depotCornerPosition = new Position(-1280,1200,150);
        depotSidePosition = new Position(-1475, 1200, 150);

        depotPositionArray[DEPOT] = new Position(-1440,1410,150);
        depotPositionArray[CRATER] = new Position(-1300,1425,150);

        craterPositonArray[CRATER] = new Position(800,1440,150);
        craterPositonArray[DEPOT] = new Position(-1475,-800,150);

        defineStateEnabledStatus();

        // vision initialization
        detector = new GoldAlignDetector();
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        detector.useDefaults();
        detector.enable();
    }

    @Override
    public void init_loop() {
        // If you are using Motorola E4 phones,
        // you should send telemetry data while waiting for start.
        telemetry.addData("status", "loop test... waiting for start");
        if(stateEnabled[DROP_FROM_LANDER]) {
            robot.lift.setLiftPosition(Lift.LiftPosition.STORED);
        }
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
                currentStateDesc = "dropping from lander";
                switch (subState) {
                    case 0:
                        subStateDesc = "entry";
                        subState++;
                        break;
                    case 1:
                        subStateDesc = "bringing robot down";
                        robot.lift.bringDownRobot(true);
                        if (robot.lift.getLiftState() == Lift.LiftState.WAITING_FOR_DRIVETRAIN) {
                            subState++;
                        }
                        break;
                    case 2:
                        subStateDesc = "exit";
                        advanceState();
                        break;
                }
                break;
            case DRIVE_FROM_LANDER:
                currentStateDesc = "driving from lander";
                switch (subState) {
                    case 0:
                        subStateDesc = "scanning minerals";
                        xpos = detector.getXPosition();
                        if (xpos < 230) {
                            goldPosition = GoldPositionDetector.GoldPosition.LEFT;
                        } else if (xpos >= 230) {
                            goldPosition = GoldPositionDetector.GoldPosition.CENTER;
                        } else {
                            goldPosition = GoldPositionDetector.GoldPosition.RIGHT;
                        }
                        subState++;
                        break;
                    case 1:
                        subStateDesc = "driving to lander clearance";
                        robot.driveToTarget(landerClearancePositionArray[STARTING_POSITION], false);
                        omniArmMoveTimer.set(OMNI_ARM_MOVE_DELAY);
                        if (!robot.rotateToTargetInProgress() && !robot.driveToTargetInProgress()) {
                            subState++;
                        }
                        break;
                    case 2:
                        subStateDesc = "moving omniarm out of the way";
                        robot.omniArm.storeOmniArm(true);
                        if (omniArmMoveTimer.isExpired()) {
                            subState++;
                        }
                        break;
                    case 3:
                        subStateDesc = "bringing hook down";
                        robot.lift.bringDownHook(true);
                        if (robot.lift.getLiftState() == Lift.LiftState.STANDING_BY_FOR_END_GAME) {
                            subState++;
                        }
                        omniArmMoveTimer.set(OMNI_ARM_MOVE_DELAY);
                        break;
                    case 4:
                        subStateDesc = "storing omniarm";
                        robot.omniArm.resetOmniArm(true);
                        if (omniArmMoveTimer.isExpired()) {
                            subState++;
                        }
                        break;
                    case 5:
                        subStateDesc = "exit";
                        advanceState();
                        break;
                }
                break;
            case SAMPLE_PIECE:
                currentStateDesc = "sampling piece";
                switch (subState) {
                    case 0:
                        subStateDesc = "entry";
                        robot.driveToTarget(goldPositionArray[STARTING_POSITION][goldPosition.ordinal()], true);
                        if (!robot.rotateToTargetInProgress() && !robot.driveToTargetInProgress()) {
                            subState++;
                        }
                        break;
                    case 1:
                        subStateDesc = "driving back to lander clearance";
                        if (STARTING_POSITION == CRATER) {
                            robot.driveToTarget(landerClearancePositionArray[CRATER], true);
                        }
                        if (!robot.rotateToTargetInProgress() && !robot.driveToTargetInProgress()) {
                            subState++;
                        }
                        break;
                    case 2:
                        subStateDesc = "exit";
                        advanceState();
                        break;
                }
                break;
            case CLAIM_DEPOT:
                currentStateDesc = "claiming depot";
                switch (subState) {
                    case 0:
                        subStateDesc = "entry";
                        if (STARTING_POSITION == CRATER) {
                            robot.driveToTarget(wallPosition, false);
                        } else if (STARTING_POSITION == DEPOT) {
                            robot.driveToTarget(depotCornerPosition, true);
                        }
                        if (!robot.rotateToTargetInProgress() && !robot.driveToTargetInProgress()) {
                            subState++;
                        }
                        break;
                    case 1:
                        subStateDesc = "driving to depot";
                        if (STARTING_POSITION == CRATER) {
                            robot.driveToTarget(depotPositionArray[CRATER], true);
                        } else if (STARTING_POSITION == DEPOT) {
                            robot.driveToTarget(depotSidePosition, false);
                        }
                        if (!robot.rotateToTargetInProgress() && !robot.driveToTargetInProgress()) {
                            subState++;
                        }
                        storedToDumpedTimer.set(MARKER_DROP_DELAY);
                        break;
                    case 2:
                        subStateDesc = "dumping marker";
                        robot.markerDrop.operateMarkerDrop(MarkerDrop.MarkerDropPosition.DUMPED);
                        if (storedToDumpedTimer.isExpired()) {
                            subState++;
                        }
                        dumpedToStoredTimer.set(MARKER_DROP_DELAY);
                        break;
                    case 3:
                        subStateDesc = "storing marker drop";
                        robot.markerDrop.operateMarkerDrop(MarkerDrop.MarkerDropPosition.STORED);
                        if (dumpedToStoredTimer.isExpired()) {
                            subState++;
                        }
                        break;
                    case 4:
                        subStateDesc = "exit";
                        advanceState();
                        break;
                }
                break;
            case DRIVE_TO_CRATER:
                currentStateDesc = "drive to crater";
                switch (subState) {

                    case 0:
                        subStateDesc = "driving to crater";
                        robot.driveToTarget(craterPositonArray[STARTING_POSITION], true);
                        if (!robot.rotateToTargetInProgress() && !robot.driveToTargetInProgress()) {
                            subState++;
                        }
                        break;
                    case 1:
                        subStateDesc = "exit";
                        advanceState();
                        break;
                }
                break;
            case END:
                currentStateDesc = "end";
                break;
            default: break;
        }

        telemetry.addData("Substate: ", currentStateDesc + ", " + subStateDesc);
        telemetry.addData("Current X", robot.getCoordinate().getX());
        telemetry.addData("Current Y", robot.getCoordinate().getY());

    }

    public void advanceState() {
        if (stateEnabled[(currentState + 1)]) {
            currentState = currentState + 1;
            subState = 0;
        } else {
            currentState = currentState + 1;
            advanceState();
        }
    }
}