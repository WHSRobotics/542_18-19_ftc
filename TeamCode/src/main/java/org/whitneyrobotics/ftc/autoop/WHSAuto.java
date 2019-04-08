package org.whitneyrobotics.ftc.autoop;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import org.whitneyrobotics.ftc.lib.util.Coordinate;
import org.whitneyrobotics.ftc.lib.util.Position;
import org.whitneyrobotics.ftc.lib.util.SimpleTimer;
import org.whitneyrobotics.ftc.subsys.Lift;
import org.whitneyrobotics.ftc.subsys.MarkerDrop;
import org.whitneyrobotics.ftc.subsys.OmniArm;
import org.whitneyrobotics.ftc.subsys.WHSRobotImpl;

import java.util.List;

@Autonomous(name = "WHSAuto", group = "auto")
public class WHSAuto extends OpMode{

    WHSRobotImpl robot;

    /**
     * Positioning
     */
    Coordinate[] startingCoordinateArray = new Coordinate[2];
    Position[] landerClearancePositionArray = new Position[2];
    Position[][] goldPositionArray = new Position[2][3];
    Position wallPosition;
    Position[] depotCornerPositionArray = new Position[3];
    Position depotSidePosition;
    Position[] craterPositonArray = new Position[2];
    Position[] depotPositionArray = new Position[2];

    static final int CRATER = 0;
    static final int DEPOT = 1;
    static final int LEFT = 0;
    static final int CENTER = 1;
    static final int RIGHT = 2;
    static final int STARTING_POSITION = CRATER;

    /**
     * State Definitions
     */
    static final int INIT = 0;
    static final int DROP_FROM_LANDER = 1;
        /**
         * Substates:
         * - Entry
         * - Scanning Minerals
         * - Moving OmniArm out of Lift's way
         * - Bringing robot down
         * - Exit
         */
    static final int DRIVE_FROM_LANDER = 2;
        /**
         * Substates:
         * - Entry
         * - Driving to lander clearance
         * - Bringing hook down
         * - Resetting OmniArm
         * - Exit
         */
    static final int SAMPLE_MINERAL = 3;
        /**
         * Substates:
         * - Entry
         * - Driving to mineral
         * - Driving back to lander clearance [CRATER]
         * - Exit
         */
    static final int CLAIM_DEPOT = 4;
        /**
         * Substates:
         * - Entry
         * - Driving to intermediate position
         * - Driving to depot
         * - Rotating robot [CRATER]
         * - Dumping MarkerDrop
         * - Storing MarkerDrop
         * - Rotating robot [DEPOT]
         * - Exit
         */
    static final int DRIVE_TO_CRATER = 5;
        /**
         * Substates:
         * - Entry
         * - Driving to wall position [CRATER]
         * - Driving to crater
         * - Exit
         */
    static final int END = 6;

    static final int NUM_OF_STATES = 7;

    boolean[] stateEnabled = new boolean[NUM_OF_STATES];

    int state = INIT;
    int subState = 0;
    int goldPosition = CENTER;
    String stateDesc;
    String subStateDesc;

    boolean zThresholdExceeded = false;

    /**
     * Timers
     */
    SimpleTimer scanMineralsTimer = new SimpleTimer();
    SimpleTimer dumpMarkerDropTimer = new SimpleTimer();
    SimpleTimer storeMarkerDropTimer = new SimpleTimer();

    static final double SCAN_MINERALS_DURATION = 2.0;
    static final double MOVE_MARKER_DROP_DURATION = 0.75;

    /**
     * Tensorflow Variables
     */
    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    private static final String VUFORIA_KEY = "AZpuDDL/////AAABmTx4NapZXku6l0aaFFDwgWsYaPViIxPYFdJ8R9R4gPesBY5Ublbla/sRrihytU6cN9eb5Z30d8FuDcbgxBpdrg7gwPgn8GDXm5EEVpuOZnXYEOlcMTAz1nQcnDTPHcyFz1OKJz17ZoHeYtW70mbD7gqAmu/pP4Zz1cCR44pCFw954WOg7SsmVvuiL6J5iEQFIq68QEdX2sjOK7TmaE3RPATv8pZiU/1pwS5iBxSd+8X7PpBII0Ncc88CsKzrNBhF710j9j6fyHl2BeZhcAcRZ/9Fp1W+Cz2kvMPgI5Ah+FmAvWCOJLverLIYm/lgWhEeQrzpCNhc1eAxjtIAUnq5vhBz++vugvyv8o9fuf8HAxWB";

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    private boolean goldMineralDetected = false;

    /**
     * Determines which states will be run.
     */
    public void defineStateEnabledStatus() {
        stateEnabled[INIT] = true;
        stateEnabled[DROP_FROM_LANDER] = true;
        stateEnabled[DRIVE_FROM_LANDER] = true;
        stateEnabled[SAMPLE_MINERAL] = true;
        stateEnabled[CLAIM_DEPOT] = true;
        stateEnabled[DRIVE_TO_CRATER] = true;
        stateEnabled[END] = true;
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

    /**
     * Initializes the Vuforia localization engine.
     */
    private void initVuforia() {
        // Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        // Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    /**
     * Initializes the Tensor Flow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }

    /**
     * Returns position of the gold mineral.
     */
    private int detectGoldPosition() {
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                telemetry.addData("# Object Detected", updatedRecognitions.size());
                if (updatedRecognitions.size() == 3) {
                    int goldMineralX = -1;
                    int silverMineral1X = -1;
                    int silverMineral2X = -1;
                    for (Recognition recognition : updatedRecognitions) {
                        if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                            goldMineralX = (int) recognition.getLeft();
                        } else if (silverMineral1X == -1) {
                            silverMineral1X = (int) recognition.getLeft();
                        } else {
                            silverMineral2X = (int) recognition.getLeft();
                        }
                    }
                    if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                        goldMineralDetected = true;
                        if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                            return LEFT;
                        } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                            return RIGHT;
                        } else {
                            return CENTER;
                        }
                    }
                }
            }
        }
        return 3;
    }

    @Override
    public void init() {
        robot = new WHSRobotImpl(hardwareMap);
        robot.lift.liftMotor.setPower(0);

        // from the perspective of blue alliance
        startingCoordinateArray[CRATER] = new Coordinate(350, 350, 150, 47.5);
        startingCoordinateArray[DEPOT] = new Coordinate(-350, 350, 150, 137.5);

        // Position in which we move the robot to allow for the lift to go down
        landerClearancePositionArray[CRATER] = new Position(590, 590, 150);
        landerClearancePositionArray[DEPOT] = new Position(-590, 590, 150);

        // setting the three different mineral positions for the crater side
        goldPositionArray[CRATER][LEFT] = new Position(500, 1250, 150);
        goldPositionArray[CRATER][CENTER] = new Position(930, 930, 150);
        goldPositionArray[CRATER][RIGHT] = new Position(1250, 500, 150);

        // setting the three different mineral positions for the depot side
        goldPositionArray[DEPOT][LEFT] = new Position(-1220, 590, 150);
        goldPositionArray[DEPOT][CENTER] = new Position(-900, 900, 150);
        goldPositionArray[DEPOT][RIGHT]=  new Position(-600, 1220, 150);

        // rAndOm cRaTer and dEpOt pOsiTiOns
        wallPosition = new Position(0, 1500, 150);
        depotCornerPositionArray[LEFT] = new Position(-1280, 1120, 150);
        depotCornerPositionArray[CENTER] = new Position(-1280, 1320, 150);
        depotCornerPositionArray[RIGHT] = new Position(-1280, 1520, 150);
        depotSidePosition = new Position(-1450, 1320, 150);

        depotPositionArray[DEPOT] = new Position(-1440, 1420, 150);
        depotPositionArray[CRATER] = new Position(-1290, 1490, 150);

        craterPositonArray[CRATER] = new Position(640, 1370, 150);//(750, 1365, 150);
        craterPositonArray[DEPOT] = new Position(-1490, -640, 150);

        defineStateEnabledStatus();
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }
    }

    @Override
    public void init_loop() {
        // If you are using Motorola E4 phones,
        // you should send telemetry data while waiting for start.
        telemetry.addData("Status", "Waiting for start...");
        if (stateEnabled[DROP_FROM_LANDER]) {
            robot.lift.setLiftPosition(Lift.LiftPosition.STORED);
        }
    }

    @Override
    public void start(){
        /** Activate Tensor Flow Object Detection. */
        if (tfod != null) {
            tfod.activate();
        }
    }

    @Override
    public void loop() {
        robot.estimateHeading();
        robot.estimatePosition();

        switch (state) {
            case INIT:
                stateDesc = "Starting Auto";
                robot.setInitialCoordinate(startingCoordinateArray[STARTING_POSITION]);
                advanceState();
                break;
            case DROP_FROM_LANDER:
                stateDesc = "Dropping from lander";
                switch (subState) {
                    case 0:
                        subStateDesc = "Entry";
                        goldPosition = CENTER;
                        scanMineralsTimer.set(SCAN_MINERALS_DURATION);
                        subState++;
                        break;
                    case 1:
                        subStateDesc = "Scanning minerals";
                        int goldDetection = detectGoldPosition();
                        if (goldDetection != 3) {
                            goldPosition = goldDetection;
                        }

                        if (scanMineralsTimer.isExpired()) {
                            if (goldPosition == 3) {
                                goldPosition = CENTER;
                            }
                            subState++;
                        }
                        break;
                    case 2:
                        subStateDesc = "Bringing robot down";
                        robot.lift.bringDownRobot(true);
                        if (robot.lift.getCurrentLiftPosition() == Lift.LiftPosition.ABOVE_LATCH) {
                            subState++;
                        }
                        break;
                    case 3:
                        subStateDesc = "Exit";
                        advanceState();
                        break;
                }
                break;
            case DRIVE_FROM_LANDER:
                stateDesc = "Driving from lander";
                switch (subState) {
                    case 0:
                        subStateDesc = "Entry";
                        subState++;
                    case 1:
                        subStateDesc = "Driving to lander clearance";
                        robot.driveToTarget(landerClearancePositionArray[STARTING_POSITION], false);
                        if (!robot.rotateToTargetInProgress() && !robot.driveToTargetInProgress()) {
                            subState++;
                        }
                        break;
                    case 2:
                        subStateDesc = "Bringing hook down";
                        robot.lift.bringDownHook(true);
                        if (robot.lift.getCurrentLiftPosition() == Lift.LiftPosition.STORED) {
                            subState++;
                        }
                        break;
                    case 3:
                        subStateDesc = "Exit";
                        advanceState();
                        break;
                }
                break;
            case SAMPLE_MINERAL:
                stateDesc = "Sampling mineral";
                switch (subState) {
                    case 0:
                        subStateDesc = "Entry";
                        subState++;
                    case 1:
                        subStateDesc = "Driving to mineral";
                        robot.driveToTarget(goldPositionArray[STARTING_POSITION][goldPosition], true);
                        if (!robot.rotateToTargetInProgress() && !robot.driveToTargetInProgress()) {
                            subState++;
                        }
                        break;
                    case 2:
                        subStateDesc = "Driving back to lander clearance";
                        if (STARTING_POSITION == CRATER) {
                            robot.driveToTarget(landerClearancePositionArray[CRATER], true);
                            if (!robot.rotateToTargetInProgress() && !robot.driveToTargetInProgress()) {
                                subState++;
                            }
                        } else {
                            subState++;
                        }
                        break;
                    case 3:
                        subStateDesc = "Exit";
                        advanceState();
                        break;
                }
                break;
            case CLAIM_DEPOT:
                stateDesc = "Claiming depot";
                switch (subState) {
                    case 0:
                        subStateDesc = "Entry";
                        subState++;
                        break;
                    case 1:
                        subStateDesc = "Driving to intermediate position";
                        if (STARTING_POSITION == CRATER) {
                            robot.driveToTarget(wallPosition, false);
                        } else if (STARTING_POSITION == DEPOT ) {
                            robot.driveToTarget(depotCornerPositionArray[goldPosition], false);
                        }
                        if (!robot.rotateToTargetInProgress() && !robot.driveToTargetInProgress()) {
                            subState++;
                        }
                        break;
                    case 2:
                        subStateDesc = "Driving to depot";
                        if (STARTING_POSITION == CRATER) {
                            robot.driveToTarget(depotPositionArray[CRATER], true);
                        } else if (STARTING_POSITION == DEPOT) {
                            robot.driveToTarget(depotSidePosition, false);
                        }
                        if (!robot.rotateToTargetInProgress() && !robot.driveToTargetInProgress()) {
                            subState++;
                        }
                        break;
                    case 3:
                        subStateDesc = "Rotating robot";
                        if (STARTING_POSITION == CRATER) {
                            robot.rotateToTarget(270, false);
                        } else {
                            robot.rotateToTarget(270, false);
                        }
                        if (!robot.rotateToTargetInProgress()) {
                            dumpMarkerDropTimer.set(MOVE_MARKER_DROP_DURATION);
                            subState++;
                        }
                        break;
                    case 4:
                        subStateDesc = "Dumping MarkerDrop";
                        robot.markerDrop.operateMarkerDrop(MarkerDrop.MarkerDropPosition.DUMPED);
                        if (dumpMarkerDropTimer.isExpired()) {
                            storeMarkerDropTimer.set(MOVE_MARKER_DROP_DURATION);
                            subState++;
                        }
                        break;
                    case 5:
                        subStateDesc = "Storing MarkerDrop";
                        robot.markerDrop.operateMarkerDrop(MarkerDrop.MarkerDropPosition.STORED);
                        if (storeMarkerDropTimer.isExpired()) {
                            subState++;
                        }
                        break;
                    case 6:
                        subStateDesc = "Rotating robot";
                        if (STARTING_POSITION == DEPOT){
                            robot.rotateToTarget(270, false);
                            if (!robot.rotateToTargetInProgress()) {
                                dumpMarkerDropTimer.set(MOVE_MARKER_DROP_DURATION);
                                subState++;
                            }
                        } else {
                            subState++;
                        }
                        break;
                    case 7:
                        subStateDesc = "Exit";
                        advanceState();
                        break;
                }
                break;
            case DRIVE_TO_CRATER:
                stateDesc = "Drive to crater";
                switch (subState) {

                    case 0:
                        subStateDesc = "Entry";
                        subState++;
                        break;
                    case 1:
                        subState++;
                        /*
                        subStateDesc = "Driving to wall position";
                        if (STARTING_POSITION == CRATER) {
                            robot.driveToTarget(wallPosition, true);
                        }
                        if (!robot.rotateToTargetInProgress() && !robot.driveToTargetInProgress()) {
                            subState++;
                        }*/
                        break;
                    case 2:
                        subStateDesc = "Driving to crater";
                        robot.driveToTarget(craterPositonArray[STARTING_POSITION], false);
                        zThresholdExceeded = robot.imu.exceedZAccelThreshold();
                        if (robot.imu.exceedZAccelThreshold() || (!robot.rotateToTargetInProgress() && !robot.driveToTargetInProgress())) {
                            robot.drivetrain.operate(0.0, 0.0);
                            subState++;
                        }
                        break;
                    case 3:
                        subStateDesc = "Exit";
                        advanceState();
                        break;
                }
                break;
            case END:
                stateDesc = "Ending Auto";
                if (tfod != null) {
                    tfod.shutdown();
                }
                break;
            default:
            break;
        }

        telemetry.addData("State: ", stateDesc);
        telemetry.addData("Substate: ", subStateDesc);
        telemetry.addData("Gold Position: ", goldPosition);
        telemetry.addData("Gold Mineral Detected: ", goldMineralDetected);
        telemetry.addData("Angle to Target: ", robot.angleToTargetDebug);
        telemetry.addData("DriveToTarget in progress: ", robot.driveToTargetInProgress());
        telemetry.addData("RotateToTarget in progress: ", robot.rotateToTargetInProgress());
        telemetry.addData("IMU", robot.imu.getHeading());
        telemetry.addData("X", robot.getCoordinate().getX());
        telemetry.addData("Y", robot.getCoordinate().getY());
        telemetry.addData("Heading", robot.getCoordinate().getHeading());
        telemetry.addData("Distance to target", robot.distanceToTargetDebug);
        telemetry.addData("Rotate Integral", robot.rotateController.getIntegral());
        telemetry.addData("Rotate Derivative", robot.rotateController.getDerivative());
        telemetry.addData("Drive Integral", robot.driveController.getIntegral());
        telemetry.addData("Drive Derivative", robot.driveController.getDerivative());
        telemetry.addData("Rotate Power", robot.rotateController.getOutput());
        telemetry.addData("Drive Power", robot.driveController.getOutput());
        telemetry.addData("Z Exceeded Threshold", zThresholdExceeded);
    }
}