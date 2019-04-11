package org.whitneyrobotics.ftc.autoop;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

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

@Autonomous(name = "WHSAutoCrater", group = "auto")
public class WHSAutoCrater extends OpMode{

    WHSRobotImpl robot;

    /**
     * Positioning
     */
    Coordinate startingCoordinate;
    Position landerClearancePosition;
    Position[] goldPositionArray = new Position[3];
    Position wallPosition;
    Position depotPosition;
    Position outtakePosition;

    static final int LEFT = 0;
    static final int CENTER = 1;
    static final int RIGHT = 2;

    /**
     * State Definitions
     */
    static final int INIT = 0;
    static final int DROP_FROM_LANDER = 1;
    static final int DRIVE_FROM_LANDER = 2;
    static final int CLAIM_DEPOT = 3;
    static final int SAMPLE_MINERAL = 4;
    static final int END = 5;

    static final int NUM_OF_STATES = 6;

    boolean[] stateEnabled = new boolean[NUM_OF_STATES];

    int state = INIT;
    int subState = 0;
    int goldPosition = CENTER;
    String stateDesc;
    String subStateDesc;
    boolean shouldHookBeDown= false;
    boolean zThresholdExceeded = false;

    /**
     * Timers
     */
    SimpleTimer scanMineralsTimer = new SimpleTimer();
    SimpleTimer dumpMarkerDropTimer = new SimpleTimer();
    SimpleTimer storeMarkerDropTimer = new SimpleTimer();
    SimpleTimer halfExtendArmTimer = new SimpleTimer();
    SimpleTimer intakeMineralsTimer = new SimpleTimer();
    SimpleTimer outtakeMineralsTimer = new SimpleTimer();

    static final double SCAN_MINERALS_DURATION = 2.0;
    static final double MOVE_MARKER_DROP_DURATION = 0.75;
    static final double HALF_EXTEND_ARM_DURATION = 0.542;
    static final double INTAKE_MINERALS_DURATION = 2.0;
    static final double OUTTAKE_MINERALS_DURATION =2.0;

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
        startingCoordinate = new Coordinate(350, 350, 150, 47.5);

        // Position in which we move the robot to allow for the lift to go down
        landerClearancePosition = new Position(590, 590, 150);

        // setting the three different mineral positions for the crater side
        goldPositionArray[LEFT] = new Position(500, 1250, 150);
        goldPositionArray[CENTER] = new Position(930, 930, 150);
        goldPositionArray[RIGHT] = new Position(1250, 500, 150);

        // rAndOm cRaTer and dEpOt pOsiTiOns
        wallPosition = new Position(-15, 1450, 150);

        depotPosition = new Position(-1290, 1490, 150);

        outtakePosition = new Position(300, 650, 150);

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
        if (robot.lift.getCurrentLiftPosition() != Lift.LiftPosition.STORED) {
            robot.lift.bringDownHook(shouldHookBeDown);
        }
        robot.estimateHeading();
        robot.estimatePosition();

        switch (state) {
            case INIT:
                stateDesc = "Starting Auto";
                robot.setInitialCoordinate(startingCoordinate);
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
                        robot.driveToTarget(landerClearancePosition, false);
                        if (!robot.rotateToTargetInProgress() && !robot.driveToTargetInProgress()) {
                            subState++;
                        }
                        break;
                    case 2:
                        subStateDesc = "Bringing hook down";
                        shouldHookBeDown = true;
                        subState++;
                        break;
                     case 3:
                        subStateDesc = "Exit";
                        advanceState();
                        break;
                }
                break;
            case CLAIM_DEPOT:
                stateDesc = "Claim depot";
                switch (subState) {
                    case 0:
                        subStateDesc = "Entry";
                        subState++;
                    case 1:
                        robot.driveToTarget(wallPosition, true);
                        if (!robot.driveToTargetInProgress() && !robot.rotateToTargetInProgress()) {
                            subState++;
                        }
                        break;
                    case 2:
                        subStateDesc = "Driving to depot";
                        robot.driveToTarget(depotPosition, true);
                        if (!robot.rotateToTargetInProgress() && !robot.driveToTargetInProgress()) {
                            subState++;
                        }
                        break;
                    case 3:
                        subStateDesc = "Rotating Robot";
                            robot.rotateToTarget(225, false);
                            if (!robot.rotateToTargetInProgress() && !robot.driveToTargetInProgress()) {
                                dumpMarkerDropTimer.set(MOVE_MARKER_DROP_DURATION);
                                subState++;
                            }
                        break;
                    case 4:
                        subStateDesc = "Dumping Marker";
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
                        subStateDesc = "Exit";
                        advanceState();
                        break;
                }
                break;
            case SAMPLE_MINERAL:
                stateDesc = "Sampling Mineral";
                switch (subState) {
                    case 0:
                        subStateDesc = "Entry";
                        subState++;
                        break;
                    case 1:
                        subStateDesc = "Driving to intermediate position";
                        robot.driveToTarget(wallPosition, true);
                        if (!robot.rotateToTargetInProgress() && !robot.driveToTargetInProgress()) {
                            subState++;
                        }
                        break;
                    case 2:
                        subStateDesc = "Driving to Lander Clearance";
                        robot.driveToTarget(landerClearancePosition, true);
                        if (!robot.rotateToTargetInProgress() && !robot.driveToTargetInProgress()) {
                            subState++;
                        }
                        break;
                    case 3:
                        subStateDesc = "Rotating robot";
                        if (goldPosition == LEFT) {
                            robot.rotateToTarget(90, false);
                        }
                        if (goldPosition == RIGHT) {
                            robot.rotateToTarget(-8, false);
                        }
                        if (goldPosition == CENTER) {
                            robot.rotateToTarget(45, false);
                        }
                        if (!robot.rotateToTargetInProgress()) {
                            subState++;
                        }
                        break;
                    case 4:
                        subStateDesc = "Intermediate-ing pivot motor";
                        robot.omniArm.setPivotPosition(OmniArm.PivotPosition.INTERMEDIATE);
                        if (robot.omniArm.getCurrentPivotPosition() == OmniArm.PivotPosition.INTERMEDIATE) {
                            halfExtendArmTimer.set(HALF_EXTEND_ARM_DURATION);
                            subState++;
                        }
                        break;
                    case 5:
                        subStateDesc = "Half-extending arm";
                        robot.omniArm.setExtendPosition(OmniArm.ExtendPosition.INTERMEDIATE);
                        if (halfExtendArmTimer.isExpired()) {
                            subState++;
                        }
                        break;
                    case 6:
                        subStateDesc = "Intaking mineral";
                        robot.omniArm.setExtendPosition(OmniArm.ExtendPosition.INTERMEDIATE);
                        robot.omniArm.setPivotPosition(OmniArm.PivotPosition.AUTO_INTAKE);
                        robot.omniArm.operateIntake(false, true, false);
                        robot.omniArm.operateIntakeClearence(false);
                        if (robot.omniArm.getCurrentPivotPosition() == OmniArm.PivotPosition.AUTO_INTAKE) {
                            intakeMineralsTimer.set(INTAKE_MINERALS_DURATION);
                            subState++;
                        }
                        break;
                        /*
                    case 7:
                        subStateDesc = "bringing up omni arm to intermediate";
                        robot.omniArm.setPivotPosition(OmniArm.PivotPosition.INTERMEDIATE);
                        if (robot.omniArm.getCurrentPivotPosition() == OmniArm.PivotPosition.INTERMEDIATE){
                            robot.omniArm.operateIntake(false,false,false);
                            subState++;
                        }
                        break;
                    case 8:
                        subStateDesc = "Bringing OmniArm back in a Little";
                        robot.omniArm.setExtendPosition(OmniArm.ExtendPosition.OUTTAKE);
                        if (robot.omniArm.getCurrentExtendPosition() == OmniArm.ExtendPosition.OUTTAKE) {
                            subState++;
                        }
                        break;
                        */
                    case 7:
                        subStateDesc = "Intaking mineral";
                        if (intakeMineralsTimer.isExpired()) {
                            subState++;
                        }
                        break;
                    case 8:
                        subStateDesc = "Bringing OmniArm Up";
                        robot.omniArm.operateIntake(false, false, false);
                        robot.omniArm.setPivotPosition(OmniArm.PivotPosition.OUTTAKE);
                        if (robot.omniArm.getCurrentPivotPosition() == OmniArm.PivotPosition.OUTTAKE){
                            subState++;
                        }
                        break;
                    case 9:
                        subStateDesc = "extending arm to outtake";
                        robot.omniArm.setExtendPosition(OmniArm.ExtendPosition.INTAKE);
                        if (robot.omniArm.getCurrentExtendPosition() == OmniArm.ExtendPosition.INTAKE) {
                            subState++;
                        }
                        break;
                    case 10:
                        subStateDesc = "Driving to outtake position";
                        robot.driveToTarget(outtakePosition, true);
                        if (!robot.driveToTargetInProgress() && !robot.rotateToTargetInProgress()){
                            outtakeMineralsTimer.set(OUTTAKE_MINERALS_DURATION);
                            subState++;
                        }
                        break;
                    case 11:
                        subStateDesc = "Rotating to face lander";
                        robot.rotateToTarget(45, false);
                        if (!robot.rotateToTargetInProgress()) {
                            subState++;
                        }
                        break;
                    case 12:
                        subStateDesc = "Outtaking gold mineral";
                        robot.omniArm.operateIntakeClearence(true);
                        robot.omniArm.operateIntake(false, true, false);
                        if (outtakeMineralsTimer.isExpired()) {
                            subState++;
                        }
                        break;
                    case 13:
                        subStateDesc = "Driving to lander clearance";
                        robot.driveToTarget(landerClearancePosition, false);
                        if (!robot.driveToTargetInProgress() && !robot.rotateToTargetInProgress()) {
                            subState++;
                        }
                        break;
                    case 14:
                        subStateDesc = "Retracting Arm";
                        robot.omniArm.operateIntake(false, false, false);
                        robot.omniArm.setExtendPosition(OmniArm.ExtendPosition.RETRACTED);
                        if (robot.omniArm.getCurrentExtendPosition() == OmniArm.ExtendPosition.RETRACTED) {
                            subState++;
                        }
                        break;
                    case 15:
                        robot.omniArm.setPivotPosition(OmniArm.PivotPosition.AUTO_INTERMEDIATE);
                        if (robot.omniArm.getCurrentPivotPosition() == OmniArm.PivotPosition.AUTO_INTERMEDIATE){
                            subState++;
                        }
                        break;
                    case 16:
                        robot.omniArm.setExtendPosition(OmniArm.ExtendPosition.INTAKE);
                        if (robot.omniArm.getCurrentExtendPosition() == OmniArm.ExtendPosition.INTAKE){
                            subState++;
                        }
                        break;
                    case 17:
                        subStateDesc = "Exit";
                        robot.omniArm.pivotMotor.setPower(0.0);
                        robot.omniArm.extendMotor.setPower(0.0);
                        advanceState();
                        break;
                }
                break;
            case END:
                if (tfod != null) {
                    tfod.shutdown();
                }

                /*
                switch (subState) {
                    case 0:
                        subStateDesc = "Driving to gold position";
                        robot.driveToTarget(goldPositionArray[goldPosition], false);
                        if (!robot.driveToTargetInProgress() || !robot.rotateToTargetInProgress()) {
                            subState++;
                        }
                        break;
                    case 1:
                        subStateDesc = "Lowering OmniArm";
                        robot.omniArm.setPivotPosition(OmniArm.PivotPosition.INTAKE);
                        robot.omniArm.operateIntake(true, false, false);
                        if (robot.omniArm.getCurrentPivotPosition() == OmniArm.PivotPosition.INTAKE) {
                            subState++;
                        }
                        break;
                    case 2:
                        subStateDesc = "Bringing up OmniArm";
                        robot.omniArm.setPivotPosition(OmniArm.PivotPosition.OUTTAKE);
                        if (robot.omniArm.getCurrentPivotPosition() == OmniArm.PivotPosition.OUTTAKE) {
                            outtakeMineralsTimer.set(OUTTAKE_MINERALS_DURATION);
                            subState++;
                        }
                        break;
                    case 3:
                        subStateDesc = "Driving to outtake position";
                        robot.driveToTarget(outtakePosition, true);
                        robot.omniArm.operateIntake(true, false, false);
                        if (!robot.driveToTargetInProgress() && !robot.rotateToTargetInProgress()){
                            subState++;
                        }
                        break;
                    case 4:
                        subStateDesc = "Outtaking gold mineral";
                        robot.omniArm.operateIntakeClearence(true);
                        if (outtakeMineralsTimer.isExpired()) {
                            subState++;
                        }
                        break;
                    case 5:
                        subStateDesc = "Repeat";
                        subState = 0;
                        break;
                }
                */
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