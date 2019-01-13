package org.whitneyrobotics.ftc.autoop;

//import com.disnodeteam.dogecv.CameraViewDisplay;
//import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.sun.tools.javac.tree.DCTree;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import org.whitneyrobotics.ftc.lib.subsys.goldpositiondetector.GoldPositionDetector;
import org.whitneyrobotics.ftc.lib.util.Coordinate;
import org.whitneyrobotics.ftc.lib.util.Position;
import org.whitneyrobotics.ftc.lib.util.SimpleTimer;
import org.whitneyrobotics.ftc.subsys.Lift;
import org.whitneyrobotics.ftc.subsys.MarkerDrop;
import org.whitneyrobotics.ftc.subsys.WHSRobotImpl;

import java.util.List;

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

    static final int STARTING_POSITION = CRATER;

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

    public void defineStateEnabledStatus() {
        stateEnabled[INIT] = true;
        stateEnabled[DROP_FROM_LANDER] = true;
        stateEnabled[DRIVE_FROM_LANDER] = true;
        stateEnabled[SAMPLE_PIECE] = true;
        stateEnabled[CLAIM_DEPOT] = false;
        stateEnabled[DRIVE_TO_CRATER] = false;
        stateEnabled[END] = true;
    }

    int currentState;
    int subState;
    String currentStateDesc;
    String subStateDesc;

    SimpleTimer scanParticlesTimer = new SimpleTimer();
    SimpleTimer omniArmMoveTimer = new SimpleTimer();
    SimpleTimer storedToDumpedTimer = new SimpleTimer();
    SimpleTimer dumpedToStoredTimer = new SimpleTimer();

    static final double SCAN_PARTICLES_DURATION = 3.0;
    static final double OMNI_ARM_MOVE_DELAY = 0.38;
    static final double MARKER_DROP_DELAY = 0.75;

    GoldPositionDetector.GoldPosition goldPosition;

    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    private static final String VUFORIA_KEY = "AZpuDDL/////AAABmTx4NapZXku6l0aaFFDwgWsYaPViIxPYFdJ8R9R4gPesBY5Ublbla/sRrihytU6cN9eb5Z30d8FuDcbgxBpdrg7gwPgn8GDXm5EEVpuOZnXYEOlcMTAz1nQcnDTPHcyFz1OKJz17ZoHeYtW70mbD7gqAmu/pP4Zz1cCR44pCFw954WOg7SsmVvuiL6J5iEQFIq68QEdX2sjOK7TmaE3RPATv8pZiU/1pwS5iBxSd+8X7PpBII0Ncc88CsKzrNBhF710j9j6fyHl2BeZhcAcRZ/9Fp1W+Cz2kvMPgI5Ah+FmAvWCOJLverLIYm/lgWhEeQrzpCNhc1eAxjtIAUnq5vhBz++vugvyv8o9fuf8HAxWB";

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    private boolean TFRun = false;
    private boolean goldParticleDetected = false;

    @Override
    public void init() {
        robot = new WHSRobotImpl(hardwareMap);
        robot.lift.liftMotor.setPower(0);
        currentState = INIT;
        subState = 0;

        // from the perspective of blue alliance
        startingCoordinateArray[CRATER] = new Coordinate(350, 350, 150, 47.5);
        startingCoordinateArray[DEPOT] = new Coordinate(-350, 350, 150, 137.5);

        landerClearancePositionArray[CRATER] = new Position(590, 590, 150);
        landerClearancePositionArray[DEPOT] = new Position(-590, 590, 150);

        // setting the three different particle positions for the crater side
        goldPositionArray[CRATER][LEFT] = new Position(590, 1190, 150);//(600,1200, 150);
        goldPositionArray[CRATER][CENTER] = new Position(930, 930, 150);//(900,900,150);
        goldPositionArray[CRATER][RIGHT] = new Position(1190, 590, 150);//(1200,600,150);

        // setting the three different particle positions for the depot side
        goldPositionArray[DEPOT][LEFT] = new Position(-1220,590,150);
        goldPositionArray[DEPOT][CENTER] = new Position(-900,900,150);
        goldPositionArray[DEPOT][RIGHT]=  new Position(-600,1320, 150);

        wallPosition = new Position(-20,1490,150);
        depotCornerPosition = new Position(-1280,1300,150);
        depotSidePosition = new Position(-1400, 1275, 150);

        depotPositionArray[DEPOT] = new Position(-1440,1420,150);
        depotPositionArray[CRATER] = new Position(-1250,1505,150);

        craterPositonArray[CRATER] = new Position(750,1490,150);
        craterPositonArray[DEPOT] = new Position(-1575,-640,150);

        defineStateEnabledStatus();

        // default gold position
        goldPosition = GoldPositionDetector.GoldPosition.CENTER;

        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
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
        telemetry.addData("status", "loop test... waiting for start");
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
                        scanParticlesTimer.set(SCAN_PARTICLES_DURATION);
                        subState++;
                        break;
                    case 1:
                        subStateDesc = "scanning minerals";

                        // particle detection
                        if (tfod != null) {
                            TFRun = true;
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
                                        goldParticleDetected = true;
                                        if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                                            goldPosition = GoldPositionDetector.GoldPosition.LEFT;
                                        } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                                            goldPosition = GoldPositionDetector.GoldPosition.RIGHT;
                                        } else {
                                            goldPosition = GoldPositionDetector.GoldPosition.CENTER;
                                        }
                                    }
                                }
                            }
                        }

                        // advance substate after gold particle is found, or the timer expires
                        if (/*goldParticleDetected ||*/ scanParticlesTimer.isExpired()) {
                            omniArmMoveTimer.set(OMNI_ARM_MOVE_DELAY);
                            subState++;
                        }

                        break;
                    case 2:
                        subStateDesc = "moving arm out of way of lift";
                        robot.omniArm.storeOmniArm(true);
                        if(omniArmMoveTimer.isExpired()){
                            subState++;
                        }
                        break;
                    case 3:
                        subStateDesc = "bringing robot down";
                        robot.lift.bringDownRobot(true);
                        if (robot.lift.getLiftState() == Lift.LiftState.WAITING_FOR_DRIVETRAIN) {
                            subState++;
                        }
                        break;
                    case 4:
                        subStateDesc = "exit";
                        advanceState();
                        break;
                }
                break;
            case DRIVE_FROM_LANDER:
                currentStateDesc = "driving from lander";
                switch (subState) {
                    case 0:
                        subStateDesc = "driving to lander clearance";
                        robot.driveToTarget(landerClearancePositionArray[STARTING_POSITION], false);
                        omniArmMoveTimer.set(OMNI_ARM_MOVE_DELAY);
                        if (!robot.rotateToTargetInProgress() && !robot.driveToTargetInProgress()) {
                            subState++;
                        }
                        break;
                    case 1:
                        subStateDesc = "moving omniarm out of the way";
                        robot.omniArm.storeOmniArm(true);
                        if (omniArmMoveTimer.isExpired()) {
                            subState++;
                        }
                        break;
                    case 2:
                        subStateDesc = "bringing hook down";
                        robot.lift.bringDownHook(true);
                        if (robot.lift.getLiftState() == Lift.LiftState.STANDING_BY_FOR_END_GAME) {
                            subState++;
                        }
                        omniArmMoveTimer.set(OMNI_ARM_MOVE_DELAY);
                        break;
                    case 3:
                        subStateDesc = "storing omniarm";
                        robot.omniArm.resetOmniArm(true);
                        if (omniArmMoveTimer.isExpired()) {
                            subState++;
                        }
                        break;
                    case 4:
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
                            if (STARTING_POSITION == DEPOT) {
                                storedToDumpedTimer.set(MARKER_DROP_DELAY);
                            }
                            subState++;
                        }
                        break;
                    case 2:
                        subStateDesc = "[crater] rotating robot";
                        if (STARTING_POSITION == CRATER) {
                            robot.rotateToTarget(90, true);
                            if (!robot.rotateToTargetInProgress()) {
                                storedToDumpedTimer.set(MARKER_DROP_DELAY);
                                subState++;
                            }
                        } else {
                            subState++;
                        }
                        break;
                    case 3:
                        subStateDesc = "dumping marker";
                        robot.markerDrop.operateMarkerDrop(MarkerDrop.MarkerDropPosition.DUMPED);
                        if (storedToDumpedTimer.isExpired()) {
                            subState++;
                        }
                        dumpedToStoredTimer.set(MARKER_DROP_DELAY);
                        break;
                    case 4:
                        subStateDesc = "storing marker drop";
                        robot.markerDrop.operateMarkerDrop(MarkerDrop.MarkerDropPosition.STORED);
                        if (dumpedToStoredTimer.isExpired()) {
                            subState++;
                        }
                        break;
                    case 5:
                        subStateDesc = "[depot] rotating robot";
                        if (STARTING_POSITION == DEPOT){
                            robot.rotateToTarget(270, false);
                            if (!robot.rotateToTargetInProgress()) {
                                storedToDumpedTimer.set(MARKER_DROP_DELAY);
                                subState++;
                            }
                        } else {
                            subState++;
                        }
                        break;
                    case 6:
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
                        robot.driveToTarget(craterPositonArray[STARTING_POSITION], STARTING_POSITION == CRATER);

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
                if (tfod != null) {
                    tfod.shutdown();
                }
                break;
            default: break;
        }

        telemetry.addData("State: ", currentStateDesc);
        telemetry.addData("Substate: ", subStateDesc);
        telemetry.addData("Gold Position: ", goldPosition);
        telemetry.addData("Gold Particle Detected: ", goldParticleDetected);
        telemetry.addData("TFRun: ", TFRun);
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

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }
}
