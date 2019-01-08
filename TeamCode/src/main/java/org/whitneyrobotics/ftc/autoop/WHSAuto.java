package org.whitneyrobotics.ftc.autoop;

//import com.disnodeteam.dogecv.CameraViewDisplay;
//import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.sun.tools.javac.tree.DCTree;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
//import org.whitneyrobotics.ftc.lib.subsys.goldpositiondetector.GoldPositionDetector;
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
    Position intermediatePosition;
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

    static final double OMNI_ARM_MOVE_DELAY = .7;
    static final double MARKER_DROP_DELAY = 0.65;
    static final double DRIVE_FORWARD_SMALL_BIT_DURATION = 0.2;

    //private GoldAlignDetector detector;
    GoldPositionDetector.GoldPosition goldPosition;
    //double xpos;

    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    private static final String VUFORIA_KEY = "AZpuDDL/////AAABmTx4NapZXku6l0aaFFDwgWsYaPViIxPYFdJ8R9R4gPesBY5Ublbla/sRrihytU6cN9eb5Z30d8FuDcbgxBpdrg7gwPgn8GDXm5EEVpuOZnXYEOlcMTAz1nQcnDTPHcyFz1OKJz17ZoHeYtW70mbD7gqAmu/pP4Zz1cCR44pCFw954WOg7SsmVvuiL6J5iEQFIq68QEdX2sjOK7TmaE3RPATv8pZiU/1pwS5iBxSd+8X7PpBII0Ncc88CsKzrNBhF710j9j6fyHl2BeZhcAcRZ/9Fp1W+Cz2kvMPgI5Ah+FmAvWCOJLverLIYm/lgWhEeQrzpCNhc1eAxjtIAUnq5vhBz++vugvyv8o9fuf8HAxWB";

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;

    @Override
    public void init() {
        robot = new WHSRobotImpl(hardwareMap);
        robot.lift.liftMotor.setPower(0);
        currentState = INIT;
        subState = 0;

        // from the perspective of blue alliance
        startingCoordinateArray[CRATER] = new Coordinate(350, 350, 150, 47.5);
        startingCoordinateArray[DEPOT] = new Coordinate(-350, 350, 150, 135);

        landerClearancePositionArray[CRATER] = new Position(590, 590, 150);
        landerClearancePositionArray[DEPOT] = new Position(-590, 590, 150);

        // setting the three different particle positions for the crater side
        goldPositionArray[CRATER][LEFT] = new Position(590, 1190, 150);//(600,1200, 150);
        goldPositionArray[CRATER][CENTER] = new Position(890, 890, 150);//(900,900,150);
        goldPositionArray[CRATER][RIGHT] = new Position(1190, 590, 150);//(1200,600,150);

        // setting the three different particle positions for the depot side
        goldPositionArray[DEPOT][LEFT] = new Position(-1190,590,150);
        goldPositionArray[DEPOT][CENTER] = new Position(-900,900,150);
        goldPositionArray[DEPOT][RIGHT]=  new Position(-600,1200, 150);

        wallPosition = new Position(-50,1420,150);

        intermediatePosition = new Position(-1625,0,150);

        depotCornerPosition = new Position(-1280,1300,150);
        depotSidePosition = new Position(-1550, 1300, 150);

        depotPositionArray[DEPOT] = new Position(-1440,1420,150);
        depotPositionArray[CRATER] = new Position(-1300,1425,150);

        craterPositonArray[CRATER] = new Position(800,1440,150);
        craterPositonArray[DEPOT] = new Position(-1625,-700,150);

        defineStateEnabledStatus();

        // vision initialization
        /*
        detector = new GoldAlignDetector();
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        detector.useDefaults();
        detector.enable();
        */

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
        /** Activate Tensor Flow Object Detection. */
        if (tfod != null) {
            tfod.activate();
        }
    }

    @Override
    public void loop() {
        robot.estimateHeading();
        robot.estimatePosition();

        if (tfod != null) {
            tfod.shutdown();
        }

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
                        /*
                        xpos = detector.getXPosition();
                        if (0< xpos && xpos < 230) {
                            goldPosition = GoldPositionDetector.GoldPosition.LEFT;
                        } else if (xpos >= 230) {
                            goldPosition = GoldPositionDetector.GoldPosition.CENTER;
                        } else if (xpos==0){
                            goldPosition = GoldPositionDetector.GoldPosition.RIGHT;
                        }
                        */
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
                                        if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                                            telemetry.addData("Gold Mineral Position", "Left");
                                            goldPosition = GoldPositionDetector.GoldPosition.LEFT;
                                        } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                                            telemetry.addData("Gold Mineral Position", "Right");
                                            goldPosition = GoldPositionDetector.GoldPosition.RIGHT;
                                        } else {
                                            telemetry.addData("Gold Mineral Position", "Center");
                                            goldPosition = GoldPositionDetector.GoldPosition.CENTER;
                                        }
                                    }
                                }
                                telemetry.update();
                            }
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
                        if (STARTING_POSITION == DEPOT){
                            robot.driveToTarget(intermediatePosition, false);
                        }else if (STARTING_POSITION == CRATER) {
                            robot.driveToTarget(craterPositonArray[STARTING_POSITION], STARTING_POSITION == CRATER);
                        }
                        if (!robot.rotateToTargetInProgress() && !robot.driveToTargetInProgress()) {
                            subState++;
                        }
                        break;
                    case 1:
                        if (STARTING_POSITION == DEPOT){
                            robot.driveToTarget(craterPositonArray[DEPOT], false);
                        }else if (STARTING_POSITION == CRATER){
                            subState++;
                        }

                        if (!robot.driveToTargetInProgress() && !robot.rotateToTargetInProgress()){
                            subState++;
                        }
                    case 2:
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

        telemetry.addData("Xpos", detector.getXPosition());
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