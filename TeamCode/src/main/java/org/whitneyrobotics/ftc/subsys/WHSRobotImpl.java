package org.whitneyrobotics.ftc.subsys;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.robot.Robot;
import com.vuforia.Vuforia;

import org.whitneyrobotics.ftc.lib.subsys.robot.WHSRobot;
import org.whitneyrobotics.ftc.lib.util.Coordinate;
import org.whitneyrobotics.ftc.lib.util.Functions;
import org.whitneyrobotics.ftc.lib.util.Position;
import org.whitneyrobotics.ftc.lib.util.RobotConstants;

/**
 * Created by Jason on 10/20/2017.
 */

public class WHSRobotImpl implements WHSRobot {

    public TileRunner drivetrain;
    public IMU imu;
    public OmniArm omniArm;
    public MarkerDrop markerDrop;
    public Lift lift;
    Coordinate currentCoord;
    public double targetHeading; //field frame
    public double angleToTargetDebug;
    private double lastKnownHeading = 0.1;
    private static final double DEADBAND_DRIVE_TO_TARGET = 70; //in mm
    private static final double DEADBAND_ROTATE_TO_TARGET = 1.5; //in degrees
    private static final double[] DRIVE_TO_TARGET_POWER_LEVEL = {0.22, 0.28, 0.4, 0.45}; //{0.33, 0.6, 0.7, 0.9};
    private static final double[] DRIVE_TO_TARGET_THRESHOLD = {DEADBAND_DRIVE_TO_TARGET, 300, 600, 1200};
    private static final double[] ROTATE_TO_TARGET_POWER_LEVEL = {0.2, 0.3, 0.4};
    private static final double[] ROTATE_TO_TARGET_THRESHOLD = {DEADBAND_ROTATE_TO_TARGET, 30, 60};
    private double rightMultiplier = 1.0;
    private int count = 0;
    private boolean driveBackwards;

    private double angleToTargetSum = 0;
    private double lastAngleToTarget = 0;
    private double lastTime /*on WHS Robotics */ = 0;
    private boolean firstRotateLoop = true;
    public double angleToTargetSumDebug = 0;
    public double timeSumDebug = 0;
    public double totalTime = 0;
    public double deltaAngleDebug = 0;
    public double deltaTimeDebug = 0;
    double initialTime = 0;

    private boolean driveToTargetInProgress = false;
    private boolean rotateToTargetInProgress = false;

    private boolean hasDriveToTargetExited;
    private boolean hasRotateToTargetExited;

    public double distanceToTargetDebug = 0;
    public WHSRobotImpl(HardwareMap hardwareMap){
        drivetrain = new TileRunner(hardwareMap);
        currentCoord = new Coordinate(0.0, 0.0, 150.0, 0.0);
        imu = new IMU(hardwareMap);
        omniArm = new OmniArm(hardwareMap);
        markerDrop = new MarkerDrop(hardwareMap);
        lift = new Lift(hardwareMap);
    }

    @Override
    public void driveToTarget(Position targetPos, boolean backwards) {
        Position vectorToTarget = Functions.subtractPositions(targetPos, currentCoord.getPos()); //field frame
        vectorToTarget = field2body(vectorToTarget); //body frame

        double distanceToTarget = Functions.calculateMagnitude(vectorToTarget);
        distanceToTargetDebug = distanceToTarget;
        //TODO test code
        double power = Functions.map(distanceToTarget, RobotConstants.DEADBAND_DRIVE_TO_TARGET, 2500, RobotConstants.drive_min, RobotConstants.drive_max);
        double degreesToRotate = Math.atan2(vectorToTarget.getY(), vectorToTarget.getX()); //from -pi to pi rad
        //double degreesToRotate = Math.atan2(targetPos.getY(), targetPos.getX()); //from -pi to pi rad
        degreesToRotate = degreesToRotate * 180 / Math.PI;
        /*double*/ targetHeading = Functions.normalizeAngle(currentCoord.getHeading() + degreesToRotate); //-180 to 180 deg
        if (!hasRotateToTargetExited()) {
            rotateToTarget(targetHeading, backwards);
            hasDriveToTargetExited = false;
        } else if (!driveBackwards && distanceToTarget > RobotConstants.DEADBAND_DRIVE_TO_TARGET) {
            hasDriveToTargetExited = false;
            driveToTargetInProgress = true;
            drivetrain.operateLeft(power);
            drivetrain.operateRight(power);
        }
        else if (driveBackwards && distanceToTarget > RobotConstants.DEADBAND_DRIVE_TO_TARGET) {
            hasDriveToTargetExited = false;
            driveToTargetInProgress = true;
            drivetrain.operateLeft(-power);
            drivetrain.operateRight(-power);
        }
        else {
            drivetrain.operateRight(0.0);
            drivetrain.operateLeft(0.0);
            driveToTargetInProgress = false;
            rotateToTargetInProgress = false;
            count = 0;
            hasDriveToTargetExited = true;
            hasRotateToTargetExited = false;
        }
    }

    @Override
    public void rotateToTarget(double targetHeading, boolean backwards) {

        double angleToTarget = targetHeading - currentCoord.getHeading();
        angleToTarget = Functions.normalizeAngle(angleToTarget); //-180 to 180 deg

        if (backwards && angleToTarget > 90) {
            angleToTarget = angleToTarget - 180;
            driveBackwards = true;
        }
        else if (backwards && angleToTarget < -90) {
            angleToTarget = angleToTarget + 180;
            driveBackwards = true;
        }
        else {
            driveBackwards = false;
        }

        if (firstRotateLoop) {
            lastTime = System.nanoTime() / 1E9;
            initialTime = lastTime;
            lastAngleToTarget = angleToTarget;
            firstRotateLoop = false;
        }

        double deltaTime = System.nanoTime() / 1E9 - lastTime;
        lastTime = System.nanoTime() / 1E9;
        double deltaAngle = angleToTarget - lastAngleToTarget;
        lastAngleToTarget = angleToTarget;

        angleToTargetSum += angleToTarget * deltaTime;
        angleToTargetSumDebug = angleToTargetSum;
        timeSumDebug += deltaTime;
        totalTime = (System.nanoTime()/1E9)-initialTime;

        angleToTargetDebug = angleToTarget;
        deltaAngleDebug = deltaAngle;
        deltaTimeDebug = deltaTime;

        //drivetrain.setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double power = Functions.map(Math.abs(angleToTarget), RobotConstants.DEADBAND_ROTATE_TO_TARGET, 180, RobotConstants.rotate_min, RobotConstants.rotate_max);
        if (angleToTarget < -RobotConstants.DEADBAND_ROTATE_TO_TARGET) {
            hasRotateToTargetExited = false;
            //targetQuadrant = 4;
            drivetrain.operateLeft(power);
            drivetrain.operateRight(-power);
            rotateToTargetInProgress = true;
        }
        else if (angleToTarget > RobotConstants.DEADBAND_ROTATE_TO_TARGET) {
            hasRotateToTargetExited = false;
            //targetQuadrant = 1;
            drivetrain.operateLeft(-power);
            drivetrain.operateRight(power);
            rotateToTargetInProgress = true;
        }
        else {
            drivetrain.operateLeft(0.0);
            drivetrain.operateRight(0.0);
            rotateToTargetInProgress = false;
            hasRotateToTargetExited = true;
            hasDriveToTargetExited = false;
            firstRotateLoop = true;
        }
    }

    @Override
    public boolean driveToTargetInProgress() {
        return driveToTargetInProgress;
    }

    @Override
    public boolean rotateToTargetInProgress() {
        return rotateToTargetInProgress;
    }

    @Override
    public boolean hasDriveToTargetExited() {
        return hasDriveToTargetExited;
    }

    @Override
    public boolean hasRotateToTargetExited() {
        return hasRotateToTargetExited;
    }

    @Override
    public void estimatePosition() {

        Position estimatedPos;
        if(rotateToTargetInProgress) {
            //if rotating, do NOT update position and get rid of encoder values as it turns
            double[] encoderValues = drivetrain.getEncoderDelta();

            estimatedPos = currentCoord.getPos();
        }
        else {
            if (/*driveToTargetInProgress & */!rotateToTargetInProgress) {
                double[] encoderValues = drivetrain.getEncoderDelta();
                double encoderPosL = encoderValues[0];
                double encoderPosR = encoderValues[1];

                double encoderAvg = (encoderPosL + encoderPosR) * 0.5;
                double hdg = currentCoord.getHeading();
                double dist = drivetrain.encToMM(encoderAvg);

                double xPos = currentCoord.getX() + dist * Functions.cosd(hdg);
                double yPos = currentCoord.getY() + dist * Functions.sind(hdg);

                estimatedPos = new Position(xPos, yPos, currentCoord.getZ());

                currentCoord.setX(xPos);
                currentCoord.setY(yPos);
            } else if (rotateToTargetInProgress) {
                drivetrain.getEncoderDelta();
                estimatedPos = currentCoord.getPos();

            } else {
                estimatedPos = currentCoord.getPos();
            }
        }


    }

    @Override
    public void estimateHeading() {
        double currentHeading;
        currentHeading = Functions.normalizeAngle(imu.getHeading() + imu.getImuBias()); //-180 to 180 deg
        if (currentHeading != 0.0) {
            lastKnownHeading = currentHeading;
        }
        currentCoord.setHeading(lastKnownHeading); //updates global variable
    }

    @Override
    public void setInitialCoordinate(Coordinate initCoord) {
        currentCoord = initCoord;
        imu.setImuBias(currentCoord.getHeading());
    }

    @Override
    public void setCoordinate(Coordinate coord) {
        currentCoord = coord;
        imu.setImuBias(currentCoord.getHeading());
    }

    @Override
    public Coordinate getCoordinate() {
        return currentCoord;
    }


    public Position body2field(Position bodyVector)
    {
        Position fieldVector;
        double heading = currentCoord.getHeading();

        double[][] C_b2f = {{Functions.cosd(heading),  -Functions.sind(heading),  0},
                {Functions.sind(heading),   Functions.cosd(heading),  0},
                {0,                         0,                        1}};

        fieldVector = Functions.transformCoordinates(C_b2f,bodyVector);
        return fieldVector;

    }

    public Position field2body(Position fieldVector)
    {
        Position bodyVector;
        double heading = currentCoord.getHeading();

        double[][] C_f2b = {{ Functions.cosd(heading),   Functions.sind(heading),  0},
                {-Functions.sind(heading),   Functions.cosd(heading),  0},
                { 0,                         0,                        1}};

        bodyVector = Functions.transformCoordinates(C_f2b,fieldVector);
        return bodyVector;

    }

    public Position front2back(Position frontVector)
    {
        Position backVector;
        double heading = 180;

        double[][] C_f2b = {{ -1,  0, 0},
                {  0, -1, 0},
                {  0,  0, 1}};

        backVector = Functions.transformCoordinates(C_f2b,frontVector);
        return backVector;
    }
}
