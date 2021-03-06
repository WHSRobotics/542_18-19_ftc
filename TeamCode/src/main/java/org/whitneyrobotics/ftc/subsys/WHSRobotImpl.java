package org.whitneyrobotics.ftc.subsys;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.whitneyrobotics.ftc.lib.subsys.robot.WHSRobot;
import org.whitneyrobotics.ftc.lib.util.Coordinate;
import org.whitneyrobotics.ftc.lib.util.Functions;
import org.whitneyrobotics.ftc.lib.util.PIDController;
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
    private double targetHeading; //field frame
    public double angleToTargetDebug;
    public double distanceToTargetDebug = 0;
    private double lastKnownHeading = 0.1;

    private static double DEADBAND_DRIVE_TO_TARGET = RobotConstants.DEADBAND_DRIVE_TO_TARGET; //in mm
    private static double DEADBAND_ROTATE_TO_TARGET = RobotConstants.DEADBAND_ROTATE_TO_TARGET; //in degrees

    public static double DRIVE_MIN = RobotConstants.drive_min;
    public static double DRIVE_MAX = RobotConstants.drive_max;
    public static double ROTATE_MIN = RobotConstants.rotate_min;
    public static double ROTATE_MAX = RobotConstants.rotate_max;

    private static double ROTATE_KP = RobotConstants.R_KP;
    private static double ROTATE_KI = RobotConstants.R_KI;
    private static double ROTATE_KD = RobotConstants.R_KD;

    private static double DRIVE_KP = RobotConstants.D_KP;
    private static double DRIVE_KI = RobotConstants.D_KI;
    private static double DRIVE_KD = RobotConstants.D_KD;

    public PIDController rotateController = new PIDController(ROTATE_KP, ROTATE_KI, ROTATE_KD);
    public PIDController driveController = new PIDController(DRIVE_KP, DRIVE_KI, DRIVE_KD);

    private boolean firstRotateLoop = true;
    private boolean firstDriveLoop = true;
    private boolean driveBackwards;

    private int driveSwitch = 0;

    private boolean driveToTargetInProgress = false;
    private boolean rotateToTargetInProgress = false;

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
        double distanceToTarget = vectorToTarget.getX()/*Functions.calculateMagnitude(vectorToTarget) * (vectorToTarget.getX() >= 0 ? 1 : -1)*/;
        distanceToTargetDebug = distanceToTarget;

        double degreesToRotate = Math.atan2(vectorToTarget.getY(), vectorToTarget.getX()); //from -pi to pi rad
        degreesToRotate = degreesToRotate * 180 / Math.PI;
        targetHeading = Functions.normalizeAngle(currentCoord.getHeading() + degreesToRotate); //-180 to 180 deg

        switch (driveSwitch) {
            case 0:
                driveToTargetInProgress = true;
                rotateToTarget(targetHeading, backwards);
                if (!rotateToTargetInProgress()) {
                    driveSwitch = 1;
                }
                break;
            case 1:

                if (firstDriveLoop) {
                    driveToTargetInProgress = true;
                    driveController.init(distanceToTarget);
                    firstDriveLoop = false;
                }

                driveController.setConstants(DRIVE_KP, DRIVE_KI, DRIVE_KD);
                driveController.calculate(distanceToTarget);

                double power = Functions.map(Math.abs(driveController.getOutput()), DEADBAND_DRIVE_TO_TARGET, 1500, DRIVE_MIN, DRIVE_MAX);

                // this stuff may be causing the robot to oscillate around the target position
                if (distanceToTarget < 0) {
                    power = -power;
                } else if (distanceToTarget > 0){
                    power = Math.abs(power);
                }
                if (Math.abs(distanceToTarget) > DEADBAND_DRIVE_TO_TARGET) {
                    driveToTargetInProgress = true;
                    drivetrain.operateLeft(power);
                    drivetrain.operateRight(power);
                } else {
                    drivetrain.operateRight(0.0);
                    drivetrain.operateLeft(0.0);
                    driveToTargetInProgress = false;
                    rotateToTargetInProgress = false;
                    firstDriveLoop = true;
                    driveSwitch = 0;
                }
                // end of weird code
                break;
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

        angleToTargetDebug = angleToTarget;

        if (firstRotateLoop) {
            rotateToTargetInProgress = true;
            rotateController.init(angleToTarget);
            firstRotateLoop = false;
        }

        rotateController.setConstants(ROTATE_KP, ROTATE_KI, ROTATE_KD);
        rotateController.calculate(angleToTarget);

        double power = (rotateController.getOutput() >= 0 ? 1 : -1) * (Functions.map(Math.abs(rotateController.getOutput()),  0, 180, ROTATE_MIN, ROTATE_MAX));

        if (Math.abs(angleToTarget) > DEADBAND_ROTATE_TO_TARGET) {
            drivetrain.operateLeft(-power);
            drivetrain.operateRight(power);
            rotateToTargetInProgress = true;
        }
        else {
            drivetrain.operateLeft(0.0);
            drivetrain.operateRight(0.0);
            rotateToTargetInProgress = false;
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
