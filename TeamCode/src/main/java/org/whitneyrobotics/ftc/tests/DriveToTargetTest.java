package org.whitneyrobotics.ftc.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.whitneyrobotics.ftc.lib.util.Coordinate;
import org.whitneyrobotics.ftc.lib.util.Position;
import org.whitneyrobotics.ftc.subsys.WHSRobotImpl;
import com.acmerobotics.dashboard.telemetry.*;
import com.acmerobotics.dashboard.FtcDashboard;


/**
 * Created by Amar2 on 11/15/2017.
 */
@Autonomous(name = "DriveToTargetTest", group = "tests")

public class DriveToTargetTest extends OpMode {
    WHSRobotImpl robot;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    Position p1 = new Position(600, 600, 150);
    Position p2 = new Position(600,1200,150);
    boolean backwards = true;
    boolean b = true;
    int i = 0;
    @Override
    public void init() {
        TelemetryPacket packet = new TelemetryPacket();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new WHSRobotImpl(hardwareMap);
        robot.setInitialCoordinate(new Coordinate(0, 0, 150, 0));
        telemetry.setMsTransmissionInterval(10);
    }

    @Override
    public void start(){
    }

    @Override
    public void loop() {

        switch (i){
            case 0:
                robot.rotateToTarget(45, true);
                if (!robot.rotateToTargetInProgress()) {
                    i = 1;
                }
                break;
            case 1:
                robot.rotateToTarget(-90, true);
                if (!robot.rotateToTargetInProgress()) {
                    i = 2;
                    try {
                        Thread.sleep(500);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                }
                break;
            case 2:
                robot.driveToTarget(p1, true);
                if (!robot.driveToTargetInProgress() && !robot.rotateToTargetInProgress()){
                    i = 3;
                    try {
                        Thread.sleep(500);
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                }
                break;
            case 3:
                robot.driveToTarget(p2, false);
                if (!robot.driveToTargetInProgress() && !robot.rotateToTargetInProgress()){
                    i = 4;
                }
                break;
            case 4:
                break;
        }

        robot.estimatePosition();
        robot.estimateHeading();

        telemetry.addData("State", i);
        telemetry.addData("Angle to Target: ", robot.angleToTargetDebug);
        telemetry.addData("DriveToTarget in progress: ", robot.driveToTargetInProgress());
        telemetry.addData("RotateToTarget in progress: ", robot.rotateToTargetInProgress());
        telemetry.addData("IMU", robot.imu.getHeading());
        telemetry.addData("X", robot.getCoordinate().getX());
        telemetry.addData("Y", robot.getCoordinate().getY());
        telemetry.addData("Z", robot.getCoordinate().getZ());
        telemetry.addData("Heading", robot.getCoordinate().getHeading());
        telemetry.addData("FL Power", robot.drivetrain.frontLeft.getPower());
        telemetry.addData("BL Power", robot.drivetrain.backLeft.getPower());
        telemetry.addData("FR Power", robot.drivetrain.frontRight.getPower());
        telemetry.addData("BR Power", robot.drivetrain.backRight.getPower());
        telemetry.addData("Distance to target", robot.distanceToTargetDebug);
        telemetry.addData("BLdelta", robot.drivetrain.backLeft.getCurrentPosition());
        telemetry.addData("BRdelta", robot.drivetrain.backRight.getCurrentPosition());
        telemetry.addData("FLdelta", robot.drivetrain.frontLeft.getCurrentPosition());
        telemetry.addData("FRdelta", robot.drivetrain.frontRight.getCurrentPosition());
        telemetry.addData("Rotate Integral", robot.rotateController.getIntegral());
        telemetry.addData("Rotate Derivative", robot.rotateController.getDerivative());
        telemetry.addData("Drive Integral", robot.driveController.getIntegral());
        telemetry.addData("Drive Derivative", robot.driveController.getDerivative());
        telemetry.addData("Rotate Power", robot.rotateController.getOutput());
        telemetry.addData("Drive Power", robot.driveController.getOutput());
    }
}
