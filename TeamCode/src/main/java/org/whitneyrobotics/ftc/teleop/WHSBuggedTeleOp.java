package org.whitneyrobotics.ftc.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.whitneyrobotics.ftc.lib.util.RobotConstants;
import org.whitneyrobotics.ftc.lib.util.Toggler;
import org.whitneyrobotics.ftc.subsys.WHSRobotImpl;

import java.util.ArrayList;

@TeleOp(name="BuggedTeleOp", group="tele")
public class WHSBuggedTeleOp extends OpMode{

    WHSRobotImpl robot;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();

    private double leftMultiplier = 1, rightMultiplier = 1;
    private double totalMultiplier = 1;
    private boolean canDrive = true;
    private boolean canIntake = true;
    private boolean canExtend = true;
    private boolean canStoreArm = true;
    private boolean canLift = true;

    @Override
    public void init() {
        robot = new WHSRobotImpl(hardwareMap);
    }

    @Override
    public void loop() {
        /* Update variables based on Dashboard */
        leftMultiplier = RobotConstants.leftMultiplier;
        rightMultiplier = RobotConstants.rightMultiplier;
        totalMultiplier = RobotConstants.totalMultiplier;
        canDrive = RobotConstants.canDrive;
        canIntake = RobotConstants.canIntake;
        canExtend = RobotConstants.canExtend;
        canStoreArm = RobotConstants.canStoreArm;
        canLift = RobotConstants.canLift;

        packet.fieldOverlay()
                .setStroke("blue")
                .strokeRect(robot.getCoordinate().getX()/100/2.54, robot.getCoordinate().getY()/100/2.54, 20, 20);
        packet.put("Switch Current Pos ", robot.omniArm.switchMotor.getCurrentPosition());
        packet.put("Switch Target Pos ", robot.omniArm.switchMotor.getTargetPosition());
        packet.put("Distance Sensor Distance ", robot.lift.distancer.getDistance(DistanceUnit.MM));
        packet.put("Coordinates", robot.getCoordinate());
        packet.put("Sensor Lift", gamepad1.y);
        packet.put("Lift State", robot.lift.getSensorLiftState());
        dashboard.sendTelemetryPacket(packet);

        /* DRIVETRAIN */
        //Precision driving mode
        if (canDrive) {
            if (gamepad1.left_bumper || gamepad2.b) {
                robot.drivetrain.operateWithOrientation(gamepad1.left_stick_y / 2.54 * leftMultiplier * totalMultiplier, gamepad1.right_stick_y / 2.54 * rightMultiplier * totalMultiplier);
            } else {
                robot.drivetrain.operateWithOrientation(gamepad1.left_stick_y * leftMultiplier * totalMultiplier, gamepad1.right_stick_y * rightMultiplier * totalMultiplier);
            }
            robot.drivetrain.switchOrientation(gamepad1.a);
        }

        if (canIntake) {
            robot.omniArm.operateIntake(gamepad2.right_bumper, gamepad2.left_bumper);
        }
        if (canExtend) {
            robot.omniArm.operateExtension(gamepad2.a);
        }
        if (canLift) {
            robot.lift.sensorLift(gamepad1.y);
        }
        if (canStoreArm) {
            if (gamepad1.right_bumper) {
                robot.omniArm.storeOmniArm(gamepad1.right_bumper);

            } else {
                robot.omniArm.operateModeSwitch(gamepad2.x);
            }
        }

        if (canLift) {
            if (gamepad2.dpad_down) {
                if (gamepad1.dpad_up) {
                    robot.lift.setLiftMotorPower(0.6);
                } else if (gamepad1.dpad_down) {
                    robot.lift.setLiftMotorPower(-0.66);
                } else {
                    robot.lift.setLiftMotorPower(0.0);
                }
            } else {
                robot.lift.liftUpRobot(gamepad2.dpad_up);
            }
        }
    }
}
