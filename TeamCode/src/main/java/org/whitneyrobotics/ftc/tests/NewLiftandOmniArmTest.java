package org.whitneyrobotics.ftc.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.whitneyrobotics.ftc.subsys.WHSRobotImpl;

@TeleOp (name = "New TeleOp", group = "tests")
public class NewLiftandOmniArmTest extends OpMode{
    WHSRobotImpl robot;
    @Override
    public void init() {
        robot = new WHSRobotImpl(hardwareMap);
    }

    @Override
    public void loop() {
        /* DRIVETRAIN */
        //Precision driving mode
        if (gamepad1.left_bumper) {
            robot.drivetrain.operateWithOrientation(gamepad1.left_stick_y / 2.45, gamepad1.right_stick_y / 2.45);
        } else {
            robot.drivetrain.operateWithOrientation(gamepad1.left_stick_y, gamepad1.right_stick_y);
        }
        robot.drivetrain.switchOrientation(gamepad1.a);

        if (gamepad1.dpad_up) {
            robot.lift.setLiftMotorPower(0.9);
        } else if (gamepad1.dpad_down) {
            robot.lift.setLiftMotorPower(-0.9);
        } else {
            robot.lift.setLiftMotorPower(0.0);
        }

        robot.omniArm.operateIntake(gamepad2.right_bumper,gamepad2.left_bumper, gamepad2.left_trigger>0.01);

    }
}
