package org.whitneyrobotics.ftc.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.whitneyrobotics.ftc.subsys.WHSRobotImpl;


@TeleOp(name="TeleOp", group="tele")
public class WHSTeleOp extends OpMode{

    WHSRobotImpl robot;
    @Override
    public void init() {
        robot = new WHSRobotImpl(hardwareMap);
    }

    @Override
    public void loop() {

        /* DRIVETRAIN */
        //Precision driving mode
        if (gamepad1.left_bumper || gamepad2.b) {
            robot.drivetrain.operateWithOrientation(gamepad1.left_stick_y / 2.54, gamepad1.right_stick_y / 2.54);
        } else {
            robot.drivetrain.operateWithOrientation(gamepad1.left_stick_y, gamepad1.right_stick_y);
        }

        robot.drivetrain.switchOrientation(gamepad1.a);

        robot.omniArm.operateIntake(gamepad2.right_bumper, gamepad2.left_bumper);
        robot.omniArm.operateExtension(gamepad2.y);
        robot.omniArm.operateModeSwitch(gamepad2.x);
        telemetry.addData("Switch Current Pos", robot.omniArm.switchMotor.getCurrentPosition());
        telemetry.addData("Switch Target Pos", robot.omniArm.switchMotor.getTargetPosition());


        if (gamepad1.left_trigger>0.01 && gamepad1.right_trigger>0.01){
            robot.lift.liftUpRobot();
        }
        
    }
}
