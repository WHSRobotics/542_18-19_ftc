package org.whitneyrobotics.ftc.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.whitneyrobotics.ftc.subsys.WHSRobotImpl;



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

        if (gamepad2.right_trigger > 0.01){
            robot.omniArm.intake();
        }

        if (gamepad2.left_trigger > 0.01){
            robot.omniArm.outtake();
        }

        if (gamepad2.x){
            robot.omniArm.changeMode();
        }


        if (gamepad1.left_trigger>0.01 && gamepad1.right_trigger>0.01){
            robot.lift.liftUpRobot();
        }
        
    }
}
