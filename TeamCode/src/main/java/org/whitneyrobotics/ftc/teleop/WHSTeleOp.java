package org.whitneyrobotics.ftc.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.whitneyrobotics.ftc.subsys.WHSRobotImpl;


@TeleOp(name="TeleOp", group="tele")
public class WHSTeleOp extends OpMode{

    WHSRobotImpl robot;
    long i = 0;
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
        robot.omniArm.operateExtension(gamepad2.a);
       // robot.lift.sensorLift(gamepad1.y);
        telemetry.addData("Sensor Lift", gamepad1.y);
        telemetry.addData("Lift State", robot.lift.getSensorLiftState());
        if (gamepad1.right_bumper) {
           robot.omniArm.storeOmniArm(gamepad1.right_bumper);

        } else {
            robot.omniArm.operateModeSwitch(gamepad2.x);
        }
        telemetry.addData("Switch Current Pos", robot.omniArm.switchMotor.getCurrentPosition());
        telemetry.addData("Switch Target Pos", robot.omniArm.switchMotor.getTargetPosition());
      //  telemetry.addData("Distance Sensor Distance ", robot.lift.distancer.getDistance(DistanceUnit.MM));
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
        i++;
        telemetry.addData("i", i);
    }
}
