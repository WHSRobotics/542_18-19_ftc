package org.whitneyrobotics.ftc.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.whitneyrobotics.ftc.lib.util.Toggler;
import org.whitneyrobotics.ftc.subsys.WHSRobotImpl;


@TeleOp(name="TeleOp", group="tele")
public class WHSTeleOp extends OpMode{

    WHSRobotImpl robot;
    long i = 0;
    Toggler liftTog = new Toggler(2);
    Toggler armTog = new Toggler(3);

    @Override
    public void init() {
        robot = new WHSRobotImpl(hardwareMap);
    }

    @Override
    public void loop() {

        /* DRIVETRAIN */
        //Precision driving mode
        if (gamepad1.left_stick_button && gamepad1.right_stick_button) {
            robot.drivetrain.operateWithOrientation(gamepad1.left_stick_y / 2.45, gamepad1.right_stick_y / 2.45);
        } else {
            robot.drivetrain.operateWithOrientation(gamepad1.left_stick_y, gamepad1.right_stick_y);
        }
        robot.drivetrain.switchOrientation(gamepad1.a);

        robot.omniArm.operateIntake(gamepad2.right_bumper, gamepad2.left_bumper);
        //robot.omniArm.operateSweepServos(gamepad2.right_trigger > 0.01, gamepad2.left_trigger > 0.01);
        robot.omniArm.operateExtension(gamepad2.a);
        //robot.lift.sensorLift(gamepad1.y);
        //telemetry.addData("Sensor Lift", gamepad1.y);
        //telemetry.addData("Lift State", robot.lift.getSensorLiftState());

        armTog.changeState(gamepad2.right_trigger > 0.01);
        robot.omniArm.limitSwitchReset(gamepad2.y);
        if (armTog.currentState() == 0) {
            if (gamepad2.dpad_up) {
                robot.omniArm.setSwitchMotorPower(0.25);
            } else if (gamepad2.dpad_down) {
                robot.omniArm.setSwitchMotorPower(-0.25);
            } else {
                robot.omniArm.setSwitchMotorPower(0.0);
            }
            if (gamepad2.dpad_left) {
                robot.omniArm.setExtendMotorPower(0.25);
            } else if (gamepad2.dpad_right) {
                robot.omniArm.setExtendMotorPower(-0.25);
            } else {
                robot.omniArm.setExtendMotorPower(0.0);
            }
        } else if (armTog.currentState() == 1) {
            robot.omniArm.resetEncoders();
            armTog.changeState(true);
        } else if (armTog.currentState() == 2) {
            if (gamepad1.left_bumper) {
                robot.omniArm.storeOmniArm(gamepad1.right_bumper);
            } else {
                robot.omniArm.operateModeSwitch(gamepad2.x);
            }
        }

        telemetry.addData("Switch Current Pos", robot.omniArm.switchMotor.getCurrentPosition());
        telemetry.addData("Switch Target Pos", robot.omniArm.switchMotor.getTargetPosition());
        telemetry.addData("Current OmniArm Mode", armTog.currentState());
        telemetry.addData("Current LimitSwitch reset state", robot.omniArm.omniArmLimitSwitchResetState);
        //telemetry.addData("Distance Sensor Distance ", robot.lift.distancer.getDistance(DistanceUnit.MM));

        liftTog.changeState(gamepad1.left_trigger > 0.01);
        if (liftTog.currentState() == 0) {
            if (gamepad1.dpad_up) {
                robot.lift.setLiftMotorPower(0.6);
            } else if (gamepad1.dpad_down) {
                robot.lift.setLiftMotorPower(-0.66);
            } else {
                robot.lift.setLiftMotorPower(0.0);
            }
        } else if (liftTog.currentState() == 1){
            robot.lift.liftUpRobot(gamepad1.dpad_right);
            robot.lift.bringDownHook(gamepad1.dpad_left);
        }
    }
}
