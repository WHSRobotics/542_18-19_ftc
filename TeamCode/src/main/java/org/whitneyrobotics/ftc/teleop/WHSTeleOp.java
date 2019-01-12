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
        if (gamepad1.left_bumper) {
            robot.drivetrain.operateWithOrientation(gamepad1.left_stick_y / 2.45, gamepad1.right_stick_y / 2.45);
        } else {
            robot.drivetrain.operateWithOrientation(gamepad1.left_stick_y, gamepad1.right_stick_y);
        }
        robot.drivetrain.switchOrientation(gamepad1.a);

        robot.omniArm.operateIntake(gamepad2.right_bumper, gamepad2.left_bumper);
        robot.omniArm.operateSweepServos(gamepad2.right_trigger > 0.01, gamepad2.left_trigger > 0.01);

        armTog.changeState(gamepad2.dpad_up);
        if (armTog.currentState() == 0) {
            robot.omniArm.limitSwitchReset(gamepad2.y);     //This doesn't work in non-override mode
            if(gamepad2.left_stick_button) {
                robot.omniArm.setSwitchMotorPower(gamepad2.left_stick_y);
            }else {
                robot.omniArm.setExtendMotorPower(0);
            }
            if (gamepad2.right_stick_button) {
                robot.omniArm.setExtendMotorPower(gamepad2.right_stick_y);
            }else {
                robot.omniArm.setSwitchMotorPower(0);
            }
            robot.omniArm.limitSwitchReset(gamepad2.b);

        } else if (armTog.currentState() == 1) {
            robot.omniArm.resetEncoders();
            armTog.setState(2);
        } else if (armTog.currentState() == 2) {
            robot.omniArm.operateModeSwitch(gamepad2.x);
            robot.omniArm.operateExtension(gamepad2.a);

        }

        //robot.lift.sensorLift(gamepad1.y);b
        //telemetry.addData("Sensor Lift", gamepad1.y);
        //telemetry.addData("Lift State", robot.lift.getSensorLiftState());

        liftTog.changeState(gamepad2.dpad_right);
        if (liftTog.currentState() == 0) {
            if (gamepad1.dpad_up) {
                robot.lift.setLiftMotorPower(0.8);
            } else if (gamepad1.dpad_down) {
                robot.lift.setLiftMotorPower(-0.66);
            } else {
                robot.lift.setLiftMotorPower(0.0);
            }
        } else if (liftTog.currentState() == 1){
            robot.lift.liftUpRobot(gamepad1.dpad_right);
           // robot.lift.bringDownHook(gamepad1.dpad_left);
        }

        telemetry.addData("Switch Current Pos", robot.omniArm.switchMotor.getCurrentPosition());
        telemetry.addData("Switch Target Pos", robot.omniArm.switchMotor.getTargetPosition());
        telemetry.addData("Current OmniArm Mode", armTog.currentState());
        telemetry.addData("Current Lift Tog State", liftTog.currentState());
        telemetry.addData("Current LimitSwitch reset state", robot.omniArm.omniArmLimitSwitchResetState);
        telemetry.addData("Current operateModeSwitch Toggler State", robot.omniArm.getOmniArmModeSwitchTogglerState());
        //telemetry.addData("Distance Sensor Distance ", robot.lift.distancer.getDistance(DistanceUnit.MM));
    }
}
