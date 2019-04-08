package org.whitneyrobotics.ftc.teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.whitneyrobotics.ftc.lib.util.Toggler;
import org.whitneyrobotics.ftc.subsys.MarkerDrop;
import org.whitneyrobotics.ftc.subsys.OmniArm;
import org.whitneyrobotics.ftc.subsys.ParticleFlipper;
import org.whitneyrobotics.ftc.subsys.WHSRobotImpl;


@TeleOp(name="TeleOp", group="tele")
public class WHSTeleOp extends OpMode{

    WHSRobotImpl robot;
    Toggler liftTog = new Toggler(2);
    Toggler armTog = new Toggler(2);
    double zAccel;
    double maxZAccel = 0;

    int i = 0;

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

        /* MARKER DROP */
        //So it's doesn't get in Doc Hudson's way
        robot.markerDrop.operateMarkerDrop(MarkerDrop.MarkerDropPosition.STORED);

        /* PARTICLE FLIPPER */
        // Gets rid of particles lodged in the robot
        if (robot.omniArm.clearedParticleFlipper()) {
            if (gamepad1.b) {
                robot.particleFlipper.operateParticleFlipper(ParticleFlipper.ParticleFlipperPosition.EXTENDED);
            } else {
                robot.particleFlipper.operateParticleFlipper(ParticleFlipper.ParticleFlipperPosition.STORED);
            }
        }

        /* OMNIARM */
        robot.omniArm.operateIntake(gamepad2.left_bumper, gamepad2.right_bumper, gamepad2.left_trigger>0.01);

        armTog.changeState(gamepad2.dpad_up);
        if (armTog.currentState() == 0) {
            if (gamepad2.back) {
                robot.omniArm.resetEncoders();
            }
            robot.omniArm.operateExtendManual(gamepad2.left_stick_button, gamepad2.left_stick_y);
            robot.omniArm.operatePivotManual(gamepad2.right_stick_button, gamepad2.right_stick_y);
            //robot.omniArm.limitSwitchReset(gamepad2.y);
        } else if (armTog.currentState() == 1) {
            robot.omniArm.operateClearance(gamepad2.right_trigger>0.01);
            robot.omniArm.newOperatePivot(gamepad2.x);
            //robot.omniArm.operateExtend(gamepad2.a);
            robot.omniArm.operateArmExtendBias(gamepad2.left_stick_y);
            robot.omniArm.operateArmPivotBias(gamepad2.right_stick_y >0.01, gamepad2.right_stick_y < -0.01);
        }

        /* LIFT  */
        liftTog.changeState(gamepad2.dpad_right);
        if (liftTog.currentState() == 0) {
            if (gamepad1.dpad_up) {
                robot.lift.setLiftMotorPower(0.9);
            } else if (gamepad1.dpad_down) {
                robot.lift.setLiftMotorPower(-0.9);
            } else {
                robot.lift.setLiftMotorPower(0.0);
            }
        } else if (liftTog.currentState() == 1){
            robot.lift.liftUpRobot(gamepad1.dpad_right);
           // robot.lift.bringDownHook(gamepad1.dpad_left);
        }

        zAccel = robot.imu.getZAcceleration();
        if (Math.abs(zAccel) > Math.abs(maxZAccel)) {
            maxZAccel = zAccel;
        }

        telemetry.addData("Current Z accel:", zAccel);
        telemetry.addData("Max Z accel:", maxZAccel);

        telemetry.addData("operatePivotState", robot.omniArm.operatePivotState);
        /*
        telemetry.addData("Pivot Current Pos", robot.omniArm.pivotMotor.getCurrentPosition());
        telemetry.addData("Pivot Target Pos", robot.omniArm.pivotMotor.getTargetPosition());
        telemetry.addData("OmniArm Mode", armTog.currentState());
        telemetry.addData("Lift Tog State", liftTog.currentState());
        telemetry.addData("LimitSwitch resetPivot state", robot.omniArm.limitSwitchResetState);
        */
        i++;
        telemetry.addData("i", i);
        //telemetry.addData("operatePivot Toggler State", robot.omniArm.getPivotTogglerState());
        //telemetry.addData("Distance Sensor Distance ", robot.lift.distancer.getDistance(DistanceUnit.MM));
    }
}
