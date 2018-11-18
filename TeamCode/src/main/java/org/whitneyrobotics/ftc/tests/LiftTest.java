package org.whitneyrobotics.ftc.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.whitneyrobotics.ftc.lib.util.Toggler;
import org.whitneyrobotics.ftc.subsys.Lift;
import org.whitneyrobotics.ftc.subsys.WHSRobotImpl;

@TeleOp(name = "LiftTest", group = "tests")
public class LiftTest extends OpMode {

    Lift lift;
    Toggler testModeTog = new Toggler(2);
    boolean isLiftResetting = false;

    @Override
    public void init() {
        lift = new Lift(hardwareMap);
        telemetry.log().add("lift.bringDownRobot(gamepad1.a);\n" +
                "            lift.bringDownHook(gamepad1.b);\n" +
                "            lift.liftUpRobot(gamepad1.y);");
    }

    @Override
    public void loop() {
        testModeTog.changeState(gamepad1.x);
        if (testModeTog.currentState() == 0) {
            if (gamepad1.dpad_up) {
                lift.setLiftMotorPower(0.5);
            }
            else if (gamepad1.dpad_down && !lift.getDigitalTouch()) {
                lift.setLiftMotorPower(-0.5);
            } else {
                lift.setLiftMotorPower(0.0);
            }
        }
        else if (testModeTog.currentState() == 1){
            lift.bringDownRobot(gamepad1.a);
            lift.bringDownHook(gamepad1.b);
            lift.liftUpRobot(gamepad1.y);
        }

        telemetry.addData("UNIVERSALLiftState", lift.getLiftState());
        telemetry.addData("Test State", testModeTog.currentState()==0 ? "manual" : "other");
        telemetry.addData("Motor Target Pos", lift.getTargetPos());
        telemetry.addData("Motor Current Pos", lift.getEncoderPos());
        telemetry.addData("LimitSwitchState", lift.getDigitalTouch());
    }

}
