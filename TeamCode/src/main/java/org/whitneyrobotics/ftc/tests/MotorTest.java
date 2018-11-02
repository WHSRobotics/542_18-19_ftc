package org.whitneyrobotics.ftc.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by ivanm on 10/27/2018.
 */
@TeleOp(name = "MotorTest", group = "tests")
public class MotorTest extends OpMode {

    DcMotor motor;

    @Override
    public void init() {
         motor = hardwareMap.dcMotor.get("switchMotor");
    }

    @Override
    public void loop() {
        motor.setPower(gamepad1.right_stick_y);
        telemetry.addData("Encoder pos", motor.getCurrentPosition());
    }
}
