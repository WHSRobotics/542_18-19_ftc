package org.whitneyrobotics.ftc.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(group = "tests", name = "Gamepad Test")
public class GamepadTest extends OpMode {

    @Override
    public void init() {
        telemetry.setMsTransmissionInterval(1);
        telemetry.log().setCapacity(20);
    }

    @Override
    public void loop() {
        telemetry.addData("g", gamepad1.toString());
        telemetry.log().add(gamepad1.toString());
    }
}
