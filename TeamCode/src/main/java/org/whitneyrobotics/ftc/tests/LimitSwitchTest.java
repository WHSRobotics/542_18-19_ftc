package org.whitneyrobotics.ftc.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
@TeleOp (name = "LimitSwitchTest" ,group = "tests")

public class LimitSwitchTest extends OpMode {

    private DigitalChannel limitSwitch;

    @Override
    public void init() {
        limitSwitch = hardwareMap.digitalChannel.get("limitSwitch");

    }

    @Override
    public void loop() {
        telemetry.addData("LimitSwitch Postion", limitSwitch.getState());
    }
}
