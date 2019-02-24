package org.whitneyrobotics.ftc.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.whitneyrobotics.ftc.subsys.Potentiometer;
@TeleOp (name = "PotentiometerTest", group = "TeleOp")
public class potentiometerTest extends OpMode {
Potentiometer potentiometer;
    @Override
    public void init() {
         potentiometer = new Potentiometer(hardwareMap);
    }

    @Override
    public void loop() {
        telemetry.addData("Potentiometer Output", potentiometer.getPotentiometer());
    }
}
